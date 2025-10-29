"""import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2 as cv


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.publisher_ = self.create_publisher(Image, '/video_data', 10)
        self.video_name = 'lab3_video.avi'
        self.play_video(self.video_name)
        self.save_video(self.video_name)

    def play_video(self, video_name):
        cap = cv.VideoCapture(video_name)
 
        while cap.isOpened():
            ret, frame = cap.read()
        
            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
            cv.imshow('frame', gray)
            if cv.waitKey(1) == ord('q'):
                break
        
        cap.release()
        cv.destroyAllWindows()

    def save_video(self, video_name):
        cap = cv.VideoCapture(0)
 
        # Define the codec and create VideoWriter object
        fourcc = cv.VideoWriter_fourcc(*'XVID')
        out = cv.VideoWriter(video_name, fourcc, 20.0, (640,  480))
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            frame = cv.flip(frame, 0)
        
            # write the flipped frame
            out.write(frame)
        
            cv.imshow('frame', frame)
            if cv.waitKey(1) == ord('q'):
                break
        
        # Release everything if job is finished
        cap.release()
        out.release()
        cv.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import os

from ament_index_python.packages import get_package_share_directory


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Publisher for raw color images
        self.publisher_ = self.create_publisher(Image, '/video_data', 10)
        self.bridge = CvBridge()

        pkg_share = get_package_share_directory('task_5')
        video_path = os.path.join(pkg_share, 'resource', 'lab3_video.avi')


        # Try to open the video
        self.cap = cv.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open video file: {video_path}')
            return

        self.get_logger().info('Publishing video frames from lab3_video.avi...')
        self.timer = self.create_timer(0.05, self.publish_frame)  # ~20 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('End of video stream reached.')
            self.cap.release()
            rclpy.shutdown()
            return

        # Convert OpenCV image (BGR) to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published one frame.')

        # Optional: show frame while publishing
        cv.imshow('Published Video', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
