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

        self.publisher_ = self.create_publisher(Image, '/video_data', 10)
        self.bridge = CvBridge()

        pkg_share = get_package_share_directory('task_5')
        video_path = os.path.join(pkg_share, 'resource', 'lab3_video.avi')

        # Open the video
        self.cap = cv.VideoCapture(video_path)

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


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
