#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.subscriber_ = self.create_subscription(Image, '/video_data', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(BoundingBox2D, '/bbox', 10)

        # Bridge between ROS Image msg and OpenCV
        self.bridge = CvBridge()
        self.get_logger().info('Object Detector Node has started...')

    def listener_callback(self, msg: Image):
        print("listener callback")
        # Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Use BGR image to HSV image
        # hsv = [hue, saturation, brightness]
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Red hue 0-10
        lower = np.array([0, 120, 130]) # 0, 120, 70
        upper = np.array([10, 255, 255]) #10, 255, 255
        mask1 = cv.inRange(hsv, lower, upper)

        # Red hue 170-180
        lower = np.array([170, 120, 130])
        upper = np.array([180, 255, 255])
        mask2 = cv.inRange(hsv, lower, upper)

        mask = mask1 | mask2

        # Remove noise and fill gaps
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv.morphologyEx(mask, cv.MORPH_DILATE, np.ones((5, 5), np.uint8))

        # Find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Look for the triangular shape above a size threshold 
        triangles = []
        for c in contours:
            area = cv.contourArea(c)
            if area < 300:
                continue

            peri = cv.arcLength(c, True)
            approx = cv.approxPolyDP(c, 0.04 * peri, True)

            if len(approx) == 3:
                triangles.append(c)

        if len(triangles) > 0:
            c = max(triangles, key=cv.contourArea)
            x, y, w, h = cv.boundingRect(c)
            cx = int(x + w/2)
            cy = int(y + h/2)


            # Publish bounding box info
            bbox_msg = BoundingBox2D()
            bbox_msg.center.position.x = float(cx)
            bbox_msg.center.position.y = float(cy)
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)
            self.publisher_.publish(bbox_msg)

            # Print details
            self.get_logger().info(f"Centroid: ({cx}, {cy}) | Size: ({w}, {h})")

            # Draw bounding box
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
        else:
            self.get_logger().info("No object detected.")

        # Display edited video with bounding box
        cv.imshow('Detected Object', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()