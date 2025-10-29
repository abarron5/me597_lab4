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
        #self.subscriber_  # prevent unused variable warning

        # Publisher for bounding box information
        self.publisher_ = self.create_publisher(BoundingBox2D, '/bbox', 10)

        # Bridge between ROS Image msg and OpenCV
        self.bridge = CvBridge()
        self.get_logger().info('Object Detector Node has started...')

    def listener_callback(self, msg: Image):
        print("listener callback")
        # Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # -----------------------------------------------------
        # 1️⃣ Simple detection logic (color segmentation)
        #    Here we look for a bright colored object (e.g., red)
        # -----------------------------------------------------
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Define color range for detection (tune as needed)
        lower = np.array([0, 120, 70])
        upper = np.array([10, 255, 255])
        mask1 = cv.inRange(hsv, lower, upper)

        # for red hue wraparound (170-180)
        lower = np.array([170, 120, 70])
        upper = np.array([180, 255, 255])
        mask2 = cv.inRange(hsv, lower, upper)

        mask = mask1 | mask2

        # Clean up mask
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv.morphologyEx(mask, cv.MORPH_DILATE, np.ones((5, 5), np.uint8))

        # -----------------------------------------------------
        # 2️⃣ Find contours
        # -----------------------------------------------------
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Choose the largest contour as the detected object
            c = max(contours, key=cv.contourArea)
            x, y, w, h = cv.boundingRect(c)

            # Compute centroid
            cx = int(x + w / 2)
            cy = int(y + h / 2)

            # -------------------------------------------------
            # 3️⃣ Publish bounding box info
            # -------------------------------------------------
            bbox_msg = BoundingBox2D()
            bbox_msg.center.position.x = float(cx)
            bbox_msg.center.position.y = float(cy)
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)
            self.publisher_.publish(bbox_msg)

            # Print details
            self.get_logger().info(f"Centroid: ({cx}, {cy}) | Size: ({w}, {h})")

            # -------------------------------------------------
            # 4️⃣ Draw bounding box on frame
            # -------------------------------------------------
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv.putText(frame, f"({cx},{cy})", (x, y - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            self.get_logger().info("No object detected.")

        # -----------------------------------------------------
        # 5️⃣ Show the processed frame
        # -----------------------------------------------------
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