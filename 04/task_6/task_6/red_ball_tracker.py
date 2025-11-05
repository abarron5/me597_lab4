#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import time


class RedBallTracker(Node):
    def __init__(self):
        super().__init__('red_ball_tracker')

        # Subscribers
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10
        )

        # Publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Bridge between ROS Image msg and OpenCV
        self.bridge = CvBridge()

        # PID controller parameters
        self.kp_ang = 0.0005    # proportional gain for angular velocity
        self.kd_ang = 0.000
        self.kp_lin = 0.00008    # proportional gain for linear velocity
        self.kd_lin = 0.000

        self.prev_error_x = 0
        self.prev_error_area = 0

        self.target_area = 50000  # desired area of red ball (tuned experimentally)
        self.last_time = time.time()

        # filtering parameters
        self.filtered_error_x = 0.0
        self.filtered_error_area = 0.0
        self.alpha = 0.2   # smoothing factor (0 = heavy smoothing, 1 = no smoothing)


        self.get_logger().info('Red Ball Tracker Node started...')

    def listener_callback(self, msg: Image):
        # Convert ROS Image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Find center of frame
        height, width, _ = frame.shape
        center_x = width // 2

        # Convert to HSV for color detection
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Define red ranges in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Combine masks for red hue wraparound
        mask1 = cv.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Clean up mask
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv.morphologyEx(mask, cv.MORPH_DILATE, np.ones((5, 5), np.uint8))

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        vel = Twist()

        if contours:
            # Choose largest contour (the ball)
            c = max(contours, key=cv.contourArea)
            area = cv.contourArea(c)

            if area > 300:
                x, y, w, h = cv.boundingRect(c)
                cx = int(x + w / 2)
                cy = int(y + h / 2)

                # Draw bounding box
                cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

                # --- PID Control ---
                error_x = center_x - cx
                error_area = self.target_area - area

                # filter error
                self.filtered_error_x = (
                    self.alpha * error_x + (1 - self.alpha) * self.filtered_error_x
                )
                self.filtered_error_area = (
                    self.alpha * error_area + (1 - self.alpha) * self.filtered_error_area
                )

                # use filtered versions for control
                error_x = self.filtered_error_x
                error_area = self.filtered_error_area

                # set error to 0 within specified margins
                if abs(error_x) < 30:
                    error_x = 0
                if abs(error_area) < self.target_area * 0.3:
                    error_area = 0

                current_time = time.time()
                dt = current_time - self.last_time if self.last_time else 0.1
                derivative_x = (error_x - self.prev_error_x) / dt if dt > 0 else 0.0
                derivative_area = (error_x - self.prev_error_x) / dt if dt > 0 else 0.0

                vel.angular.z = self.kp_ang * error_x + self.kd_ang * derivative_x
                vel.linear.x = self.kp_lin * error_area + self.kd_lin * derivative_area    # move forward/backward

                # smooth turning
                turn_factor = max(0.3, 1.0 - abs(vel.angular.z) * 3.0)
                vel.linear.x *= turn_factor

                # Clamp velocities
                vel.linear.x = np.clip(vel.linear.x, -0.25, 0.25)
                vel.angular.z = np.clip(vel.angular.z, -0.2, 0.2)

                self.prev_error_x = error_x
                self.prev_error_area = error_area
                self.last_time = current_time

                self.get_logger().info(
                    f"Ball at ({cx}, {cy}), area={area:.1f}, vel=({vel.linear.x:.2f}, {vel.angular.z:.2f})"
                )
            else:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
        else:
            # No ball detected → stop
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.get_logger().info("No ball detected: stopping.")

        # Publish command
        self.publisher_.publish(vel)

        # Show image scaled to fit nicely on screen
        cv.namedWindow("Red Ball Tracker", cv.WINDOW_NORMAL)
        cv.resizeWindow("Red Ball Tracker", 640, 480)
        cv.imshow("Red Ball Tracker", cv.resize(frame, (640, 480)))
        cv.moveWindow("Red Ball Tracker", 100, 50)
        cv.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = RedBallTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
