#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from picamera2 import Picamera2
from geometry_msgs.msg import Vector3
from collections import deque

class BallTracker:
    def __init__(self):
        rospy.init_node('ball_tracker', anonymous=True)

        # Publisher for combined position and velocity data
        self.ball_pub = rospy.Publisher('/ball_position', Vector3, queue_size=10)

        # Initialize Picamera2
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(
            main={"size": (1920, 1080)}
        )
        self.picam2.configure(camera_config)
        self.picam2.start()
        rospy.loginfo("Camera initialized.")

        # Store previous position and time for velocity calculation
        self.prev_position = None
        self.prev_time = rospy.Time.now()

        # Use a deque to implement a moving average filter for position
        self.position_history = deque(maxlen=5)  # Store the last 5 positions

        # Set visual display option
        self.visual_display = True  # Toggle visual display on/off

    def detect_ball(self, frame):
        """Detect the red ball using color segmentation."""
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Define HSV range for red color (adjust as needed)
        lower_red = np.array([0, 149, 0])
        upper_red = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour (assume it's the ball)
            c = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)

            if radius > 10:  # Ignore small detections
                return (int(x), int(y)), frame_bgr

        return None, frame_bgr

    def apply_moving_average(self, position):
        """Apply a moving average filter to smooth the position."""
        self.position_history.append(position)
        avg_x = np.mean([pos[0] for pos in self.position_history])
        avg_y = np.mean([pos[1] for pos in self.position_history])
        return int(avg_x), int(avg_y)

    def calculate_velocity(self, current_position):
        """Calculate the velocity vector."""
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        if self.prev_position is not None and dt > 0:
            vx = (current_position[0] - self.prev_position[0]) / dt
            vy = (current_position[1] - self.prev_position[1]) / dt
        else:
            vx, vy = 0.0, 0.0

        # Update previous position and time
        self.prev_position = current_position
        self.prev_time = current_time

        return vx, vy

    def run(self):
        """Main loop to track the ball and publish data."""
        rate = rospy.Rate(30)  # 30 Hz

        while not rospy.is_shutdown():
            # Capture frame from the camera
            frame = self.picam2.capture_array()

            # Detect the ball's position
            position, processed_frame = self.detect_ball(frame)

            if position:
                # Apply the moving average filter to the position
                smoothed_position = self.apply_moving_average(position)

                # Calculate velocity vector
                vx, vy = self.calculate_velocity(smoothed_position)

                # Publish combined position and velocity data
                msg = Vector3()
                msg.x, msg.y, msg.z = smoothed_position[0], smoothed_position[1], np.hypot(vx, vy)
                self.ball_pub.publish(msg)

                if self.visual_display:
                    # Draw the ball position and velocity on the frame
                    cv2.circle(processed_frame, smoothed_position, 5, (0, 255, 0), -1)  # Green dot
                    text = f"Pos: {smoothed_position}, Vel: ({vx:.2f}, {vy:.2f})"
                    cv2.putText(processed_frame, text, (smoothed_position[0] + 10, smoothed_position[1] + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)

            # Display the frame if visual display is enabled
            if self.visual_display:
                cv2.namedWindow("Ball Tracker", cv2.WINDOW_NORMAL)  # Allow resizing
                cv2.resizeWindow("Ball Tracker", 1280, 720)
                cv2.imshow("Ball Tracker", processed_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tracker = BallTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
