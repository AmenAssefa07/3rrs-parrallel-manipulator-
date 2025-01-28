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

        # Publisher for ball position and velocity
        self.ball_pub = rospy.Publisher('/ball_position', Vector3, queue_size=1)

        # Initialize Picamera2 with error handling
        self.picam2 = Picamera2()
        try:
            camera_config = self.picam2.create_preview_configuration(
                main={"size": (1000, 1000)},
                controls={"FrameRate":120}
                )
            self.picam2.configure(camera_config)
            self.picam2.start()
            rospy.loginfo("Camera initialized successfully.")
        except Exception as e:
            rospy.logerr(f"Camera initialization failed: {e}")
            exit(1)

        # State variables
        self.prev_position = None
        self.prev_time = rospy.Time.now()
        self.position_history = deque(maxlen=5)

        # Control display
        self.visual_display = False  # Toggle to enable/disable display
        rospy.loginfo(f"Display: {self.visual_display}")

    def detect_ball(self, frame):
        """Detect the ball using HSV color segmentation."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 150, 0]), np.array([180, 255, 255]))

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius > 10:
                #rospy.loginfo(f"Ball detected at: {int(x)}, {int(y)} with radius: {radius}")
                return (int(x), int(y)), frame
        rospy.logwarn("No ball detected.")
        return None, frame

    def apply_moving_average(self, position):
        """Smooth the position using a moving average filter."""
        self.position_history.append(position)
        avg = np.mean(self.position_history, axis=0)
        return int(avg[0]), int(avg[1])

    def calculate_velocity(self, position):
        """Calculate the velocity vector."""
        now = rospy.Time.now()
        dt = (now - self.prev_time).to_sec()
        if self.prev_position and dt > 0:
            vx = (position[0] - self.prev_position[0]) / dt
            vy = (position[1] - self.prev_position[1]) / dt
        else:
            vx, vy = 0.0, 0.0

        self.prev_position, self.prev_time = position, now
        return vx, vy

    def run(self):
        """Main loop for real-time ball tracking."""
        rate = rospy.Rate(60)  # Higher frame rate

        while not rospy.is_shutdown():
            frame = self.picam2.capture_array()
            position, processed_frame = self.detect_ball(frame)

            if position:
                smoothed_position = self.apply_moving_average(position)
                vx, vy = self.calculate_velocity(smoothed_position)

                # Publish position and velocity
                msg = Vector3(x=smoothed_position[0], y=smoothed_position[1], z=np.hypot(vx, vy))
                self.ball_pub.publish(msg)
                #
                # rospy.loginfo(f"Published: {msg}")

                if self.visual_display==True:
                    # Draw the detected ball and velocity vector
                    
                    cv2.namedWindow("Ball Tracker", cv2.WINDOW_NORMAL)  # Allow resizing
                    
                    cv2.resizeWindow("Ball Tracker", 700, 700)
                    cv2.circle(processed_frame, smoothed_position, 5, (0, 255, 0), -1)
                    text = f"Pos: {smoothed_position}, Vel: ({vx:.2f}, {vy:.2f})"
                    cv2.putText(processed_frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    # Show the frame
                    cv2.imshow("Ball Tracker", processed_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        rospy.loginfo("Exiting visual display.")
                        break

            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tracker = BallTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
