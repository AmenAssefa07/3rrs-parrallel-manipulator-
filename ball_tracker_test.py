#!/usr/bin/env python3
# TODOs (known issues not yet fixed):
# 1) NoIR camera white balance / color gains not handled (colors can drift). Consider setting AwbMode/ColourGains.
# 2) Log spam: "No ball detected" every cycle. Use rospy.logwarn_throttle to reduce noise.
# 3) OpenCV window is created/resized every frame. Create once before the loop for efficiency.
# 4) Camera is not explicitly stopped on shutdown. Add self.picam2.stop() in a finally/on_shutdown.
# 5) Velocity can spike on tiny dt / jitter. Add dt floor and/or smooth vx, vy.
# 6) Using Vector3 with speed in z is ambiguous. Consider publishing separate position/velocity topics or a custom msg.
# 7) No morphological cleanup on mask. Add OPEN/CLOSE to reduce speckles/holes.
# 8) Fixed radius threshold may fail at different scales. Consider area/circularity checks or adaptive thresholds.
# 9) Frame rate request may not be honored as-is. Verify with proper sensor mode / FrameDurationLimits.
# 10) No warm-up after camera start for AWB/exposure. Sleep briefly so stats settle.
# 11) apply_moving_average casts via int(), which floors. Consider round() for less bias.
# 12) Robustness: guard against None/invalid frames from capture_array().

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
                controls={"FrameRate": 120}
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

    def detect_ball(self, frame_bgr):
        """Detect the ball using HSV color segmentation (two red ranges wrapping hue seam)."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Two-range red mask (example defaults; tune for your lighting/camera)
        lower1 = np.array([0, 120, 80], dtype=np.uint8)
        upper1 = np.array([10, 255, 255], dtype=np.uint8)
        lower2 = np.array([170, 120, 80], dtype=np.uint8)
        upper2 = np.array([180, 255, 255], dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius > 10:
                # rospy.loginfo(f"Ball detected at: {int(x)}, {int(y)} with radius: {radius}")
                return (int(x), int(y)), frame_bgr
        rospy.logwarn("No ball detected.")
        return None, frame_bgr

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
            # --- Option A: convert RGB (from Picamera2) to BGR once and keep the rest the same ---
            frame_rgb = self.picam2.capture_array()
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            position, processed_frame = self.detect_ball(frame)

            if position:
                smoothed_position = self.apply_moving_average(position)
                vx, vy = self.calculate_velocity(smoothed_position)

                # Publish position and velocity
                msg = Vector3(
                    x=smoothed_position[0],
                    y=smoothed_position[1],
                    z=np.hypot(vx, vy)
                )
                self.ball_pub.publish(msg)
                # rospy.loginfo(f"Published: {msg}")

                if self.visual_display == True:
                    # Draw the detected ball and velocity vector
                    cv2.namedWindow("Ball Tracker", cv2.WINDOW_NORMAL)  # Allow resizing
                    cv2.resizeWindow("Ball Tracker", 700, 700)
                    cv2.circle(processed_frame, smoothed_position, 5, (0, 255, 0), -1)
                    text = f"Pos: {smoothed_position}, Vel: ({vx:.2f}, {vy:.2f})"
                    cv2.putText(processed_frame, text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

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
