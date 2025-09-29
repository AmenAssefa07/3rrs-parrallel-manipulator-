#!/usr/bin/env python3
# TODOs (known issues not yet fixed):
# 1) NoIR camera white balance / color gains not handled (colors can drift). Consider setting AwbMode/ColourGains.
# 2) Camera is not explicitly stopped on shutdown. Add self.picam2.stop() in a finally/on_shutdown.
# 3) Velocity can spike on tiny dt / jitter. Add a dt floor and/or smooth vx, vy.
# 4) Using Vector3 with speed in z is ambiguous. Consider publishing separate position/velocity topics or a custom msg.
# 5) No morphological cleanup on mask. Add OPEN/CLOSE to reduce speckles/holes.
# 6) Fixed radius threshold may fail at different scales. Consider area/circularity checks or adaptive thresholds.
# 7) Requested frame rate may not be honored as-is. Verify with proper sensor mode / FrameDurationLimits.
# 8) No warm-up after camera start for AWB/exposure. Sleep briefly so stats settle.
# 9) apply_moving_average casts via int(), which floors. Consider round() for less bias.
# 10) Robustness: guard against None/invalid frames from capture_array().
# 11) (Optional future) Restrict search to ROI near last known position to cut latency.
# 12) (Optional future) Use RETR_EXTERNAL for contours; current RETR_TREE is unnecessary.
# 13) (Optional future) Consider async capture / producer-consumer to reduce blocking latency.

# CURRENT VERSION (v0.2) — changes from previous:
# - Color pipeline: Removed redundant conversions. We now convert RGB->HSV for detection.
#   Only when visual_display is True do we convert RGB->BGR for imshow/overlay.
# - Window management: Create & size the OpenCV window once (before the loop) if display is enabled.
# - Resolution: Dropped main stream to 640x640 to reduce per-frame processing cost.
# - Logging: "No ball detected" is throttled to once every 0.5 seconds.
# - (Kept) Two-range HSV mask for red hue wraparound.

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
            # Fix #3: reduce resolution to 640x640
            camera_config = self.picam2.create_preview_configuration(
                main={"size": (640, 640)},
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

        # Fix #2: if displaying, create/size window once (not every frame)
        if self.visual_display:
            cv2.namedWindow("Ball Tracker", cv2.WINDOW_NORMAL)  # Allow resizing
            cv2.resizeWindow("Ball Tracker", 700, 700)

    def detect_ball(self, hsv_frame, display_frame_for_return):
        """
        Detect the ball using HSV color segmentation (two red ranges wrapping hue seam).
        hsv_frame: HSV image (converted from RGB outside).
        display_frame_for_return: the frame we'll return for drawing (BGR if displaying, else RGB).
        """
        # Two-range red mask (tune for your lighting/camera)
        lower1 = np.array([0, 120, 80], dtype=np.uint8)
        upper1 = np.array([10, 255, 255], dtype=np.uint8)
        lower2 = np.array([170, 120, 80], dtype=np.uint8)
        upper2 = np.array([180, 255, 255], dtype=np.uint8)

        mask1 = cv2.inRange(hsv_frame, lower1, upper1)
        mask2 = cv2.inRange(hsv_frame, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius > 10:
                # rospy.loginfo(f"Ball detected at: {int(x)}, {int(y)} with radius: {radius}")
                return (int(x), int(y)), display_frame_for_return

        # Fix #4: throttle logs to once per 0.5 seconds
        rospy.logwarn_throttle(0.5, "No ball detected.")
        return None, display_frame_for_return

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
        rate = rospy.Rate(60)  # Target processing rate

        while not rospy.is_shutdown():
            # Capture RGB frame
            frame_rgb = self.picam2.capture_array()

            # Fix #1: Avoid extra conversions.
            # - For detection: convert RGB -> HSV
            hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

            # - For display: only convert to BGR if visual_display is on
            if self.visual_display:
                frame_for_display = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            else:
                frame_for_display = frame_rgb  # won't be shown; passed through for API consistency

            position, processed_frame = self.detect_ball(hsv, frame_for_display)

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

                if self.visual_display:
                    # Draw overlays on the BGR frame
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
