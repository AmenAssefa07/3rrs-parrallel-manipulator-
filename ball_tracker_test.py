#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from picamera2 import Picamera2
from geometry_msgs.msg import Vector3

class BallTracker:
    def __init__(self):
        rospy.init_node('ball_tracker', log_level=rospy.INFO)
        
        # Visual debug and calibration controls
        self.visual_display = True
        self.show_hsv_mask = True  # Toggle to see what the camera detects
        
        # Initial HSV ranges (adjust these values based on your ball)
        self.hsv_lower1 = np.array([0, 120, 70])    # Lower red range
        self.hsv_upper1 = np.array([10, 255, 255])
        self.hsv_lower2 = np.array([170, 120, 70])  # Upper red range
        self.hsv_upper2 = np.array([180, 255, 255])
        
        # Camera setup
        self.resolution = (640, 480)
        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={"size": self.resolution, "format": "RGB888"},
            controls={"FrameDurationLimits": (8333, 8333)}
        )
        self.camera.configure(config)
        self.camera.start()
        
        # Publisher setup
        self.pub = rospy.Publisher('/ball_data', Vector3, queue_size=1)
        
        # Tracking variables
        self.kalman = cv2.KalmanFilter(4, 2)
        self.prev_pos = None
        self.prev_time = rospy.Time.now()
        self._init_kalman()

        # Create trackbars if debug mode enabled
        if self.visual_display:
            cv2.namedWindow("HSV Calibration")
            cv2.createTrackbar("H1", "HSV Calibration", 0, 180, self._nothing)
            cv2.createTrackbar("S1", "HSV Calibration", 120, 255, self._nothing)
            cv2.createTrackbar("V1", "HSV Calibration", 70, 255, self._nothing)
            cv2.createTrackbar("H2", "HSV Calibration", 170, 180, self._nothing)
            cv2.createTrackbar("S2", "HSV Calibration", 120, 255, self._nothing)
            cv2.createTrackbar("V2", "HSV Calibration", 70, 255, self._nothing)

    def _nothing(self, x): pass  # Dummy function for trackbars

    def _init_kalman(self):
        # ... (same Kalman initialization as before) ...

    def _detect_ball(self, frame):
        """Improved ball detection with dynamic HSV calibration"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Update HSV values from trackbars in real-time
        if self.visual_display:
            self.hsv_lower1 = np.array([
                cv2.getTrackbarPos("H1", "HSV Calibration"),
                cv2.getTrackbarPos("S1", "HSV Calibration"),
                cv2.getTrackbarPos("V1", "HSV Calibration")
            ])
            self.hsv_upper2 = np.array([
                cv2.getTrackbarPos("H2", "HSV Calibration"),
                255, 255  # Keep upper bounds at max
            ])
        
        # Create masks with current thresholds
        mask1 = cv2.inRange(hsv, self.hsv_lower1, self.hsv_upper1)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        combined_mask = cv2.bitwise_or(mask1, mask2)
        
        # Noise reduction
        kernel = np.ones((5,5), np.uint8)
        combined_mask = cv2.erode(combined_mask, kernel, iterations=1)
        combined_mask = cv2.dilate(combined_mask, kernel, iterations=2)
        
        # Show HSV mask for debugging
        if self.show_hsv_mask and self.visual_display:
            cv2.imshow("HSV Mask", combined_mask)
        
        # Contour processing
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        if not contours:
            return None
            
        # Filter by size and circularity
        valid = []
        for c in contours:
            area = cv2.contourArea(c)
            if 500 < area < 10000:  # Adjusted size range
                perimeter = cv2.arcLength(c, True)
                if perimeter == 0: continue
                circularity = 4 * np.pi * area / (perimeter ** 2)
                if circularity > 0.6:  # Require round shapes
                    valid.append(c)
        
        if not valid:
            return None
            
        c = max(valid, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)
        return (int(x), int(y), radius)

    def run(self):
        # ... (rest of run() method remains the same) ...

if __name__ == '__main__':
    tracker = BallTracker()
    try:
        tracker.run()
    except rospy.ROSInterruptException:
        pass
