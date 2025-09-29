#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from picamera2 import Picamera2
from geometry_msgs.msg import Vector3

class BallTracker:
    def __init__(self):
        rospy.init_node('ball_tracker', log_level=rospy.INFO)
        
        # Configuration parameters
        self.visual_display = True
        self.show_hsv_mask = True
        self.resolution = (640, 480)
        
        # HSV calibration parameters (initial values)
        self.hsv_lower1 = np.array([0, 120, 70])
        self.hsv_upper1 = np.array([10, 255, 255])
        self.hsv_lower2 = np.array([170, 120, 70])
        self.hsv_upper2 = np.array([180, 255, 255])
        
        # Camera initialization
        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={"size": self.resolution, "format": "RGB888"},
            controls={"FrameDurationLimits": (8333, 8333)}  # 120 FPS
        )
        self.camera.configure(config)
        self.camera.start()
        
        # ROS publisher
        self.pub = rospy.Publisher('/ball_data', Vector3, queue_size=1)
        
        # Tracking variables
        self.prev_pos = None
        self.prev_time = rospy.Time.now()
        self.kalman = cv2.KalmanFilter(4, 2)
        self._init_kalman()
        
        # Initialize debug windows if enabled
        if self.visual_display:
            self._init_debug_windows()

    def _init_kalman(self):
        """Initialize Kalman filter matrices"""
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kalman.transitionMatrix = np.array([
            [1,0,1,0],
            [0,1,0,1],
            [0,0,1,0],
            [0,0,0,1]
        ], np.float32)
        self.kalman.processNoiseCov = 1e-4 * np.eye(4, dtype=np.float32)
        self.kalman.measurementNoiseCov = 1e-2 * np.eye(2, dtype=np.float32)

    def _init_debug_windows(self):
        """Create debug windows and trackbars"""
        cv2.namedWindow("Ball Tracker", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Ball Tracker", 700, 700)
        cv2.namedWindow("HSV Calibration")
        
        # Create trackbars for HSV calibration
        cv2.createTrackbar("H1", "HSV Calibration", 0, 180, lambda x: None)
        cv2.createTrackbar("S1", "HSV Calibration", 120, 255, lambda x: None)
        cv2.createTrackbar("V1", "HSV Calibration", 70, 255, lambda x: None)
        cv2.createTrackbar("H2", "HSV Calibration", 170, 180, lambda x: None)
        cv2.createTrackbar("S2", "HSV Calibration", 120, 255, lambda x: None)
        cv2.createTrackbar("V2", "HSV Calibration", 70, 255, lambda x: None)
        
        # Set initial trackbar positions
        cv2.setTrackbarPos("H1", "HSV Calibration", 0)
        cv2.setTrackbarPos("S1", "HSV Calibration", 120)
        cv2.setTrackbarPos("V1", "HSV Calibration", 70)
        cv2.setTrackbarPos("H2", "HSV Calibration", 170)
        cv2.setTrackbarPos("S2", "HSV Calibration", 120)
        cv2.setTrackbarPos("V2", "HSV Calibration", 70)

    def _update_hsv_values(self):
        """Update HSV values from trackbars"""
        self.hsv_lower1 = np.array([
            cv2.getTrackbarPos("H1", "HSV Calibration"),
            cv2.getTrackbarPos("S1", "HSV Calibration"),
            cv2.getTrackbarPos("V1", "HSV Calibration")
        ])
        self.hsv_upper2 = np.array([
            cv2.getTrackbarPos("H2", "HSV Calibration"),
            255,
            255
        ])

    def _process_mask(self, mask):
        """Clean up detection mask with morphological operations"""
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        return mask

    def _detect_ball(self, frame):
        """Main detection pipeline"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Update HSV values from trackbars
        if self.visual_display:
            self._update_hsv_values()
        
        # Create HSV masks
        mask1 = cv2.inRange(hsv, self.hsv_lower1, self.hsv_upper1)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        combined_mask = cv2.bitwise_or(mask1, mask2)
        processed_mask = self._process_mask(combined_mask)
        
        # Show HSV mask if enabled
        if self.visual_display and self.show_hsv_mask:
            cv2.imshow("HSV Mask", processed_mask)
        
        # Find and filter contours
        contours, _ = cv2.findContours(processed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        valid_contours = []
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 500 < area < 10000:
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = 4 * np.pi * area / (perimeter ** 2)
                if circularity > 0.6:
                    valid_contours.append(cnt)
        
        if not valid_contours:
            return None
            
        # Find largest valid contour
        largest_contour = max(valid_contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        return (int(x), int(y), int(radius))

    def run(self):
        """Main processing loop"""
        try:
            while not rospy.is_shutdown():
                # Capture frame
                frame = self.camera.capture_array()
                current_time = rospy.Time.now()
                
                # Process detection
                detection = self._detect_ball(frame)
                processed_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if self.visual_display else None
                
                if detection:
                    x, y, radius = detection
                    
                    # Kalman filter update
                    prediction = self.kalman.predict()
                    measured = np.array([[x], [y]], dtype=np.float32)
                    state = self.kalman.correct(measured)
                    
                    # Calculate velocity
                    dt = (current_time - self.prev_time).to_sec()
                    if self.prev_pos and dt > 0:
                        vx = (state[0][0] - self.prev_pos[0]) / dt
                        vy = (state[1][0] - self.prev_pos[1]) / dt
                    else:
                        vx, vy = 0.0, 0.0
                    
                    speed = np.hypot(vx, vy)
                    
                    # Update previous values
                    self.prev_pos = (state[0][0], state[1][0])
                    self.prev_time = current_time
                    
                    # Publish data
                    msg = Vector3()
                    msg.x = state[0][0]
                    msg.y = state[1][0]
                    msg.z = speed
                    self.pub.publish(msg)
                    
                    # Update debug display
                    if self.visual_display:
                        cv2.circle(processed_frame, (int(state[0][0]), int(state[1][0])), 
                                  5, (0, 255, 0), -1)
                        cv2.putText(processed_frame,
                                  f"Pos: ({state[0][0]:.1f}, {state[1][0]:.1f}) Vel: ({vx:.1f}, {vy:.1f})",
                                  (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                else:
                    self._init_kalman()
                    self.prev_pos = None
                
                # Handle display
                if self.visual_display:
                    cv2.imshow("Ball Tracker", processed_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        rospy.loginfo("Exiting visual display.")
                        break
        
        finally:
            cv2.destroyAllWindows()
            self.camera.stop()
            rospy.loginfo("Camera resources released.")

if __name__ == '__main__':
    tracker = BallTracker()
    try:
        tracker.run()
    except rospy.ROSInterruptException:
        pass
