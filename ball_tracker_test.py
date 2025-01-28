#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from picamera2 import Picamera2
from geometry_msgs.msg import Vector3

class BallTracker:
    def __init__(self):
        rospy.init_node('ball_tracker', log_level=rospy.INFO)
        
        # Camera configuration for max performance
        self.resolution = (640, 480)  # Optimal for high FPS
        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={"size": self.resolution, "format": "RGB888"},
            controls={"FrameDurationLimits": (8333, 8333)}  # 120 FPS target
        )
        self.camera.configure(config)
        self.camera.start()
        
        # Publisher for combined data
        self.pub = rospy.Publisher('/ball_data', Vector3, queue_size=1)
        
        # Tracking variables
        self.kalman = cv2.KalmanFilter(4, 2)
        self.prev_pos = None
        self.prev_time = rospy.Time.now()
        self._init_kalman()

    def _init_kalman(self):
        """Kalman filter initialization for smooth tracking"""
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kalman.transitionMatrix = np.array([
            [1,0,1,0],
            [0,1,0,1],
            [0,0,1,0],
            [0,0,0,1]
        ], np.float32)
        self.kalman.processNoiseCov = 1e-4 * np.eye(4, dtype=np.float32)
        self.kalman.measurementNoiseCov = 1e-2 * np.eye(2, dtype=np.float32)

    def _detect_ball(self, frame):
        """Optimized ball detection with dual-range HSV thresholding"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Red color detection with wrap-around handling
        mask1 = cv2.inRange(hsv, np.array([0, 150, 50]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 150, 50]), np.array([180, 255, 255]))
        combined_mask = cv2.bitwise_or(mask1, mask2)
        
        # Contour processing
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        if not contours:
            return None
            
        # Size-based filtering
        valid = [c for c in contours if 100 < cv2.contourArea(c) < 10000]
        if not valid:
            return None
            
        c = max(valid, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)
        return (int(x), int(y), radius)

    def run(self):
        """Main tracking loop with minimal latency"""
        while not rospy.is_shutdown():
            try:
                # Frame capture with timestamp
                frame = self.camera.capture_array()
                current_time = rospy.Time.now()
                
                # Detection pipeline
                detection = self._detect_ball(frame)
                
                if detection:
                    x, y, radius = detection
                    
                    # Kalman filter update
                    prediction = self.kalman.predict()
                    measured = np.array([[x], [y]], dtype=np.float32)
                    state = self.kalman.correct(measured)
                    
                    # Velocity calculation
                    dt = (current_time - self.prev_time).to_sec()
                    vx = (state[0][0] - self.prev_pos[0])/dt if self.prev_pos else 0.0
                    vy = (state[1][0] - self.prev_pos[1])/dt if self.prev_pos else 0.0
                    speed = np.hypot(vx, vy)
                    
                    # Update previous values
                    self.prev_pos = (state[0][0], state[1][0])
                    self.prev_time = current_time
                    
                    # Publish combined data
                    msg = Vector3()
                    msg.x = state[0][0]
                    msg.y = state[1][0]
                    msg.z = speed
                    self.pub.publish(msg)

                else:
                    self._init_kalman()
                    self.prev_pos = None

            except Exception as e:
                rospy.logerr(f"Processing error: {str(e)}")
                continue

if __name__ == '__main__':
    tracker = BallTracker()
    try:
        tracker.run()
    except rospy.ROSInterruptException:
        tracker.camera.stop()
