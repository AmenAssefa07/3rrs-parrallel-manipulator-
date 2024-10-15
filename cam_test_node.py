#!/usr/bin/env python3

from picamera2 import Picamera2
import cv2
import numpy as np
import rospy

class CamTestNode:
    def __init__(self):
        rospy.init_node('cam_test_node', anonymous=True)

        # Initialize Picamera2
        self.picam2 = Picamera2()

        # Configure the camera with desired settings
        camera_config = self.picam2.create_preview_configuration(
            main={"size": (1920, 1080)},  # Wide-angle resolution
            
            #controls={
             #   "AwbEnable": False,  # Disable auto white balance for consistency
              #  "ExposureTime": 20000,  # Set exposure time to avoid over/under-exposure
               # "AnalogueGain": 1.5,  # Adjust gain for brightness
                #"ColourGains": (2.0, 1.5)  # Optional: Adjust color balance manually
            #}
        )

        # Apply configuration and start the camera
        self.picam2.configure(camera_config)
        self.picam2.start()
        rospy.loginfo("Camera initialized with new configuration.")

    def detect_ball(self, frame):
        """Detect the red ball using color segmentation and return the position."""
        # Convert frame from RGB to BGR
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert BGR frame to HSV for color segmentation
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Define color range for detecting red (adjust as needed)
        lower_red = np.array([0, 149, 0])
        upper_red = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour, assuming it’s the ball
            c = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)

            if radius > 10:  # Ignore small objects
                # Draw a green circle around the detected ball
                cv2.circle(frame_bgr, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                # Display the coordinates below the circle
                text = f"X: {int(x)}, Y: {int(y)}"
                cv2.putText(
                    frame_bgr, text, (int(x) - 50, int(y) + int(radius) + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
                )
                return frame_bgr, (x, y)

        return frame_bgr, None

    def run(self):
        """Main loop to capture frames and detect the red ball."""
        rospy.loginfo("Cam Test Node is running... Press 'q' to quit.")
        while not rospy.is_shutdown():
            # Capture frame from the camera
            frame = self.picam2.capture_array()

            # Process the frame to detect the ball
            processed_frame, position = self.detect_ball(frame)

            # Display the frame with bounding box and coordinates
            cv2.imshow("Ball Detection", processed_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = CamTestNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
