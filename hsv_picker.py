from picamera2 import Picamera2
import cv2
import numpy as np

def nothing(x):
    pass

# Initialize Picamera2
picam2 = Picamera2()

# Configure the camera (similar to your camera node)
camera_config = picam2.create_preview_configuration(
    main={"size": (1920, 1080)},

)
picam2.configure(camera_config)
picam2.start()

# Create a window with trackbars for adjusting HSV values
cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 0, 180, nothing)  # Lower Hue
cv2.createTrackbar("L-S", "Trackbars", 100, 255, nothing)  # Lower Saturation
cv2.createTrackbar("L-V", "Trackbars", 100, 255, nothing)  # Lower Value
cv2.createTrackbar("U-H", "Trackbars", 10, 180, nothing)  # Upper Hue
cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)  # Upper Saturation
cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)  # Upper Value

while True:
    # Capture a frame from the camera
    frame = picam2.capture_array()

    # Convert from RGB to BGR for OpenCV
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Convert the BGR frame to HSV
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Get the current values from the trackbars
    l_h = cv2.getTrackbarPos("L-H", "Trackbars")
    l_s = cv2.getTrackbarPos("L-S", "Trackbars")
    l_v = cv2.getTrackbarPos("L-V", "Trackbars")
    u_h = cv2.getTrackbarPos("U-H", "Trackbars")
    u_s = cv2.getTrackbarPos("U-S", "Trackbars")
    u_v = cv2.getTrackbarPos("U-V", "Trackbars")

    # Create a mask using the current HSV range
    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Apply the mask to the original frame
    result = cv2.bitwise_and(frame_bgr, frame_bgr, mask=mask)

    # Display the original frame, mask, and result
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
