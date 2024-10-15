from picamera2 import Picamera2
import cv2

# Initialize the camera
picam2 = Picamera2()

# Configure the camera with desired settings
camera_config = picam2.create_preview_configuration(
    main={"size": (1920, 1080)},  # Use wide-angle resolution
    #controls={
     #   "AwbEnable": True,  # Disable auto white balance
     #   "ExposureTime": 20000,  # Set exposure time
     #   "AnalogueGain": 1.5,  # Set gain for brightness
    #}
)

# Apply the configuration and start the camera
picam2.configure(camera_config)
picam2.start()

while True:
    # Capture frame from the camera
    frame = picam2.capture_array()

    # Optionally flip the frame if orientation is incorrect
    
    # frame = cv2.flip(frame, 0)  # Flip vertically
    # frame = cv2.flip(frame, 1)  # Flip horizontally
    
    # Convert RGB to BGR (since OpenCV uses BGR)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    # Display the frame
    cv2.imshow("Camera Test", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
