# the size of my aruco tags is 1.97inch, or 0.05m
from ArucoDetector import ArucoDetector
import cv2

# Initialize the ArucoDetector
aruco_detector = ArucoDetector(cv2.aruco.DICT_5X5_250)

# Start the camera stream
aruco_detector.start_stream()

# Set the exposure of the camera
# aruco_detector.auto_calibration()  # Make sure this value is within the supported range

while True:
    # Process frames from the camera
    aligned_depth_frame, color_image = aruco_detector.process_frames()
    if color_image is not None and aligned_depth_frame is not None:
        # Update marker positions in the frame
        color_image = aruco_detector.update_marker_positions(color_image, aligned_depth_frame)
        # Display the image
        cv2.imshow('RealSense Color', color_image)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

