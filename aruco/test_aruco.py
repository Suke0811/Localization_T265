# the size of my aruco tags is 1.97inch, or 0.05m
from ArucoDetector import ArucoDetector
import cv2


def aruco_test():
    # Initialize the ArucoDetector
    aruco_detector = ArucoDetector(cv2.aruco.DICT_5X5_250)

    # Start the camera stream   
    aruco_detector.start_stream()

    # Set the exposure of the camera
    aruco_detector.auto_calibration()  # Make sure this value is within the supported range

    while True:
        # Process frames from the camera
        aligned_depth_frame, color_image = aruco_detector.process_frames()
        if color_image is not None and aligned_depth_frame is not None:
            # Update marker positions in the frame
            color_image = aruco_detector.update_marker_positions(color_image, aligned_depth_frame)
            # Display the image
            cv2.imshow('RealSense Color', color_image)

        # buttons to test functions
        key = cv2.waitKey(1) & 0xFF

        # Check for 'c' key to run auto_calibration
        if key == ord('c'):
            aruco_detector.auto_calibration()

        # Check for 'r' key to reset exposure
        elif key == ord('r'):
            aruco_detector.reset_exposure()

        # Break the loop when 'q' is pressed
        elif key == ord('q'):
            break

    # Clean up resources properly
    aruco_detector.uninitialize()
    # cv2.destroyAllWindows() # redundant


if __name__ == '__main__':
    aruco_test()
