# the size of my aruco tags is 1.97inch, or 0.05m
from ArucoDetector import ArucoDetector
import cv2

aruco = ArucoDetector(cv2.aruco.DICT_6X6_250)
aruco.start_stream()


while True:
    aligned_depth_frame, color_image = aruco.process_frames()
    if color_image is not None and aligned_depth_frame is not None:
        color_image = aruco.update_marker_positions(color_image, aligned_depth_frame)
        cv2.imshow('RealSense Color', color_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    
# To get the position of a specific marker by ID, for example, ID 2:
position = aruco.get_marker_position(2)
print(position)
