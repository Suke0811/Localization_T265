import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
profile = pipeline.start(config)

# forgot i had to align the two camera since there is physical offset :(((
align_to = rs.stream.color
align = rs.align(align_to)

# Use 6x6_250 Dict aruco library
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters_create()

# Get the camera intrinsics from the color stream
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# Dictionary to hold marker positions
marker_positions = {}
# Function to retrieve the latest recorded position of a marker by its ID
def get_marker_position(marker_id):
    return marker_positions.get(marker_id, "Marker not found")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Detect ArUco markers
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            # Draw detected markers
            aruco.drawDetectedMarkers(color_image, corners, ids)

            for i in range(len(ids)):
                # Average the corners for better center estimation
                center = np.mean(corners[i][0], axis=0)
                x_pixel, y_pixel = int(center[0]), int(center[1])

                # Get depth value
                depth = aligned_depth_frame.get_distance(x_pixel, y_pixel)

                # Deproject pixels to 3D space
                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x_pixel, y_pixel], depth)
                
                # Adjust y-coordinate to have positive values above the center
                # idk why realsense makes below the center positive for y lol
                x, y, z = point_3d
                y = -y 

                # Display the 3D coordinates
                cv2.putText(color_image, f"ID {ids[i][0]} XYZ: {x:.2f}, {y:.2f}, {z:.2f}", 
                            (x_pixel - 100, y_pixel - 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

        # Display rbg frames
        cv2.imshow('RealSense Color', color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
