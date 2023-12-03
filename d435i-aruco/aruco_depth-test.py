import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

# brooo I forgot to use rs-align to align the two cameras since there is physics offset :((
align_to = rs.stream.color
align = rs.align(align_to)

# using 6x6_250 Dict
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters_create()

try:
    while True:
        frames = pipeline.wait_for_frames()
        
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Aruco marker detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        # Visualize the depth image
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        if ids is not None:
            # Color image detection
            aruco.drawDetectedMarkers(color_image, corners, ids)

            # Depth image detection
            aruco.drawDetectedMarkers(depth_colormap, corners, ids)

            for i in range(len(ids)):
                # Get the corner coordinates to draw the line
                corner = corners[i][0]
                x = int(corner[:, 0].mean())
                y = int(corner[:, 1].mean())

                # Get depth value from depth image
                depth = aligned_depth_frame.get_distance(x, y)

                # Display the distance and ID in color image
                cv2.putText(color_image, f"ID {ids[i][0]}: {depth:.2f}m", (x, y), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                # depth colormap display
                cv2.putText(depth_colormap, f"ID {ids[i][0]}: {depth:.2f}m", (x, y), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)

        cv2.imshow('RealSense Color', color_image)
        cv2.imshow('RealSense Depth', depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
