import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco

class ArucoDetector:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        
        # Align the two cameras since there is physical offset
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        # Use 6x6_250 Dict aruco library
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Dictionary to hold marker positions
        self.marker_positions = {}
        
        # Variable to hold camera intrinsics
        self.intrinsics = None

    def start_stream(self):
        # Start the pipeline
        self.profile = self.pipeline.start(self.config)
        # Get the camera intrinsics from the color stream
        self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def stop_stream(self):
        # Stop the pipeline
        self.pipeline.stop()
        
    def process_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            return None, None, None

        # Convert color image to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        return aligned_depth_frame, color_image
    
    def update_marker_positions(self, color_image, depth_frame):
        # Detect ArUco markers
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Draw detected markers
            aruco.drawDetectedMarkers(color_image, corners, ids)

            for i in range(len(ids)):
                # Average the corners for better center estimation
                center = np.mean(corners[i][0], axis=0)
                x_pixel, y_pixel = int(center[0]), int(center[1])

                # Get depth value
                depth = depth_frame.get_distance(x_pixel, y_pixel)

                # Deproject pixels to 3D space
                point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x_pixel, y_pixel], depth)
                
                # Adjust y-coordinate so that up is positive y
                x, y, z = point_3d
                y = -y  
                
                # Convert from meters to centimeters
                x_cm, y_cm, z_cm = x * 100, y * 100, z * 100
                
                # Update the dictionary with the latest position data
                self.marker_positions[ids[i][0]] = [x_cm, y_cm, z_cm]

                # Display the 3D coordinates next to the marker
                coord_text = f"ID {ids[i][0]}: X={x_cm:.2f}, Y={y_cm:.2f}, Z={z_cm:.2f}"
                cv2.putText(color_image, coord_text, (x_pixel + 20, y_pixel + 20), 
                            cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
        return color_image

    def get_marker_position(self, marker_id):
        # Retrieve the latest recorded position of a marker by its ID
        # return is x, y, z as a list of floats
        return self.marker_positions.get(marker_id)
