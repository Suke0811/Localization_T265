import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import time

class ArucoDetector:
    def __init__(self, aruco_dict_type=aruco.DICT_5X5_250):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        
        # Align the two cameras since there is physical offset
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        # Use 6x6_250 Dict aruco library
        self.aruco_dict = aruco.Dictionary_get(aruco_dict_type)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Dictionary to hold marker positions
        self.marker_positions = {}
        
        # Variable to hold camera intrinsics
        self.intrinsics = None
    
    def __del__(self):
        # Destructor to safely close windows and stop the stream
        print("Closing ArucoDetector and releasing resources")
        try:
            if self.pipeline:
                self.pipeline.stop()
        except Exception as e:
            print(f"Error stopping pipeline: {e}")
        finally:
            cv2.destroyAllWindows()
            # set back to auto exposure
            color_sensor = self.profile.get_device().first_color_sensor()
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
    
    # same as desructor, can be manually called
    def uninitialize(self):
        # Destructor to safely close windows and stop the stream
        print("Closing ArucoDetector and releasing resources")
        try:
            if self.pipeline:
                self.pipeline.stop()
        except Exception as e:
            print(f"Error stopping pipeline: {e}")
        finally:
            cv2.destroyAllWindows()
            # set back to auto exposure
            color_sensor = self.profile.get_device().first_color_sensor()
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)

            
    def start_stream(self):
        # Start the pipeline
        self.profile = self.pipeline.start(self.config)
        # Get the camera intrinsics from the color stream
        self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def stop_stream(self):
        # Stop the pipeline
        self.pipeline.stop()
    
    """
        Automatically calibrate the camera's exposure to detect ArUco markers.
        Args:
            min_exposure (int): Minimum exposure value to start from.
            max_exposure (int): Maximum exposure value to go up to.
            step (int): Step size to increment exposure.
            frames_to_check (int): Number of frames to check at each exposure level.
        """
    def auto_calibration(self, min_exposure=50, max_exposure=1600, step=50, frames_to_check=5):
        color_sensor = self.profile.get_device().first_color_sensor()
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)

        best_exposure = min_exposure
        max_detected_markers = 0

        for exposure_value in range(min_exposure, max_exposure + 1, step):
            color_sensor.set_option(rs.option.exposure, exposure_value)
            print(f"Setting exposure to {exposure_value} and checking markers...")

            markers_detected = 0

            for frame_count in range(frames_to_check):
                aligned_depth_frame, color_image = self.process_frames()
                if color_image is not None and aligned_depth_frame is not None:
                    # Display the live RGB output
                    cv2.imshow('Calibration - RGB Output', color_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit if 'q' is pressed
                        cv2.destroyAllWindows()
                        return

                    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

                    if ids is not None:
                        markers_detected += len(ids)

                time.sleep(0.025)  # Give some time for the camera to adjust if needed

            print(f"Exposure: {exposure_value}, Markers Detected: {markers_detected}")
            if markers_detected > max_detected_markers:
                max_detected_markers = markers_detected
                best_exposure = exposure_value
                print(f"New best exposure found: {best_exposure} with {markers_detected} markers detected.")

        # After finding the best exposure, set the camera to use it
        color_sensor.set_option(rs.option.exposure, best_exposure)
        print(f"Auto-calibration complete. Best exposure set to {best_exposure}.")
        cv2.destroyAllWindows()
    
    def reset_exposure(self):
        # Get the color sensor from the connected device
        color_sensor = self.profile.get_device().first_color_sensor()
        
        # Set the exposure mode back to automatic
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        print("Exposure reset to automatic.")

        
    # non-blocking processing of frames (poll_for_frames)
    def process_frames(self):
        # Attempt to retrieve the next set of frames
        frames = self.pipeline.poll_for_frames()

        # Check if frames are available
        if frames:
            aligned_frames = self.align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                return None, None

            # Convert color image to numpy array
            color_image = np.asanyarray(color_frame.get_data())

            return aligned_depth_frame, color_image
        else:
            # No new frames available
            return None, None

    
# ### helper functions for update_marker_positions ### #
    #  calculates x, y, z position
    def calculate_m (self, x_pixel, y_pixel, depth_frame):
        # Get depth value
        depth = depth_frame.get_distance(x_pixel, y_pixel)

        # Deproject pixels to 3D space
        point_3D = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x_pixel, y_pixel], depth)      
        
        # Adjust y-coordinate so that up is positive y
        # spec sheet gives that camera is -4.2mm from front glass
        x, y, z = point_3D
        y = -y  
        z = z - 0.0042
                
        # Convert from meters to centimeters
        # consistently 6mm too much?
        # x_cm, y_cm, z_cm = x * 100, y * 100, z * 100 
        
        return x, y, z 
    
    # Display the 3D coordinates next to the marker
    # display x, y, z in different lines
    def display(self, color_image, x_pixel, y_pixel, marker_id):
        # Get the position data from the dictionary
        x, y, z = self.marker_positions[marker_id]
        
        coord_text = (f"ID {marker_id}:\n"
                      f"X={x:.4f}m\n"
                      f"Y={y:.4f}m\n"
                      f"Z={z:.4f}m")
        for j, line in enumerate(coord_text.split('\n')):
            y_offset = y_pixel + 20 + (15 * j)
            cv2.putText(color_image, line, (x_pixel + 20, y_offset), 
                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)    
            
            
# ######################################################## # 
    
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

                # calulcate position in 3D space in cm
                x, y, z = self.calculate_m(x_pixel, y_pixel, depth_frame)
                
                # Update the dictionary with the latest position data
                marker_id = ids[i][0]
                self.marker_positions[marker_id] = [x, y, z]

                self.display(color_image, x_pixel, y_pixel, marker_id)
                
        return color_image

    def get_marker_position(self, marker_id):
        # Retrieve the latest recorded position of a marker by its ID
        # return is x, y, z as a list of floats
        return self.marker_positions.get(marker_id)
