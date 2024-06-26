from t265 import Tracking
import time
import numpy as np

if __name__ == "__main__":
    """
    Example of using the Tracking class
    prints the pose, velocity and acceleration every second in ros frame
    """
    track = Tracking()

    # define a custom frane
    c_T_b = np.linalg.inv(np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0.1515],
        [0, -1, 0, 0.038],
        [0, 0, 0, 1]]))
    # Add the custom frame
    track.add_custom_frames('body', T=c_T_b)
    while True:
        track.update_pose(wait=True)
        pos = track.get_translation()      # get all 6d pose
        print(f"Camera_sn: {track.get_camera_sn()}")
        print(f'pos: {pos[0:3]}')
        print(f"T: {track.get_matrix('ros')}")  # get matrix in ros frame
        # Get the translation in the body frame
        print(f"euler {track.get_translation('body', trans=False, rotation='XYZ', degrees=True)}")
        print(f"default {track.get_translation(trans=False, rotation='XYZ', degrees=True)}")

        # print("Velocity: {}".format(track.get_velocity('ros')))
        # print("Acceleration: {}\n".format(track.get_acceleration()))
        time.sleep(1)
