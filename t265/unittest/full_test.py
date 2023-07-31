from t265 import Tracking
import time

if __name__ == "__main__":
    """
    Example of using the Tracking class
    prints the pose, velocity and acceleration every second in ros frame
    """
    track = Tracking()
    while True:
        track.update_pose(wait=True)
        pos = track.get_translation('ros')
        print(f"Camera_sn: {track.get_camera_sn()}")
        print(f'pos: {pos[0:3]}')
        print(f"T: {track.get_matrix('ros')}")
        print(f"euler {track.get_translation('ros', trans=False, rotation='zyx', degrees=True)}")
        print("Velocity: {}".format(track.get_velocity('ros')))
        print("Acceleration: {}\n".format(track.get_acceleration()))
        time.sleep(1)

