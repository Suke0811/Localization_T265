import pyrealsense2 as rs
import numpy as np


class Tracking:
    def __init__(self, camera_sn=None):
        self.pose = None
        self.camera_on = False
        self.camera_sn = camera_sn

        self.TIMEOUT = 100
        self.pipe = rs.pipeline()
        self.config_camera()

    def config_camera(self):
        self.config = rs.config()
        if self.camera_sn is None:
            self.config.enable_stream(rs.stream.pose)
        else:
            self.config.enable_device(self.camera_sn)

    def start_tracking(self):
        self.pipe.start(self.config)

    def stop_tracking(self):
        if self.camera_on:
            self.pipe.stop()
            self.camera_on = False

    def update_pose(self, wait=True):
        if not self.camera_on:
            self.start_tracking()
            self.camera_on = True
        try:
            if wait:
                frames = self.pipe.wait_for_frames(self.TIMEOUT)
            else:
                frames = self.pipe.poll_for_frames()
            self.pose = frames.get_pose_frame()
        except RuntimeError:
            pass
        except KeyboardInterrupt:
            self.stop_tracking()


    def get_translation(self):
        if self.pose:
            trans = self._vector2np(self.pose.get_pose_data().translation)
            quat = self._quat2np(self.pose.get_pose_data().rotation)
            return np.append(trans, quat)

    def get_velocity(self):
        if self.pose:
            vel = self._vector2np(self.pose.get_pose_data().velocity)
            ang_vel = self._vector2np(self.pose.get_pose_data().angular_velocity)
            return np.append(vel, ang_vel)

    def get_acceleration(self):
        if self.pose:
            acc = self._vector2np(self.pose.get_pose_data().acceleration)
            ang_acc = self._vector2np(self.pose.get_pose_data().angular_acceleration)
            return np.append(acc, ang_acc)

    def _vector2np(self, vector):
        return np.array([vector.x, vector.y, vector.z])

    def _quat2np(self, quat, order='xyzw'):
        if order == 'wxyz':
            return np.array([quat.w, quat.x, quat.y, quat.z])
        else:
            return np.array([quat.x, quat.y, quat.z, quat.w])

    def __del__(self):
        self.stop_tracking()

if __name__ == "__main__":
    track = Tracking()
    while True:
        track.update_pose(wait=True)
        print("Position: {}".format(track.get_translation()))
        #print("Velocity: {}".format(track.get_velocity()))
        #print("Acceleration: {}\n".format(track.get_acceleration()))
