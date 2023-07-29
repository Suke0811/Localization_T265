import pyrealsense2 as rs
import numpy as np
import logging, time
from  scipy.spatial.transform import Rotation as R



DEFAULT_CONNECTION_RETRY = 5
DEFAULT_RETRY_TIME = 0.5
DEFAULT_TIMEOUT = 100





class Tracking:
    def __init__(self, camera_sn=None, connection_retry=DEFAULT_CONNECTION_RETRY):
        self.pose = None
        self.camera_on = False
        self.camera_sn = camera_sn

        self.TIMEOUT = DEFAULT_TIMEOUT
        self.pipe = rs.pipeline()
        self.config_camera()
        self.connection_retry = connection_retry
        self.init_T = np.eye(4)

    def config_camera(self):
        self.config = rs.config()
        if self.camera_sn is None:
            self.config.enable_stream(rs.stream.pose)
        else:
            self.config.enable_device(self.camera_sn)

    def start_tracking(self):
        self.pipe.start(self.config)
        for i in range(self.connection_retry):
            try:
                frames = self.pipe.wait_for_frames(self.TIMEOUT)
                if frames.get_pose_frame() is not None:
                    self.camera_on = True
                    self.update_pose(wait=True)
                    self.init_T = self.get_matrix()

                    break
            except RuntimeError:
                pass
            except KeyboardInterrupt:
                self.stop_tracking()


            logging.info(f'T265 Camera retry {i}')
            time.sleep(DEFAULT_RETRY_TIME)
        else:
            raise ConnectionError("Camera communication errors")


    def stop_tracking(self):
        if self.camera_on:
            self.pipe.stop()
            self.camera_on = False

    def update_pose(self, wait=True):
        if not self.camera_on:
            self.start_tracking()
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

    def get_translation(self, frame=None):
        if frame is None:
            return self._get_translation()
        else:
            T = self.get_matrix(frame)
            if T is not None:
                trans = T[0:3,3].flatten()
                quat = R.from_matrix(T[0:3, 0:3]).as_quat()
                return np.append(trans, quat)

    def _get_translation(self):
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

    def get_matrix(self, frame=None):
        if self.pose:
            trans = self._get_translation()
            rot_mat = R.from_quat(trans[3:]).as_matrix()
            T = np.eye(4)
            T[0:3,0:3] = rot_mat
            T[0:3,3] = trans[0:3]

            if frame == 'ros':
                rot = R.from_euler('xyz', [0, 90, 90], degrees=True).as_matrix()
                c_T_ros = np.eye(4)
                c_T_ros[0:3, 0:3] = rot

                # E_T_ck = np.eye(4, dtype=np.float32)
                # E_T_ck[0:3, 0:3] = np.array(E_R_ck)
                # E_T_ck[0:3, 3] = np.array(self.current_pos)
                # # c0_T_ck = c0_T_E * E_T_ck
                # c0_T_ck = np.dot(self.c0_T_E, E_T_ck)
                # # b0_T_ck = b0_T_c0 * c0_T_ck
                # b0_T_ck = np.dot(self.b_T_c, c0_T_ck)
                # b0_T_bk = np.dot(b0_T_ck, self.c_T_b)
                E_T_ck = T
                c0_T_E = self.init_T.transpose()
                c0_T_ck = c0_T_E @ E_T_ck
                ros_T_c = c_T_ros.transpose()
                ros0_T_ck = ros_T_c @ c0_T_ck
                ros0_T_rosk = ros0_T_ck @ c_T_ros
                T = ros0_T_rosk
            return T


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
        pos = track.get_translation('ros')
        print(f'pos: {pos[0:3]}')
        rot = R.from_quat(pos[3:]).as_euler('zyx', degrees=True)
        print(f'rot: {rot}')
        #print(track.get_matrix('ros'))
        time.sleep(1)
        #print("Velocity: {}".format(track.get_velocity()))
        #print("Acceleration: {}\n".format(track.get_acceleration()))
