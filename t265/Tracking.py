import logging, time
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R

# Default values
DEFAULT_CONNECTION_RETRY = 5
DEFAULT_RETRY_TIME = 500
DEFAULT_TIMEOUT = 100


class Tracking:
    def __init__(self, camera_sn=None, connection_retry=DEFAULT_CONNECTION_RETRY, retry_time=DEFAULT_RETRY_TIME,
                 timeout=DEFAULT_TIMEOUT):
        """
        Setting up the tracing camera. If you want to use multiple cameras, create multiple Tracking object instances.

        Args:
            camera_sn (str): Camera serial number. None for the first camera that is connected.
            connection_retry (int): Retry times for connection.
            retry_time (int): Retry time for connection in milliseconds.
            timeout (int): Retry timeout for camera frame wait in milliseconds. If the camera does not respond in the given
                           timeout, it will return the latest camera pose in the default frame.
        """
        self.pose = None
        self.camera_on = False
        self.camera_sn = camera_sn
        self.retry_time = retry_time / 1000  # Convert to seconds from milliseconds
        self.TIMEOUT = timeout
        self.pipe = rs.pipeline()
        self._config_camera()
        self.connection_retry = connection_retry
        self.init_T = np.eye(4)

    def start_tracking(self):
        """
        Start tracking with the camera. It will retry for self.connection_retry times.
        If you like to start tracking with a specific camera, set the camera_sn in the constructor.

        Tips:
            This will be called automatically when you call update_pose() for the first time.
        """
        self.pipe.start(self.config)
        for i in range(self.connection_retry):
            try:
                frames = self.pipe.wait_for_frames(self.TIMEOUT)
                if frames.get_pose_frame() is not None:
                    self.camera_on = True
                    self.update_pose(wait=True)
                    self.init_T = self.get_matrix()
                    self.get_camera_sn()
                    break
            except RuntimeError:
                pass
            except KeyboardInterrupt:
                self.stop_tracking()
            if i != 0:
                logging.warning(f'T265 Camera retry {i}')
            else:
                logging.debug(f'T265 Camera Connected on the first try')
            time.sleep(self.retry_time)
        else:
            raise ConnectionError("Camera communication errors")

    def _config_camera(self):
        """
        Configure the camera.
        """
        self.config = rs.config()
        if self.camera_sn is None:
            self.config.enable_stream(rs.stream.pose)
        else:
            self.config.enable_device(self.camera_sn)

    def stop_tracking(self):
        """
        Stop tracking with the camera. Safe closure.

        Note:
            This will be called automatically when Python exits.
        """
        if self.camera_on:
            self.pipe.stop()
            self.camera_on = False

    def update_pose(self, wait=True):
        """
        Receive camera pose tracking information from the camera. you need to call this function to get the latest pose.
        Then you can call get_translation(), get_velocity(), get_acceleration(), get_matrix() to get the pose, velocity, acceleration, and transformation matrix.


        Args:
            wait (bool): If True, waits for the next frame. Blocking call. Timeouts after self.TIMEOUT milliseconds.

        Returns:
            None
        """
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

    def get_translation(self, frame=None, trans=True, rotation='quat', degrees=False):
        """
        Get the translation of the camera.

        Args:
            frame (str): The frame of the translation. If frame is None, return the translation of the default frame.
            trans (bool): If True, returns the translation.
            rotation (str): Rotation format. 'quat', or Euler 'xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx', 'ZYX', 'ZXY',
                            'YXZ', 'YZX', 'XZY', 'XYZ'.
            degrees (bool): If True, the Euler angles are returned in degrees. ignored if rotation is 'quat'.

        Returns:
            np.array: [x, y, z, qx, qy, qz, qw] or [x, y, z, rx, ry, rz] for 'xyz' format. Follows the input Euler
                      order in the output.
        """
        ret = None
        if frame is None:
            ret = self._get_translation(trans, rotation)
        else:
            T = self.get_matrix(frame)
            if T is not None:
                t = T[0:3, 3].flatten()
                q = R.from_matrix(T[0:3, 0:3]).as_quat()
                ret = self._to_nparray(t, q, trans, rotation, degrees)
        return ret

    def _to_single_nparray(self, list_array):
        if len(list_array) == 1:
            return list_array[0]
        return np.append(*list_array)

    def _get_translation(self, trans=True, rotation='quat', degrees=False):
        """
        Private function to get the translation of the camera always in the default frame.

        Args:
            rotation (str): Rotation format. 'matrix', 'quat', or Euler 'xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx',
                            'ZYX', 'ZXY', 'YXZ', 'YZX', 'XZY', 'XYZ'.
            degrees (bool): If True, the Euler angles are returned in degrees.

        Returns:
            np.array: [x, y, z, qx, qy, qz, qw]
        """
        if self.pose:
            t = self._vector2np(self.pose.get_pose_data().translation)
            q = self._quat2np(self.pose.get_pose_data().rotation)
            return self._to_nparray(t, q, trans, rotation, degrees)

    def _to_nparray(self, t, q, trans, rotation, degrees):
        """
        Convert translation and rotation to np.array.

        Args:
            t (np.array): Translation np.array([x, y, z]).
            q (np.array): Quaternion np.array([x, y, z, w]).
            trans (bool): If True, returns the translation.
            rotation (str): Format of rotation. 'quat', or Euler 'xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx', 'ZYX', 'ZXY',
                            'YXZ', 'YZX', 'XZY', 'XYZ'.
            degrees (bool): If True, the Euler angles are returned in degrees.

        Returns:
            np.array: Flat np.array([x, y, z, qx, qy, qz, qw]) or np.array([x, y, z, rx, ry, rz]) for 'xyz' format.
                      Follows the input Euler order in the output.
        """
        ret_list = []
        if trans:
            ret_list.append(t)
        if rotation is not None:
            ret_list.append(self._quat2other(q, format=rotation, degrees=degrees))
        return self._to_single_nparray(ret_list)

    def get_velocity(self):
        """
        Get the velocity of the camera.

        Returns:
            np.array: [vx, vy, vz, wx, wy, wz]
        """
        if self.pose:
            vel = self._vector2np(self.pose.get_pose_data().velocity)
            ang_vel = self._vector2np(self.pose.get_pose_data().angular_velocity)
            return np.append(vel, ang_vel)

    def get_acceleration(self):
        """
        Get the acceleration of the camera.

        Returns:
            np.array: [ax, ay, az, awx, awy, awz]
        """
        if self.pose:
            acc = self._vector2np(self.pose.get_pose_data().acceleration)
            ang_acc = self._vector2np(self.pose.get_pose_data().angular_acceleration)
            return np.append(acc, ang_acc)

    def get_matrix(self, frame=None):
        """
        Get the transformation matrix of the camera in a specific frame.

        Args:
            frame (str): Frame of the transformation matrix. If frame is None, return the transformation matrix of
                         the default frame.

        Returns:
            np.array: [r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz, 0, 0, 0, 1].reshape(4, 4)
        """
        if self.pose:
            trans = self._get_translation()
            rot_mat = R.from_quat(trans[3:]).as_matrix()
            T = np.eye(4)
            T[0:3, 0:3] = rot_mat
            T[0:3, 3] = trans[0:3]

            if frame == 'ros':
                rot = R.from_euler('xyz', [0, 90, 90], degrees=True).as_matrix()
                c_T_ros = np.eye(4)
                c_T_ros[0:3, 0:3] = rot

                E_T_ck = T
                c0_T_E = self.init_T.transpose()
                c0_T_ck = c0_T_E @ E_T_ck
                ros_T_c = c_T_ros.transpose()
                ros0_T_ck = ros_T_c @ c0_T_ck
                ros0_T_rosk = ros0_T_ck @ c_T_ros
                T = ros0_T_rosk
            return T

    def _quat2other(self, quat, order: str = 'xyzw', format='matrix', degrees=False):
        """
        Convert quaternion to other format.

        Args:
            quat (np.array): Quaternion np.array([x, y, z, w]) or np.array([w, x, y, z]).
            order (str): Quaternion order, xyzw or wxyz.
            format (str): Format to convert to. 'matrix', 'quat', or 'xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx', 'ZYX',
                          'ZXY', 'YXZ', 'YZX', 'XZY', 'XYZ'.
            degrees (bool): If True, the Euler angles are returned in degrees.

        Returns:
            np.array: Whatever format you want.
        """
        if order != 'xyzw':
            quat = np.append(quat[1:], quat[0])
        if format == 'matrix':
            return R.from_quat(quat).as_matrix()
        elif format == 'quat':
            return quat
        elif format in ['xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx', 'ZYX', 'ZXY', 'YXZ', 'YZX', 'XZY', 'XYZ']:
            return R.from_quat(quat).as_euler(format, degrees=degrees)

    def _vector2np(self, vector):
        """
        Convert rs.vector to np.array.

        Args:
            vector (pyrealsense2.vector): The vector to convert.

        Returns:
            np.array: [x, y, z]
        """
        return np.array([vector.x, vector.y, vector.z])

    def _quat2np(self, quat, order: str = 'xyzw'):
        """
        Convert rs.quaternion to np.array.

        Args:
            quat (pyrealsense2.quaternion): The quaternion to convert.
            order (str): xyzw or wxyz string.

        Returns:
            np.array: [x, y, z, w]
        """
        if order == 'wxyz':
            return np.array([quat.w, quat.x, quat.y, quat.z])
        else:
            return np.array([quat.x, quat.y, quat.z, quat.w])

    # Status functions
    def is_camera_on(self):
        """
        Check if the camera is on.

        Returns:
            bool: True if the camera is on.
        """
        return self.camera_on

    def get_camera_sn(self):
        """
        Get the camera serial number.

        Returns:
            str: Camera serial number.
        """
        self.camera_sn = self.pipe.get_active_profile().get_device().get_info(rs.camera_info.serial_number)
        return self.camera_sn

    def __del__(self):
        """
        Destructor: Turns off the camera upon Python exit.
        """
        self.stop_tracking()
