import numpy as np
from scipy.spatial.transform import Rotation as R


class FrameHandler:
    def __init__(self, frame_name):
        """
        Create a new FrameHandler object to manage the transformation between frames.

        Args:
            frame_name (str): The unique identifier of the frame.
        """
        self.frame_name = frame_name
        self.T_init = np.eye(4)
        self.T_trans = np.eye(4)
        pass

    def set_frame_transformation(self, trans=None, rot=None, rot_format='quat', degrees=False, T=None):
        """
        Define the transformation matrix for this frame.

        Args:
            trans (np.array, optional): The translation vector [x, y, z]. Defaults to zero vector if not provided.
            rot (np.array, optional): The rotation, represented as either a quaternion or Euler angles. Defaults to identity matrix if not provided.
            rot_format (str, optional): The format of the rotation (e.g., 'quat', 'matrix', 'xyz'). Defaults to 'quat'.
            degrees (bool, optional): Indicates whether the rotation angles are in degrees. Defaults to False (i.e., angles are in radians).
            T (np.array, optional): A precomputed 4x4 transformation matrix. If provided, 'trans' and 'rot' will be ignored. Defaults to None.
        """
        if T is not None:
            self.T_trans = T
        else:
            if trans is None:
                trans = np.zeros(3)
            if rot is None:
                rot = np.eye(3)
            rotation = self._convert_rotation(rot, rot_format=rot_format, from_degrees=degrees, format='matrix')

            self.T_trans[:3, :3] = rotation
            self.T_trans[0:3, 3] = trans

    def set_init_frame(self,  trans=None, rot=None, rot_format='quat', degrees=False, T=None):
        """
        Define the initial frame for the transformation.

        Args:
            trans (np.array, optional): The translation vector [x, y, z]. Defaults to zero vector if not provided.
            rot (np.array, optional): The rotation, represented as either a quaternion or Euler angles. Defaults to identity matrix if not provided.
            rot_format (str, optional): The format of the rotation (e.g., 'quat', 'matrix', 'xyz'). Defaults to 'quat'.
            degrees (bool, optional): Indicates whether the rotation angles are in degrees. Defaults to False (i.e., angles are in radians).
            T (np.array, optional): A precomputed 4x4 transformation matrix. If provided, 'trans' and 'rot' will be ignored. Defaults to None.
        """
        # If transformation matrix is provided, use it directly
        if T is not None:
            self.T_init = T
        else:
            # If no translation vector or rotation is provided, use default values
            if trans is None:
                trans = np.zeros(3)
            if rot is None:
                rot = np.eye(3)

            # Convert the rotation to a rotation matrix
            rotation = self._convert_rotation(rot, rot_format=rot_format, from_degrees=degrees, format='matrix')

            # Combine the translation and rotation into the transformation matrix
            self.T_init[:3, :3] = rotation
            self.T_init[0:3, 3] = trans


    def _convert_rotation(self, rot, rot_format='quat', from_degrees=False, format='matrix', to_degrees=False):
        """
        Convert a rotation from one representation to another (e.g., quaternion to matrix, or Euler angles to quaternion).

        Args:
            rot (np.array): The rotation in its original representation.
            rot_format (str): The format of the original rotation (e.g., 'quat', 'matrix', 'xyz').
            from_degrees (bool): Indicates whether the rotation angles in the original representation are in degrees.
            format (str): The desired format for the rotation.
            to_degrees (bool): Indicates whether the rotation angles in the converted representation should be in degrees.

        Returns:
            np.array: The rotation in the desired format.
        """
        # If the original rotation is in the wxyz format, reorder it to the xyzw format
        if rot_format == 'wxyz':
            rot = np.append(rot[1:], rot[0])
            format = 'xyzw'

        # Convert the rotation to a scipy Rotation object, based on the original format
        if rot_format in ['quat', 'xyzw']:
            rotation = R.from_quat(rot)
        elif rot_format == 'matrix':
            rotation = R.from_matrix(rot)
        elif rot_format in ['xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx', 'ZYX', 'ZXY', 'YXZ', 'YZX', 'XZY', 'XYZ']:
            rotation = R.from_euler(rot_format, rot, degrees=from_degrees)
        else:
            raise ValueError(f'Invalid rotation format: {rot_format}')

        # Convert the Rotation object to the desired format
        if format in ['quat', 'xyzw', 'wxyz']:
            ret = rotation.as_quat()
            if format == 'wxyz':
                ret = np.append(ret[3], ret[0:3])
        elif format == 'matrix':
            ret = rotation.as_matrix()
        elif format in ['xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx', 'ZYX', 'ZXY', 'YXZ', 'YZX', 'XZY', 'XYZ']:
            ret = rotation.as_euler(format, degrees=to_degrees)
        else:
            raise ValueError(f'Invalid rotation format: {format}')

        return ret

    def convert(self, current_camera_frame, T_mat=True, angular=None, linear=None, local_frame=False):
        """
        Apply the frame transformation to the given frame, resulting in a transformed frame and, optionally, angular and linear velocities.

        Args:
            current_camera_frame (np.array): 4x4 transformation matrix of the current camera frame.
            T_mat (bool, optional): If True, the resulting transformed frame matrix is included in the return tuple. Defaults to True.
            angular (np, optional): If not None, the transformed angular velocity is included in the return tuple. Defaults to True.
            linear (np, optional): If not None, the transformed linear velocity is included in the return tuple. Defaults to True.
            local_frame (bool, optional): If True, the velocity is w.r.t. the current frame

        Returns:
            tuple: A tuple containing the transformed frame, angular velocity, and linear velocity as requested.
                   The order of the returned items is as follows: transformed frame, angular velocity, linear velocity.
        """
        T = self.transform(current_camera_frame, return_all_frames=False)
        if angular is not None and linear is not None:
            if local_frame:
                vel = self.transform_velocity_to_current_pose(current_camera_frame, angular, linear)
            else:
                vel = self.transform_velocity(current_camera_frame, angular, linear)

        if T_mat and angular is not None and linear is not None:
            return T, vel
        elif angular is not None and linear is not None:
            return vel
        else:
            return T

    def transform(self, current_camera_frame, return_all_frames=False):
        """
        Apply the frame transformation to the given frame, resulting in a transformed frame.

        Args:
            current_camera_frame (np.array): 4x4 transformation matrix of the current camera frame.
            return_all_frames (bool, optional): If True, returns a tuple of all intermediate transformation matrices. Defaults to False.

        Returns:
            np.array or tuple: If return_all_frames is False, this function returns a 4x4 transformation matrix to the target frame.
                               If True, it returns a tuple containing all intermediate transformation matrices used in the process.
        """
        # Compute the transformation matrix from the current frame to the target frame
        c_T_ros = self.T_trans

        # Get the transformation matrix of the current camera frame
        E_T_ck = current_camera_frame

        # Get the inverse transformation matrix of the initial frame
        c0_T_E = self.T_init.transpose()

        # Compute the transformation matrix from the initial frame to the current camera frame
        c0_T_ck = c0_T_E @ E_T_ck

        # Compute the transformation matrix from the target frame to the current camera frame
        ros_T_c = c_T_ros.transpose()
        ros0_T_ck = ros_T_c @ c0_T_ck

        # Compute the transformation matrix from the target frame to the current frame
        ros0_T_rosk = ros0_T_ck @ c_T_ros
        T = ros0_T_rosk

        # Return the final transformation matrix or all intermediate matrices, depending on the return_all_frames flag
        if return_all_frames:
            return T, ros0_T_rosk, ros0_T_ck, c0_T_ck, c0_T_E, E_T_ck, ros_T_c, c_T_ros
        else:
            return T

    def transform_velocity(self, current_camera_frame, angular, linear):
        """
        Transforms velocity components from the current camera frame to the target frame.

        Args:
            current_camera_frame (np.array): 4x4 transformation matrix of the current camera frame.
            angular (np.array): Angular velocity vector [wx, wy, wz] in the initial frame.
            linear (np.array): Linear velocity vector [vx, vy, vz] in the initial frame.

        Returns:
            np.array: The transformed velocity vector [wx', wy', wz', vx', vy', vz'] in the target frame.
        """
        # Get all intermediate transformation matrices by calling the transform() method with return_all_frames=True
        T, ros0_T_rosk, ros0_T_ck, c0_T_ck, c0_T_E, E_T_ck, ros_T_c, c_T_ros = self.transform(current_camera_frame, return_all_frames=True)

        # Get the translation vector of the target frame relative to the current camera frame
        c_p_ros = self.T_trans[0:3, 3].flatten()

        # Get the rotation matrix from the intermediate transformation matrix (ros0_T_rosk)
        ros0_R_rosk = ros0_T_rosk[0:3, 0:3]

        # Get the rotation matrix from the inverse transformation matrix (E_T_ck) and transpose it to get the rotation matrix from the current camera frame to the initial frame (ck_R_E)
        ck_R_E = E_T_ck[0:3, 0:3].transpose()

        # Compute the angular velocity in the current camera frame by rotating the angular velocity from the initial frame to the current frame (ck_w_ck)
        ck_w_ck = ck_R_E @ angular

        # Compute the linear velocity in the current camera frame by rotating the linear velocity from the initial frame to the current frame (ck_v_ck)
        ck_v_ck = ck_R_E @ linear

        # Transform the angular velocity from the current camera frame to the target frame (ros0_w_ros)
        ros0_w_ros = (ros0_R_rosk @ ck_w_ck).reshape(3, )

        # Transform the linear velocity from the current camera frame to the target frame (ros0_v_ros)
        ros0_v_ros = (ros0_R_rosk @ ck_v_ck + np.cross(ck_w_ck, c_p_ros)).reshape(3, )

        # Return the transformed velocity vector, combining the transformed angular and linear velocities
        return np.append(ros0_v_ros, ros0_w_ros)

    def transform_velocity_to_current_pose(self, current_camera_frame, angular, linear):
        # Compute the transformation matrix from initial to current pose frame
        T = self.transform(current_camera_frame, return_all_frames=False)

        # Extract the rotation matrix and translation vector
        current_R = T[:3, :3]
        current_t = T[:3, 3]

        # Transform the angular velocity
        transformed_angular = current_R @ angular
        # Transform the linear velocity
        transformed_linear = current_R @ linear + np.cross(transformed_angular, current_t)

        return np.append(transformed_angular, transformed_linear)
