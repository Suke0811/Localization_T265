# T265 Tracking Camera Python Library
This library is to help to simplify your interaction with T265. The camera will be automatically initialized and closed.

This is very minimalistic. The point is to provide the simplest possible way of using T265 cameras. 

## Installation
this library is pip installable
```bash
pip install t265
```

## Ver 0.1.xx
This library handles the frame transformation difference between the pyrealsense2 and ROS. 
You can specify a camera by its serial number. If you want to use more than one T265, you can create multiple instances of Tracking().



## Important Note
!!! important

    Starting [Ver. 1.54.1](https://github.com/IntelRealSense/librealsense/releases/tag/v2.54.1) Intel has dropped supports for T265 entirely. 
    Hence, this library requires pyrealsense2 earlier than Ver. 2.53.1 
    Starting Ver 0.1.3 this is part of requirements.txt.



## How to use
Here is a sample code. For more details, please check the [Tracking APIs](./tracking.md).
```python
import time
from t265 import Tracking
my_tracking = Tracking(camera_sn='133122110783') # Provide your camera serial number. It should be on the camera bottom. None will use the first camera found.

my_tracking.start_tracking()    # [Optional] start tracking. It will be called automatically when you call update_pose() first time.
                                # this will retry connection (Default retry 5 times with 1sec interval)

N = 10     # will show pose, vel, acc for 10 times
for _ in range(N):
    my_tracking.update_pose(wait=True) # wait=True will block the code till you get data from camera.     
    print(f'Is camera on: {my_tracking.is_camera_on()}')
    print(f"Camera_sn: {my_tracking.get_camera_sn()}")
    
    # Getting pose
    print(f'Current pose: {my_tracking.get_translation()}')    # getting pose (x,y,z, quat_xyzw) 1 by 7 np.array
    print(f"Current pose: {my_tracking.get_translation(frame='ros')}")    # getting pose in ros frame (camera/odom/sample) (x,y,z, quat_xyzw) 1 by 7 np.array
    print(f'Current pose: {my_tracking.get_translation(rotation=None)}') # getting pose (x,y,z,) 1 by 3 np.array
    print(f'Current pose: {my_tracking.get_translation(trans=False, rotation="xyz")}') # getting pose (euler_xyz) 1 by 3 np.array
    print(f'Current pose: {my_tracking.get_translation(trans=False, rotation="zyx", degrees=True)}') # getting pose (euler_zyx) 1 by 3 np.array in degrees
    print(f"Current pose: {my_tracking.get_matrix()}")    # getting pose Transformation matrix 4 by 4 np.array. [r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz, 0, 0, 0, 1]
    print(f"Current pose: {my_tracking.get_matrix(frame='ros')}")    # getting pose Transformation matrix in ros frame (camera/odom/sample) 4 by 4 np.array

    # Getting velocity and acceleration
    print(f'Current velocity: {my_tracking.get_velocity()}')    # getting velocity (linear vel x, y, z, angular vel x, y, z)
    print(f'Current acceleration: {my_tracking.get_acceleration()()}')  # getting velocity (linear acc x, y, z, angular acc x, y, z)
        
    time.sleep(1)   # loop every 1sec
```

## Note

You don't need to provide a camera serial number. If you don't provide it, the first camera found will be used. If you provide it, it'll error out if the specific camera is not found.


You don't need to call `my_tracking.start_tracking()` since it will be called when you do `my_tracking.update_pose()` first time.
You can, of course, call it by yourself, but calling it multiple times will throw an error.

You don't need to call `my_tracking.stop_tracking()` to stop the camera since the class Tracking deconstructor will call it automatically. 
You can, of course, call it by yourself, but calling it multiple times will throw an error.


If you need to restart a camera, then you can call stop_tracking(), then call start_tracking().


## Framing 
Currently, the library supports 2 frames: t265 hardware and ros (camera/odom/sample). Velocity and acceleration are always in t265 hardware frame (to be implemented).

!!! note

    Currently only translation is supported. Velocity and acceleration are not supported yet.



