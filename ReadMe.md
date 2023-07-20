# Intel T265 wrapper
This wrapper is to help to simplify your interaction with T265. The camera will be automatically initialized and closed.

This is very minimalistic. The point is to provide the simplest possible way of using T265 cameras. 
## Ver 0.1.xx
You can specify a camera by its serial number. If you want to use more than one T265, you can create multiple instances of Tracking().

## Important Note
Starting [Ver. 1.54.1](https://github.com/IntelRealSense/librealsense/releases/tag/v2.54.1) Intel has dropped supports for T265 entirely. 
Hence, this library requires pyrealsense2 earlier than Ver. 2.53.1 
Starting Ver 0.1.3 this is part of requirements.txt.

## Installation
this library is pip installable
```bash
pip install t265
```


## How to use
Here is a sample code
```python
import time
from t265 import Tracking

my_tracking = Tracking(camera_sn='133122110783') # Provide your camera serial number. It should be on the camera bottom

N = 10     # will show pose, vel, acc for 10 times
for _ in range(N):
    my_tracking.update_pose(wait=True) # wait=True will block the code till you get data from camera.     
    print(f'Current pose: {my_tracking.get_translation()}')    # getting pose (x,y,z, quat_xyzw) 1 by 7 np.array
    print(f'Current velocity: {my_tracking.get_velocity()}')    # getting velocity (linear vel x, y, z, angular vel x, y, z)
    print(f'Current acceleration: {my_tracking.get_acceleration()()}')  # getting velocity (linear acc x, y, z, angular acc x, y, z)
    
    time.sleep(1)   # loop every 1sec
```


You don't need to call `my_tracking.start_tracking()` since it will be called when you do `my_tracking.update_pose()` first time.
You can, of course, call it by yourself, but calling it multiple times will throw an error.

You don't need to call `my_tracking.stop_tracking()` to stop the camera since the class Tracking deconstructor will call it automatically. 
You can, of course, call it by yourself, but calling it multiple times will throw an error.


If you need to restart a camera, then you can call stop_tracking(), then call start_tracking().
