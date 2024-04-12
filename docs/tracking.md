# Tracking class APIs

## T265 Frame
### T265 Body frame 
The default frame T265 returns.

Z pointing the opposite of the camera.

            Y
            |   / Z
            |  /
            | /
     X______|/ 

### ROS frame 
The default frame T265 returns if you use ros package.

Y pointing out of the camera. VR convention.

        Z
        |
        |
        |______ X
       /
      /
     Y


## APIs
::: t265.Tracking.Tracking
