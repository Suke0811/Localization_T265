# Frame Translation
$\text{transform}(\text{current\_camera\_frame}, \text{return\_all\_frames}=\text{False})$

## Transformation Matrix definition:
1. $T_{\text{c}}^{\text{ros}}$ — Transformation matrix from the camera frame to the ROS frame. (A constant)
2. $T_{\text{E}}^{\text{c}_k}$ — Transformation matrix of the current camera frame.
3. $T_{\text{E}}^{\text{c}_0}$ = Transformation matrix of the initial camera frame with respect to the Earth frame.

 
## Transformation Calculations:
1. $T_{\text{c}_k}^{\text{c}_0} = T^{\text{E}}_{\text{c}_0} \cdot T_{\text{E}}^{\text{c}_k}$ — Transformation matrix from the initial frame to the current camera frame.
2. $T^{\text{c}_k}_{\text{ros}_0} = T^{\text{c}}_{\text{ros}} \cdot T^{\text{c}_k}_{\text{c}_0}$ — Combined transformation matrix from the ROS frame to the current camera frame.
3. $T^{\text{ros}_k}_{\text{ros}_0} = T^{\text{c}_k}_{\text{ros}_0} \cdot T_{\text{c}}^{\text{ros}}$ — Final transformation matrix from the ROS frame to the transformed current frame.


## APIs and implementation :

[APIs](frame_handler.md#t265.FrameHandler.FrameHandler.transform)
