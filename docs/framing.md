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

------
## Acceleration frame conversion


Given:

- \( \mathbf{T}_{current} \): The current transformation matrix of the camera frame, a \(4 \times 4\) matrix.
- \( \mathbf{a} \): Linear acceleration in the initial frame, represented as a \(3 \times 1\) vector.
- \( \boldsymbol{\alpha} \): Angular acceleration in the initial frame, represented as a \(3 \times 1\) vector.
- \( \mathbf{T}_{trans} \): The transformation matrix to the desired frame, a \(4 \times 4\) matrix.

We want to find the transformed linear acceleration \( \mathbf{a}' \) and angular acceleration \( \boldsymbol{\alpha}' \) in the desired frame.

1. **Extract the rotation matrices and translation vectors:**

   - \( \mathbf{T}_{current} \):

     \[
     \mathbf{T}_{current} = \begin{bmatrix}
     \mathbf{R}_{current} & \mathbf{t}_{current} \\
     \mathbf{0}_{1 \times 3} & 1
     \end{bmatrix}
     \]

     where \( \mathbf{R}_{current} \) is the \(3 \times 3\) rotation matrix and \( \mathbf{t}_{current} \) is the \(3 \times 1\) translation vector.

   - \( \mathbf{T}_{trans} \):

     \[
     \mathbf{T}_{trans} = \begin{bmatrix}
     \mathbf{R}_{trans} & \mathbf{t}_{trans} \\
     \mathbf{0}_{1 \times 3} & 1
     \end{bmatrix}
     \]

     where \( \mathbf{R}_{trans} \) is the \(3 \times 3\) rotation matrix and \( \mathbf{t}_{trans} \) is the \(3 \times 1\) translation vector.

2. **Compute the inverse transformation matrix of the initial frame \( \mathbf{T}_{init}^{-1} \):**

   \[
   \mathbf{T}_{init}^{-1} = \begin{bmatrix}
   \mathbf{R}_{init}^{T} & -\mathbf{R}_{init}^{T} \mathbf{t}_{init} \\
   \mathbf{0}_{1 \times 3} & 1
   \end{bmatrix}
   \]

   Here, \( \mathbf{R}_{init}^{T} \) is the transpose of the initial rotation matrix, and \( -\mathbf{R}_{init}^{T} \mathbf{t}_{init} \) is the transformed translation vector.

3. **Transform the linear and angular accelerations:**

   \[
   \mathbf{a}_{ck} = \mathbf{R}_{current}^{T} \mathbf{a}
   \]

   \[
   \boldsymbol{\alpha}_{ck} = \mathbf{R}_{current}^{T} \boldsymbol{\alpha}
   \]

4. **Compute the final rotation matrix \( \mathbf{R}_{final} \):**

   \[
   \mathbf{R}_{final} = \mathbf{R}_{trans} \mathbf{R}_{current}
   \]

5. **Transform the linear acceleration in the desired frame:**

   \[
   \mathbf{a}' = \mathbf{R}_{final} \mathbf{a}_{ck}
   \]

6. **Transform the angular acceleration in the desired frame:**

   \[
   \boldsymbol{\alpha}' = \mathbf{R}_{final} \boldsymbol{\alpha}_{ck}
   \]

Thus, the transformed linear and angular accelerations in the desired frame can be represented as:

\[
\mathbf{a}' = \mathbf{R}_{trans} \mathbf{R}_{current}^{T} \mathbf{a}
\]

\[
\boldsymbol{\alpha}' = \mathbf{R}_{trans} \mathbf{R}_{current}^{T} \boldsymbol{\alpha}
\]
