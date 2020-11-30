# Closed Loop Metrology Dynamic Extrinsics

HRL Laboratories, LLC\
Kenny Chen - kjchen@hrl.com

Development code for proof-of-concept analytical solution of dynamic extrinsics. Run `main.m` to compute the extrinsic matrix between the Ximea and Imperx, in addition to the matrix decompositions. Uses `fsolve` to solve the system of nonlinear equations between the general decomposed extrinsic matrix (Ximea to Imperx) and the input solution matrix from OpenCV calibration.

Some stuff you can change:

- Which extrinsic matrix to use to back-calculate the twelve parameters `extrinsics.i_T_x`
- the new tilt/pan angles to calculate `new_t` and `new_p`

## Camera Specifications

### Ximea

    Native Resolution       |       7920 (W) x 6004 (H)
    Sensor Size             |       36.4 mm (H) x 27.6 mm (V)
    Focal Length            |       200 mm
    FOV                     |       10.4 deg (H) x 7.894 deg (V)
    Ideal Intrinsics        |       fx = 43516.48 px
                                    fy = 43507.25 px
                                    cx = 3960 px
                                    cy = 3002 px

### Imperx

    Native Resolution       |       5120 (W) x 5120 (H)
    Sensor Size             |       23 mm (H) x 23 mm (V)
    Focal Length            |       50 mm
    FOV                     |       25.91 deg (H) x 25.91 deg (V)
    Ideal Intrinsics        |       fx = 11130.43 px
                                    fy = 11130.43 px
                                    cx = 2560 px
                                    cy = 2560 px