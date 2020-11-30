# Closed Loop Metrology Dynamic Extrinsics
# Development code for proof-of-concept analytical solution of dynamic extrinsics. 
Deepak Khosla, May 31, 2019

# Uses Anthony's 15 parameter model instaed of Kenny's old 12 parameter model
# Ximea -> Tilt -> Pan -> Base -> Imperx
# Unknown (15)
yaw_ximea_tilt
pitch_ximea_tilt 
roll_ximea_tilt 
x_ximea_tilt 
y_ximea_tilt 
z_ximea_tilt
x_tilt_pan 
y_tilt_pan 
z_tilt_pan
yaw_base_imperx 
pitch_base_imperx 
roll_base_imperx 
x_base_imperx 
y_base_imperx 
z_base_imperx
# Known (2)
pitch_tilt_pan 
yaw_pan_base

# How to use
Run `mainlime.m`  
# WARNING: Requires symbolic and optimization toolboxes to run
# Sets up CAD model, computes dynamic extrinsics symbolically and solves for
# it by numerically using CAD-based ground-truth extrinsics at N PT
# positions (15 unknowns, 12 per extrinsic gives 12N equations to make it
# an over-determined system).
# Benefits
# 1. This can be used instead of John Miller's CAD model to verify HRL results
# 2. Kenny's model/code can be replaced with this one. See solveparams function



