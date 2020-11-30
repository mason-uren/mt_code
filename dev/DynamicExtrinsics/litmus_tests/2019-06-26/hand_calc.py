import numpy as np
import cv2

# This is a hand-reconstruction based on a test Scott and I did on 6/26 Wed 7:00pm, but
# the code he used had errors in there, so I used the Ximea extrinsics (the TV-3 ChaRuCo board)
# estimation + the dynamic extrinsic to calculate the Imperx pose of the TV-3 board
# The dynamic extrinsics are obtained with ccm.dynamic_extrinsics_correct_order() using
# cad_model=python/cad_models/cad_model_4_z0_angle_imp_5.json@156904, python/ccm.py@156887

# for (pan, tilt)=(89.99954986572266, 3.99959993362) (deg, raw)
#(TV_3)
Ximea_ext1 = \
np.array([[ 9.90700226e-01,  1.41422507e-02,  1.35325751e-01, -2.38547674e-01],
 [ 3.38151357e-03, -9.96835589e-01,  7.94189757e-02,  3.36061004e-01],
 [ 1.36020688e-01, -7.82227914e-02, -9.87613066e-01,  4.59955881e+00],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

#dynamic extrinsics 
d_ext1 =\
np.array([[ 0.99902798,  0.01977897, -0.03939389,  0.36719613],
       [-0.01628691,  0.9960688 ,  0.08707288, -0.31030985],
       [ 0.04096124, -0.08634664,  0.99542274,  0.07642024],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

# for (pan, tilt)=(87.99974822998047, 3.00104999542236) (deg, raw)
#(TV_3)
Ximea_ext2 = \
np.array([[ 9.85701078e-01,  1.25475200e-02,  1.68035546e-01, -3.95284016e-01],
 [ 2.00703184e-03, -9.98027183e-01,  6.27512027e-02,  4.18054230e-01],
 [ 1.68491414e-01, -6.15166754e-02, -9.83781654e-01,  4.57678307e+00],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]])

#dynamic extrinsics 
d_ext2 = \
np.array([[ 0.99988683,  0.01443705, -0.00423051,  0.37086673],
 [-0.01410003,  0.9973716,   0.07107093, -0.31266265],
 [ 0.00524544, -0.07100324,  0.99746229,  0.08610141],
 [ 0.,          0.,          0.,          1.        ]])
 

print('TV ChaRuCo pose in ImperX for pan/tilt #1:')

pose1 = np.matmul(d_ext1, Ximea_ext1)
pose1_rvec,_ = cv2.Rodrigues(pose1[0:3,0:3])
print('tvec={},\n rvec={}'.format(pose1[0:3,3],pose1_rvec[:,0])) 

print('TV ChaRuCo pose in ImperX for pan/tilt #2:')

pose2 = np.matmul(d_ext2, Ximea_ext2)
pose2_rvec,_ = cv2.Rodrigues(pose2[0:3,0:3])
print('tvec={},\n rvec={}'.format(pose2[0:3,3],pose2_rvec[:,0])) 
