
'''
2019-07-12 Yang Chen ychen@hrl.com
Experiment for comparing different ways of computing the ArUco features and doing the corner “interpolation”.
What I wanted to see is if there is any difference in using the blurred image to do corner interpolation vs. 
using the original (unblurred) image, and if the type of blur affects the interpolation results.  
The quick answer is basically not match difference that these approaches will make.

The following, I print the number of features and corners detected, and then compute the mean and std 
in X and Y (together), and the abs() min & max, of the differences between the corner locations computed 
using the original image and using the blurred image (which is what we do now in Scott’s code as of today).  
I also compare between the blur using 3x3 blur() (box-blur) or 3x3 GaussianBlur().

Box Blur:
num_markers=364
num_corners1 = 673
num_corners2 = 673
mean=0.0141, stdev=0.0310
min=0.0000 max=0.1187

GaussianBlur()
num_markers=364
num_corners1 = 673
num_corners2 = 673
mean=0.0106, stdev=0.0239
min=0.0000 max=0.0980

The above experiment was done using Ximea intrinsic calibration image: ximea_intrinsics_1562016103.836947.jpg
found in the archive area under DataSet/Calibration Images/.

Conclusions:
There appears to be some small differences between using the original image vs. blurred image for corner 
interpolation, and that difference is more pronounced when the blur is carried out using box-blur 3x3 than 
with a 3x3 Gaussian. But in either case, the difference seem to be small (max diff in X and Y is ~0.1 pixels).

The other interesting experiment I did was to use the original image for feature detection, and I didn’t 
have any problem with that for the single Ximea image I tried. But earlier on Scott had problem with 
un-filtered & un-resized Ximea images, which prompted us to use blurred image in the first place.
'''

# To use this code, just comment/uncomment certain sections of the code, adjust image file to read.

# Tested with Python3.7 and opencv-python:3.4.5.20 and opencv-contrib-python:3.4.5.20 for MacOS.

from CharucoBoards import *

import cv2
import numpy as np

#**************************#
#      Board Settings      #
#**************************#
squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

#image = cv2.imread('./ximea_sub.png')
image = cv2.imread('./ximea_org.jpg')
if image is None:
	print('Cannot read image.')
	exit(1)
image_org1 = np.copy(image)
image_org2 = np.copy(image)
image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
image_gray_org = np.copy(image_gray)

image_gray_blur = cv2.GaussianBlur(image_gray, (3,3), 0)
print('GaussianBlur()')

#image_gray_blur = cv2.blur(image_gray, (3,3))
#print('Box Blur')

#cv2.imshow('Input Image',image)
#cv2.waitKey(10)

markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_gray_blur, aruco_dict)
print('num_markers={}'.format(len(ids)))

ret, corners1, cids1 = cv2.aruco.interpolateCornersCharuco(markers, ids, image_gray_blur, board)
print('num_corners1 = {}'.format(len(cids1)))

ret, corners2, cids2 = cv2.aruco.interpolateCornersCharuco(markers, ids, image_gray_org, board)
print('num_corners2 = {}'.format(len(cids2)))

diff = corners1-corners2
print('mean={:.4f}, stdev={:.4f}'.format(np.mean(diff),np.std(diff)))
print('min={:.4f} max={:.4f}'.format(np.min(np.abs(diff)), np.max(np.abs(diff))))

cv2.aruco.drawDetectedCornersCharuco(image_org1, corners1, cids1,(0,255,0))
cv2.aruco.drawDetectedCornersCharuco(image_org2, corners2, cids2,(0,255,0))
#cv2.imwrite('./corners_image_org.png', image_org2)
#cv2.imwrite('./corners_blur3x3.png', image_org1)
cv2.imshow('Detected corners',image_org1)
cv2.waitKey(0)
