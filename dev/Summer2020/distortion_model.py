# Code to plot distortions and compare distortions of different camera intrinsics
# Camera distortion model reference: 
# https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html
# Author: Yang Chen
# Date: 2020-09-17

import numpy as np
import matplotlib.pyplot as plt

# we only use k1, k2, p1, p2, and k3, 5x distortion coeffcients here:
# Imperx 2020-09-16 intrinsics
cameraMatrix1=np.array([[1.164615605782963758e+04, 0.000000000000000000e+00, 2.514452390259944423e+03],
			   [0.000000000000000000e+00, 1.164387356794783045e+04, 2.590569005274415304e+03],
			   [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])
distCoeffs1=[-8.175897205103049847e-02, -4.033204383884095301e-02, 1.814373489199908051e-04, 
 			-1.261923404440164166e-04, -1.459543735024295893e-02]
# Imperx 2020-06-26 intrinsics
cameraMatrix2=np.array([[1.159204181717728716e+04, 0.000000000000000000e+00, 2.500186283919203561e+03],
			   [0.000000000000000000e+00, 1.158986721742450572e+04, 2.604137882761427591e+03],
			   [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])
distCoeffs2=[-8.477697972268238846e-02, 5.615900191535098951e-02, 1.544669397537231845e-04, 
              1.142501985655113630e-04, -6.835885137763140218e-01]


def distort(cameraMatrix, distCoeffs, x, y=0.0):

	''' Given 3x3 cameraMatrix, 5x1 distCoeffs, and x & y
		where x & y are normalized world-coordinate values
		(i.e., x=X/Z, y =Y/Z),
	    return distorted and undistorted image coordinates u & v in a list:
	    [ud, vd, uu, vu]

	    x & y: given in (Z) normalized coordinates
	'''

	fx = cameraMatrix[0,0]
	fy = cameraMatrix[1,1]
	cx = cameraMatrix[0,2]
	cy = cameraMatrix[1,2]
	k1 = distCoeffs[0]
	k2 = distCoeffs[1]
	k3 = distCoeffs[4]
	p1 = distCoeffs[2]
	p2 = distCoeffs[3]
    # k4 = k5 = k6 = s1 = s2 = s3 = s4 = 0

	r2 = x**2 + y**2

	# Radial distortion multiplier for x & y:
	rd = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**4
	# Tangential distortion factor for x & y:
	tdx = 2 * p1 * x * y + p2 * (r2 + 2 * x**2 )
	tdy = p1 * ( r2 + 2 * y**2 )

	# undistorted u & v
	uu = fx * x + cx
	vu = fy * y + cy
	# the distorted u & v
	ud = fx * ( rd * x + tdx ) + cx
	vd = fy * ( rd * y + tdy ) + cy

	return [ud, vd, uu, vu]


cameraMatrix = cameraMatrix1
distCoeffs = distCoeffs1
print('Intrinsics: --- ')
print(np.array(cameraMatrix))
print(distCoeffs)

deltX = 0.02
# range of x' (in the OpenCV web page referenced at the top)
x_min, x_max = (-0.22, 0.22+1.0E-6)

print('Distance (Z) normalized x, range of (min, max): ', x_min, x_max)
pairs=[]

for x in np.arange(x_min, x_max, deltX):
	ud, _, uu, _ = distort(cameraMatrix, distCoeffs, x)
	pairs.append((ud, uu, x))

p1 = np.array(pairs)
print(p1)

cameraMatrix = cameraMatrix2
distCoeffs = distCoeffs2
print('Intrinsics: --- ')
print(np.array(cameraMatrix))
print(distCoeffs)

pairs=[]
for x in np.arange(x_min, x_max, deltX):
	ud, _, uu, _ = distort(cameraMatrix, distCoeffs, x)
	pairs.append((ud, uu, x))

p2 = np.array(pairs)
print(p2)

plt.figure()
plt.plot(p1[:,1],p1[:,0]-p1[:,1],p2[:,1],p2[:,0]-p2[:,1])
plt.xlabel('Undistorted U (pixels)')
plt.ylabel('(ud - uu) (pixels)')
plt.title('Difference between distorted and undistorted U values')
plt.legend(['Imperx Intrinsics 2020-09-16','Imperx Intrinsics 2020-06-26'])
plt.grid()

# Not sure I didn't this one correctly, but it's not as informative now.
# p2 array turns out to be 1 element longer, so I can choose to cut either
# the last element or the first (shown below). However the diff is not
# very meaningful because (u0, v0)=(cx, cy) also affects the results, not 
# just distortions.
# rev.2: effects of u0/v0 are taken out.
cx1 = cameraMatrix1[0,2]
cx2 = cameraMatrix2[0,2]
plt.figure()
plt.plot(p1[:,2],(p1[:,0]-cx1) - (p2[:,0] - cx2) )
#plt.plot(p1[:,1],p1[:,0], p1[:,1], p2[:,0] )
plt.xlabel('Normalized X (no unit)')
plt.ylabel('Difference in U (pixels)')
plt.title('Difference between 2020-09-16 & 2020-06-26 in U')
plt.legend(['Diff btw 2020-09-16 & 2020-06-26 in distorted U'])
plt.grid()
plt.show()
