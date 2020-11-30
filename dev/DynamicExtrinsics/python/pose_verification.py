
# This file contains functions to verify the poses of the charuco board
# used in the data collection process for Dynamic Extrinsics.
# In the 2-step model fitting process of the dynamic extrinsics algorithm, 
# the 1st step of fitting the dynamic 
# parameters consists of finding all the dynamic parameters
# AND the global poses of the charuco board for each of the positions
# the board is in during the data collection. The global poses of the
# board may or may not be relative the same world coordinate system
# (i.e. that of Imperx), though.  On the other hand, we do have the
# poses estimated from Imperx camera for the same board positions, which
# are in world coordinate frames.
# Now even though we cannot compare the absolute poses of the board estimated
# from both sources for each of the data collection positions, we can
# compare the relative pose of the board in these positions from these sources.
# For example, the relative pose between position "bottom-left" and "bottom-right"
# from dynamic parameter fitting and from the Imperx camera should match.
# 
# Author: Yang Chen (ychen@hrl.com)
# Date:  2020-10-05, modified 2020-11-10

import numpy as np
#from transforms3d.euler import euler2mat, mat2euler
import cv2 as cv

def compose_extrinsics(rvec, tvec):

	rmat, _ = cv.Rodrigues(rvec)
	return np.vstack( (np.hstack((rmat, tvec)),[0,0,0,1]) )


def find_relative_pose(ext1, ext2):
	''' Calculate and print the relative pose of 2 poses 
	    represented by the given extrinsics as 4x4 matrices
	'''
	# Relative pose is the pose of ext2 in the coordinate frame of ext1:
	relative_pose = np.linalg.inv(ext1) @ ext2
	tvec = relative_pose[0:3,3]

	rvec, _ = cv.Rodrigues(relative_pose[0:3,0:3])
	rvec = rvec.squeeze()
	rot_mag = np.linalg.norm(rvec)
	rot_dir = rvec/rot_mag
	print('4x4 matrix:\n', relative_pose)
	print('rvec, tvec (meters): \n   {}\n   {}'.format(rvec, tvec))
	print('Distance = {:.8f} (meters)'.format(np.linalg.norm(tvec)))
	print('Rotation axis = {},\n    magnitude = {:.8f} (degrees)'.format(rot_dir, np.degrees(rot_mag)))

	return rvec, tvec, rot_dir, rot_mag

def verify_fiducial_poses(poses, imperx_exts):
	'''
	Compare the poses represented in poses and in imperx_exts
	poses: list of tuples (rvec, tvec), one each for every data collection position
	       such as bottom-left, bottom-right, etc.
	imperx_exts: the extrinsics (pose) computed from Imperx corresponding to the
	       same data collection positions in "poses".
	Results: will be printed on the console.
	'''

	exts = [ compose_extrinsics(np.array([pose[0]]).transpose(), np.array([pose[1]]).transpose()) for pose in poses]

	# Find the relative pose based on poses and imperx_exts:
	print('Relative pose based on Imperx extrinsics:')
	rvec1, tvec1, rot_dir1, rot_mag1 = find_relative_pose(imperx_exts[0], imperx_exts[1])

	print('\nRelative pose based on dynamic parameter fitting:')
	rvec2, tvec2, rot_dir2, rot_mag2= find_relative_pose(exts[0], exts[1])

	print('\nDifference in distances of the 2 boards: {:.8f}'.format(abs(np.linalg.norm(tvec1)-np.linalg.norm(tvec2))))
	print('Alignment in rotation axes (dot-product): {:.8f}'.format(np.dot(rot_dir1, rot_dir2)))
	print('Difference in rotation magnitude (degrees): {:.8f}'.format(np.degrees(abs(rot_mag1-rot_mag2))))

if __name__ == '__main__':

	import json, sys
	from load_dataset import load_dataset
	np.set_printoptions(precision=8)

	# board poses file are from dynamic extrinsics fitting, and saved by fit_new_data.py
	POSE_JSON='board_poses.json'
	print('Pose Verification - loading relevant data from: {}'.format(POSE_JSON))
	try:
		board_poses = json.load(open(POSE_JSON))
	except:
		print('Error: File "{}" may not exist. Did you run fit_new_data.py first?'.format(POSE_JSON))
		sys.exit(1)
	poses = board_poses['poses']
	dataset_paths = board_poses['datasets']['dataset_paths']
	print('Datasets involved: ')
	for path in dataset_paths: print(' ', path)
	print('Poses estimated from dynamic parameter fitting (rvec, tvect):')
	for pose in poses: print(' ', pose)


	datasets = [load_dataset(path) for path in dataset_paths]
	# Prepare to do fiducial pose verification:

	for pos in range(1):
		imperx_exts = [dataset['impx_ext'][pos] for dataset in datasets]
		print("\nExtracting extrinsics from h5 file, index: ", pos)
		print("Imperx extrinsics:", imperx_exts)
		verify_fiducial_poses(poses, imperx_exts)

