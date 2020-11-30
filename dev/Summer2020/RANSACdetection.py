# Last Modified: 8/24/2020 Changes made by Aaron

# Aaron added for personal import setup
from setupImports import setup_path
setup_path()

from hardware.ximea.Driver.client.ximea_client import *
from Charuco_Specific.ChArUcoHelpers import *
from Charuco_Specific.CharucoBoards import *
import numpy as np
import math
import time
import shutil
import glob
import pdb
import os
import matplotlib.pyplot as plt

PTU = 'PTU4' # Or PTU3
position = 'BottomLeft' # Or BottomRight
rndNum = 1 # Or 2
cameraName = 'Ximea' # Or Ximea
cameraId = 1 # Or 0 (PTU3),1 (PTU4) for Ximea
folderPath = \
'\\\\netapp1-svm3/issl/issl_archive/Archives/2019/Boeing-DR/ClosedLoopMetrology/2020/DataSets/2020-08-04-DataCollection/' + PTU + '/' + position + '/Round' + str(rndNum) + '/'
imagePath = folderPath + cameraName + '/'
#writePath = '\\\\netapp1-svm3/issl/issl_archive/Archives/2019/Boeing-DR/ClosedLoopMetrology/2020/DataSets/2020-08-04-DataCollection/Analysis/' + PTU + '/' + position + '/Round' + str(rndNum) + '/' + cameraName + '/'

if cameraName == 'Imperx':
    date = '2020-06-26'
elif cameraId == 1:
    date = '2020-07-21'
elif cameraId == 0:
    date = '2020-07-15'
    
intrinsicsPath = 'C:/Users/aofeldman/Desktop/ClosedLoopMetrology/Code/src/Metrology2020_VS/Metrology2020/Config/CameraIntrinsics/' + cameraName + '_id' + str(cameraId) +'/' + date + '/'

# Load the appropriate intrinsics, distCoeffs
intrinsics = np.load(intrinsicsPath + cameraName.lower() + '_intrinsics.npy')
dist = np.load(intrinsicsPath + cameraName.lower() + '_distCoeffs.npy')

##########################
# Charuco Board Consts   #
##########################

# Aaron Changed to TV_4 was TV_3, put back to TV_3 since used big monitor again
squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

# Load the appropriate images

# For now, just read in one image

filenames = glob.glob(imagePath + '/*_35.tiff')

print('Finished collecting images')

# Reorder based on initial numbering
imageNumbers = []
for i in range(len(filenames)):
    pos = len(filenames[i]) - 1 - filenames[i][::-1].find('_') # Find position of last underscore
    imageNumbers.append(int(filenames[i][pos+1:].replace('.tiff', "")))

filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
imageNumbers = np.sort(imageNumbers)    

image_list = []
for img in filenames:
    image_list.append(cv2.imread(img, 0))
    print("Read image " + img)

print('Size of images: ', image_list[0].shape[0])

print("Number of Images in Folder: " + str(len(image_list)))

# Slightly blur all images in dataset to prevent aliasing
image_list = [cv2.blur(image, (3,3)) for image in image_list]

errBoundList = range(20, 21)
fractionList = [1 - 1e-1, 1 - 1e-3, 1 - 1e-5, 1 - 1e-7]
exageration_factor = 60

for i, image in enumerate(image_list):
    parameters = cv2.aruco.DetectorParameters_create()
    
    markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
    
    # if we dont find enough points skip
    if (ids is not None and len(ids) > 8):
        ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
        
        for maxErr in errBoundList:
            for fractionInlier in fractionList:
                retval, rvec, tvec, inliers = estimate_pose_ransac(board, chorners, chids, intrinsics, dist, 100, maxErr, fractionInlier)
                
                if retval:
                    inliers = np.squeeze(inliers)
                    
                    # Extract the appropriate chorners
                    # Should have shape numInliers X 2
                    inlierChorners = np.array([np.squeeze(chorners)[inliers[i]] for i in range(len(inliers))])
                    
                    error_vectors, error_dists = charuco_reprojection_error_for_every_corner(board, chorners, chids, rvec, tvec, intrinsics, dist)
                    
                    print('Maximum reprojection errror was: ' + str(np.max(error_dists)))
                    
                    # Extract the inlier reprojection errors
                    inlier_errs = [error_vectors[inliers[i]] for i in range(len(inliers))]
                    
                    title = 'maxErr='+str(maxErr)+' fractionInlier='+str(fractionInlier)
                    
                    highlightImage = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    # for row in range(inlierChorners.shape[0]):
                    #     highlightImage[y - 20: y + 20, x - 20: x + 20] = (0, 0, 255)
                    
                    for row in range(inlierChorners.shape[0]):
                        error_term = exageration_factor*inlier_errs[row]
                        y = int(inlierChorners[row, 1])
                        x = int(inlierChorners[row, 0]) 
                        point = (x, y)
                        cv2.arrowedLine(highlightImage, point, tuple(np.add(point, error_term).astype(int)), (0, 0, 255),exageration_factor,tipLength=.6)

                    cv2.imshow(title, cv2.resize(highlightImage, (int(0.15 * highlightImage.shape[1]), int(0.15 * highlightImage.shape[0]))))
                    cv2.waitKey(0)
                else:
                    print('solvePnPRansac failed using ' + title)