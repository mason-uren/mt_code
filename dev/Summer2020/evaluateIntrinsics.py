# Last Modified: 6/30/2020 Changes made by Aaron
# Used for evaluating camera calibration results on a given dataset

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

##########################
# User-Defined Constants #
##########################

PTU = 'PTU4' # Or PTU3
position = 'BottomRight' # Or BottomRight
rndNum = 2 # Or 2
cameraName = 'Ximea' # Or Ximea
cameraId = 2 # Or 0 (PTU3),1 (PTU4) for Ximea
folderPath = \
'\\\\netapp1-svm3/issl/issl_archive/Archives/2019/Boeing-DR/ClosedLoopMetrology/2020/DataSets/2020-08-04-DataCollection/' + PTU + '/' + position + '/Round' + str(rndNum) + '/'
#imagePath = folderPath + cameraName + '/'
#writePath = '\\\\netapp1-svm3/issl/issl_archive/Archives/2019/Boeing-DR/ClosedLoopMetrology/2020/DataSets/2020-08-04-DataCollection/Analysis/' + PTU + '/' + position + '/Round' + str(rndNum) + '/' + cameraName + '/'

if cameraName == 'Imperx':
    date = '2020-06-26'
elif cameraId == 1:
    date = '2020-07-21'
elif cameraId == 0:
    date = '2020-07-15'
    
#intrinsicsPath = 'C:/Users/aofeldman/Desktop/ClosedLoopMetrology/Code/src/Metrology2020_VS/Metrology2020/Config/CameraIntrinsics/' + cameraName + '_id' + str(cameraId) +'/' + date + '/'

imagePath = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined/AFround18'
writePath = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined/AFround18/results/'
intrinsicsPath = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined/AFround18/'

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

##########################
# Helper functions       #
##########################

# Compute mean reprojection error over a dataset given the calibration results
# Assumes blurred already
def error_for_dataset(image_list, intrinsics, dist, cameraName, RMS = True):
    image_errors = []
    for i, image in enumerate(image_list):
    
        # Aaron testing out different parameter options
        parameters = cv2.aruco.DetectorParameters_create()
        
        # Empirically observed that for the dataset slightly reducing this value
        # from the default results in significantly better detection for the 
        # imperx
        if cameraName == 'imperx':
            parameters.minMarkerPerimeterRate = 0.025
        
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
            # Compute the reprojection error
            R, _ = cv2.Rodrigues(rvec)
            if RMS:
                mean_error = charuco_RMS_reprojection_error(board, chorners, chids, R, tvec, intrinsics, dist)
            else:
                mean_error = charuco_reprojection_error(board,chorners,chids,R,tvec,intrinsics,dist)
            image_errors.append(mean_error)
        else:
            print("Skipping: Failed to find charuco board in image " + str(i))
    return image_errors, np.mean(image_errors)

##########################
# Main code              #
##########################

DISP_IMG = False

#filenames = glob.glob('C:/Users/aofeldman/Desktop/calibration6-26/*.tif')
#filenames = glob.glob('C:/Users/aofeldman/Desktop/imperxData/*.tif')
#filenames = glob.glob(imagePath + '/*.tiff')
filenames = glob.glob(imagePath + '/*.tif')

print('Finished collecting images')

# # Reorder based on initial numbering
# imageNumbers = []
# for i in range(len(filenames)):
#     pos = len(filenames[i]) - 1 - filenames[i][::-1].find('_') # Find position of last underscore
#     imageNumbers.append(int(filenames[i][pos+1:].replace('.tiff', "")))
#     imageNumbers.append(int(filenames[i][pos+1:].replace('.tif', "")))

# filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
# imageNumbers = np.sort(imageNumbers)    

filenames = [imagePath + '/' + 'ximea' + str(i) + '.tif' for i in range(1,6)]
imageNumbers = list(range(1,6))

image_list = []
for img in filenames:
    image_list.append(cv2.imread(img, 0))
    print("Read image " + img)

#image_list = [cv2.imread(img, 0) for img in filenames]

print('Size of images: ', image_list[0].shape[0])

print("Number of Images in Folder: " + str(len(image_list)))

# Slightly blur all images in dataset to prevent aliasing
image_list = [cv2.blur(image, (3,3)) for image in image_list]

saveFile = open(writePath + "imageResults.txt", "w+")
saveFile.write('Intrinsics/Dist Used: ' + intrinsicsPath + cameraName.lower() + '_<>\n\n')

# TODO: Create a .txt file to write the errors and estimated poses to
image_errors, mean_error = error_for_dataset(image_list, intrinsics, dist, cameraName)

for i, image in enumerate(image_list):
    print('image: ' + str(i))
    # Generate quiver plots
    # Also returns corners but not needed
    # This reprojection_error is not RMS
    extrinsics,reprojection_error, _ = estimate_Pose_Charucoboard_Ximea(image_list[i], board, intrinsics, dist)
    #print('i', i, 'error', reprojection_error)
        
    rvec,tvec = decompose_Extrinsics(extrinsics)
    
    # Convert to a rotation vector
    rvec, _ = cv2.Rodrigues(rvec) 
    
    saveFile.write("Image: " + cameraName.lower() + ' [ ' + str(cameraId) + ']' + '_' + str(imageNumbers[i]) + '\n')
    saveFile.write("RMS error: " + str(image_errors[i]) + '\n')
    saveFile.write("rvec: " + str(np.squeeze(rvec)) + '\n')
    saveFile.write("tvec: " + str(tvec) + '\n')
    saveFile.write('\n')
    
    # Aaron testing out different parameter options
    parameters = cv2.aruco.DetectorParameters_create()
    if cameraName == 'imperx':
        parameters.minMarkerPerimeterRate = 0.025
    
    quiver_image1 = generate_reprojection_error_quiver_plot_ids(image_list[i], aruco_dict, boards['TV_3'], rvec, tvec,
                                                           intrinsics, dist, parameters)
    
    width = int(quiver_image1.shape[1] / 4)
    height = int(quiver_image1.shape[0] / 4)
    dim = (width, height)
    newIm = cv2.resize(quiver_image1, dim)
    if DISP_IMG:
        cv2.imshow('quiver' + str(i), newIm)
    
    if os.path.exists(writePath + 'quivers'):
        cv2.imwrite(writePath + 'quivers/quiver' + str(i) + '.jpg', quiver_image1)
    else:
        os.mkdir(writePath + 'quivers')
    
    heat_image = generate_reprojection_error_heat(image_list[i], aruco_dict, boards['TV_3'], rvec, tvec, intrinsics, dist, parameters, 16)
    width = int(heat_image.shape[1] / 4)
    height = int(heat_image.shape[0] / 4)
    dim = (width, height)
    newIm = cv2.resize(heat_image, dim)
    if DISP_IMG:
        cv2.imshow('heatmap'+str(i), newIm)
    
    if os.path.exists(writePath + 'heatmaps'):
        cv2.imwrite(writePath + 'heatmaps/heatmap' + str(i) + '.jpg', newIm)
    else:
        os.mkdir(writePath + 'heatmaps')
    cv2.waitKey(1)
cv2.destroyAllWindows()

saveFile.close()
