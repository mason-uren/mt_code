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
import os
import matplotlib.pyplot as plt
import pdb
import scipy.optimize
import math
import time
import estimateBeam
import altMin
import scipy.stats as stats
from fitVaryingModel import obtainMLSfit, evaluatedModel, plotPointEstimates
from dynamicIntrinsicsHelpers import searchForFocus, loadFixedModels, error_for_dataset, altThinLensPose

##########################
# Charuco Board Consts   #
##########################

squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

##########################
# Helper Functions       #
##########################
    
if __name__ == '__main__':

    plt.close('all')
    plt.rcParams["scatter.marker"] = 'x'

    ##########################
    # Load Images from Path  #
    ##########################
        
    # Need to separate the imperx and ximea
    cameraName = 'ximea' # or imperx
    
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    
    f = 200 * 1e-3 / 1.0372608996079156 
    
    path = 'C:/Users/aofeldman/Desktop/testCollection8-31'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 6
    
    trainingRounds = set(range(numRounds+1))
    testRounds = []
        
    # First, load the fixed models using loadFixedModels
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
    
    fPoint = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
        
    inliers = np.array([1, 2, 4, 0], dtype=np.int64)
    
    trainingIntrinsics = [(intrinsicsList[0] + intrinsicsList[1])[i] for i in inliers]
    trainingDistortion = [(distortionList[0] + distortionList[1])[i] for i in inliers]
    
    correspondingFocus = []
    for AFround in roundList[0]:    
        correspondingFocus.append(searchForFocus(path + '/' + subfolderPrefixes + str(AFround) + '/' + relFocusPrefix + str(AFround) + '.txt'))
    
    # Since using lambda d: 1, turns into static LS so any focus (e.g. 100) will do
    polynomials = obtainMLSfit([correspondingFocus[i] for i in inliers], trainingIntrinsics, trainingDistortion, [100], {'f':1, 'u0':0,'v0':0, 'k1':1, 'k2':1, 'k3':1}, True, False, False, lambda d: 1)
    pList = []
    image0 = []
    
    polyIntrinsicsList = []
    polyDistList = []
    # Evaluate polynomials at each focus position
    for focus in correspondingFocus:
        intrinsics, dist = evaluatedModel(polynomials, focus)
        polyIntrinsicsList.append(intrinsics)
        polyDistList.append(dist)
    
    image0Z = [[], []]
        
    for i, AFround in enumerate(roundList[0]):
        intrinsics = intrinsicsList[0][i]
        dist = distortionList[0][i]
        
        foldername = path + '/' + subfolderPrefixes + str(AFround)
    
        # Load in all images for the given focus setting
        filenames = glob.glob(foldername + '/' + '*.tif')
        
        imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
        filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
        imageNumbers = np.sort(imageNumbers)
        
        # Read in and mildly blur images
        image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
        
        image_errors = []
        
        print('On round: ' + str(AFround))
        print('Size of Images Collected = ' + str(image_list[0].shape))
        print("Number of Images in Folder: " + str(len(image_list)))
        
        chornersList = []
        chidsList = []
        # Given the image list identify the chorners once
        for image in image_list:
            parameters = cv2.aruco.DetectorParameters_create()
            
            markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
            
            # if we dont find enough points skip
            if (ids is not None and len(ids) > 8):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                chornersList.append(chorners)
                chidsList.append(chids)
        
        pointImgErr, pointMeanErr, pointRs, pointTs = error_for_dataset(chornersList, chidsList, intrinsics, dist, cameraName)
                
        fitImgErr, fitMeanErr, fitRs, fitTs = error_for_dataset(chornersList, chidsList, polyIntrinsicsList[i], polyDistList[i], cameraName)

        image0Z[0].append(pointTs[0, 2])
        image0Z[1].append(fitTs[0, 2])
    
    measuredZs = np.array([181, 125, 135.5, 147.5, 159, 169.5]) * 2.54 / 100
    order = np.argsort(focusList[0])
    orderedFocus = np.array(focusList[0])[order]
    measuredZs = measuredZs[order]
    image0Z[0] = np.array(image0Z[0])[order]
    image0Z[1] = np.array(image0Z[1])[order]

    plt.figure()
    plt.title('Comparing Measured and Estimated Change in Z: Image 0')
    plt.scatter(orderedFocus, measuredZs - measuredZs[0], label='Measured')
    plt.scatter(orderedFocus, image0Z[0] - image0Z[0][0], label='Point Estimated')
    plt.scatter(orderedFocus, image0Z[1] - image0Z[1][0], label='Fit')
    plt.xlabel('Focus Position')
    plt.ylabel('Z [m]')
    plt.legend()
    
    plt.figure()
    plt.title('Difference Between Measured and Estimated Change in Z: Image 0')
    plt.scatter(orderedFocus, measuredZs - measuredZs[0] - (image0Z[0] - image0Z[0][0]), label='Measured - Point Estimated')
    plt.scatter(orderedFocus, measuredZs - measuredZs[0] - (image0Z[1] - image0Z[1][0]), label='Measured - Fit')
    plt.xlabel('Focus Position')
    plt.ylabel('Delta Z [m]')
    plt.legend()
    
    plt.figure()
    plt.title('Measured against Point Estimated Z: Image 0')
    plt.scatter(measuredZs, image0Z[0])
    poly = np.polyfit(measuredZs, image0Z[0], 1)
    plt.plot(measuredZs, np.polyval(poly, measuredZs), linestyle='dashed', label=(f'LS Fit: {poly[0]:.4f} measured + {poly[1]:.4f}'))
    plt.xlabel('Measured Z [m]')
    plt.ylabel('Point Estimated Z [m]')
    plt.legend()
    
    plt.figure()
    plt.title('Measured against Fit Z: Image 0')
    plt.scatter(measuredZs, image0Z[1])
    poly = np.polyfit(measuredZs, image0Z[1], 1)
    plt.plot(measuredZs, np.polyval(poly, measuredZs), linestyle='dashed', label=(f'LS Fit: {poly[0]:.4f} measured + {poly[1]:.4f}'))
    plt.xlabel('Measured Z [m]')
    plt.ylabel('Fit Z [m]')
    plt.legend()
        
    
    
    
    