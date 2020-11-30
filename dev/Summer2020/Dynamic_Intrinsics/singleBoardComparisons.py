# Aaron added for personal import setup
from setupImports2 import setup_path
setup_path()

# from hardware.ximea.Driver.client.ximea_client import *
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
import scipy.optimize
import itertools
from dynamicIntrinsicsHelpers import *
import cv2

# Take a single image from each round (focus position), do estimation using point estimate, median estimate, and new approach and plot the results with
# x axis as focus position of the round and y axis as the resulting estimate (x, y, z from board to camera)
# image_list, trainingFocus, trainingIntrinsics, trainingDist should all have the same length

# Can also plot the resulting difference in p between median and new approach as a function of the difference in M between the M of current image
# and the M of the image corresponding to the median position of trainingFocus
def singleBoardResults(aruco_dict, board, image_list, trainingFocus, trainingIntrinsics, trainingDist, sensor_size, f, qList):
    rotations = [[],[],[]]
    translations = [[],[],[]]
    Ms = []
    fEffs = []
    
    # Get the intrinsics corresponding to the median focus
    temp = np.argsort(trainingFocus)
    medianInd = temp[len(temp) // 2]
    medIntrinsics = trainingIntrinsics[medianInd]
    medDist = trainingDist[medianInd]
    medFocus = trainingFocus[medianInd]
    
    for i, image in enumerate(image_list):
        parameters = cv2.aruco.DetectorParameters_create()
    
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            
            # Point estimate
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, trainingIntrinsics[i], trainingDist[i], None, None, useExtrinsicGuess=False)
            
            # Convert to roll, pitch, yaw
            R, _ = cv2.Rodrigues(rvec)
            rotations[0].append(np.rad2deg(rotationMatrixToEulerAngles(R)))
            
            translations[0].append(np.squeeze(tvec))
            
            # Median estimate
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, medIntrinsics, medDist, None, None, useExtrinsicGuess=False)
            
            # Convert to roll, pitch, yaw
            R, _ = cv2.Rodrigues(rvec)
            rotations[1].append(np.rad2deg(rotationMatrixToEulerAngles(R)))
            
            translations[1].append(np.squeeze(tvec))
            
            rvec, tvec, fEff, Mlist = estimatePoseFancy(qList, pixel2world, f, medIntrinsics, medDist, chorners, chids, markerCorners, ids, board, True)
            M = Mlist[0]
            Ms.append(M)
            
            # Could also do similar thin lens approach
            # rvec, tvec, qHatList = altThinLensPose(pixel2world, f, medIntrinsics, medDist, chorners, chids, board)
            # fEff = qHatList[-1] / pixel2world
            # Ms.append(tvec[2] / qHatList[-1])
            
            # Convert to roll, pitch, yaw
            R, _ = cv2.Rodrigues(rvec)
            rotations[2].append(np.rad2deg(rotationMatrixToEulerAngles(R)))
                        
            translations[2].append(np.squeeze(tvec))
            fEffs.append(fEff)
            
    for i in range(3):
        rotations[i] = np.array(rotations[i])
        translations[i] = np.array(translations[i])
        
    names = ['Point Estimation', 'Median Estimation', 'New Approach']
    
    plt.figure()
    plt.title('X [m]')
    for i in range(3):
        plt.scatter(trainingFocus, translations[i][:, 0], label=names[i])
    plt.legend()
    
    plt.figure()
    plt.title('Y [m]')
    for i in range(3):
        plt.scatter(trainingFocus, translations[i][:, 1], label=names[i])
    plt.legend()
    
    plt.figure()
    plt.title('Z [m]')
    for i in range(3):
        plt.scatter(trainingFocus, translations[i][:, 2], label=names[i])
    plt.legend()
    
    plt.figure()
    plt.title('roll [deg]')
    for i in range(3):
        plt.scatter(trainingFocus, rotations[i][:, 0], label=names[i])
    plt.legend()
    
    plt.figure()
    plt.title('pitch [deg]')
    for i in range(3):
        plt.scatter(trainingFocus, rotations[i][:, 1], label=names[i])
    plt.legend()
    
    plt.figure()
    plt.title('yaw [deg]')
    for i in range(3):
        plt.scatter(trainingFocus, rotations[i][:, 2], label=names[i])
    plt.legend()
    
    plt.figure()
    plt.title('M')
    plt.scatter(trainingFocus, Ms)
    
    q0 = medIntrinsics[0, 0] * pixel2world
    M0 = f / (q0 - f)
    deltaP = f * (np.divide(Ms, M0) - 1)
    measuredDelta = translations[1][:,2] - translations[2][:,2]
        
    plt.figure()
    plt.title('Theoretical vs. Measured Difference in Z [m] between Median and New Approach')
    plt.scatter(trainingFocus, measuredDelta, label='Measured Difference')
    plt.scatter(trainingFocus, deltaP, label='Theoretical Difference')
    plt.legend()
    plt.xlabel('Focus Motor Position')
    plt.ylabel('Difference in Z [m]')
    
    #pdb.set_trace()
    
    Ms = np.squeeze(Ms)
    postMs = np.divide(np.squeeze(translations[2][:,2]), pixel2world * np.squeeze(fEffs))
    plt.figure()
    plt.title('Prior vs. Post M Results')
    plt.scatter(trainingFocus, Ms, label='Prior')
    plt.scatter(trainingFocus, postMs, label='Post')
    plt.legend()
    
    plt.figure()
    plt.title('Prior vs. Post M Ratio')
    ratios = np.divide(postMs, Ms)
    plt.scatter(trainingFocus, ratios)
    plt.plot(trainingFocus, ratios)
    
    return fEffs, Ms, medIntrinsics, medDist, medFocus



if __name__ == '__main__':
    
    plt.close('all')
    
    ##########################
    # Charuco Board Consts   #
    ##########################
    
    squareLength = boards['TV_3']['squareLength']
    markerLength = boards['TV_3']['markerLength']
    # In the images, seems like board is actually rotated so that have 12 across 
    # by 8 down 
    charucoX = boards['TV_3']['charucoX']
    charucoY = boards['TV_3']['charucoY']
    
    ##########################
    # Graphng Setup          #
    ##########################
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    
    board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
    
    # Aaron added to verify that board is indeed what are seeing in the images
    dispBoard = board.draw((500, 500))
    #cv2.imshow('board', dispBoard)
    #cv2.imshow('board', cv2.rotate(dispBoard, cv2.ROTATE_90_CLOCKWISE))
    #cv2.waitKey(0)
    
    ##########################
    # Load Images from Path  #
    ##########################    
    # Need to separate the imperx and ximea
    cameraName = 'ximea'
    
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    
    imageShape = (6004, 7920)
    
    pixel2world = sensor_size[0] / imageShape[0]
    
    f = 200 * 1e-3 # / 1.0372608996079156
    
    # qList = [(f / M + f) / pixel2world for M in np.linspace(21, 23, 10)]
    
    qList = [52694.16036842526,
              61456.85608821037,
              54343.803434476904,
              53145.99221073076,
              54964.93554539974,
              54034.62926088611,
              52199.65514502811,
              51950.55275867424,
              51924.6188247505,
              47720.619314281364,
              51029.44962706456,
              51527.21750798464,
              49636.36154672016,
              51776.67397436449,
              49782.97617514236,
              45548.56557376607,
              45359.50698046031, 
              46352.66722520469,
              45462.36305855741]
 
    path = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 19
    trainingRounds = set(range(numRounds+1))
    testRounds = []
    
    intrinsicsList, distortionList, focusList, roundsList = loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
    
    trainingFocus = focusList[0]
    trainingIntrinsics = intrinsicsList[0]
    trainingDist = distortionList[0]
    
    image_list = []
    # Now, need to load in one image (ximea10) from each folder in roundsList
    for rnd in roundsList[0]:
        image_list.append(cv2.imread(path + '/' + subfolderPrefixes + str(rnd) + '/' + cameraName + str(10) + '.tif', 0))
    
    #readPath = '\\\\netapp1-svm3/issl/issl_archive/Archives/2019/Boeing-DR/ClosedLoopMetrology/2020/Weekly-Progress/Aaron/Experiments8-25/pos'
    #for i in range(1, 4):
    #    image_list.append(cv2.imread(readPath + str(i) + '.tif', 0))
    
    image_list = [cv2.blur(img, (3,3)) for img in image_list]
    
    fEffs, Ms, medIntrinsics, medDist, medFocus = singleBoardResults(aruco_dict, board, image_list, trainingFocus, trainingIntrinsics, trainingDist, sensor_size, f, qList)
        
                

        