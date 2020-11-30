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
import os
import matplotlib.pyplot as plt
import pdb
import scipy.optimize
import math
import time
import scipy.stats as stats
from dynamicIntrinsicsHelpers import *

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

def fitAlpha(Mlist, measuredZs):
    A = np.expand_dims(np.array(Mlist) + 1, axis=1)
    alpha = np.squeeze(np.linalg.pinv(A) @ np.expand_dims(np.array(measuredZs),axis=1))
    outZ = np.squeeze(alpha * A)
    return alpha, outZ

if __name__ == '__main__':

    plt.close('all')
    
    ##########################
    # Load Images from Path  #
    ##########################
        
    # Need to separate the imperx and ximea
    cameraName = 'ximea' # or imperx
    
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    
    f = 200 * 1e-3
    
    imageShape = (6004, 7920)
    
    pixel2world = sensor_size[0] / imageShape[0]
    
    path = 'C:/Users/aofeldman/Desktop/testCollectionCombined'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 18
    
    trainingRounds = set(range(numRounds+1))
    testRounds = []
    
    fListOverall = []
    
    # First, load the fixed models using loadFixedModels
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
    
    fPoint = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
    
    temp = np.argsort(focusList[0])
    medianInd = temp[len(temp) // 2]
    medIntrinsics = intrinsicsList[0][medianInd]
    medDist = distortionList[0][medianInd]
    medFocus = focusList[0][medianInd]
    
    datasetErrors = []
    q0List = []
    image0 = []

    for i, AFround in enumerate(roundList[0]):
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
        
        rotations = []
        translations = []
        fList = []
        for i, image in enumerate(image_list):
        
            # Aaron testing out different parameter options
            parameters = cv2.aruco.DetectorParameters_create()
            
            markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
            
            # if we dont find enough points skip
            if (ids is not None and len(ids) > 8):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                #retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
                
                rvec, tvec, qHatList = altThinLensPose(pixel2world, f, medIntrinsics, medDist, chorners, chids, board)
                
                revIntrinsics = np.copy(medIntrinsics)
                revIntrinsics[0,0] = qHatList[-1] * image.shape[0] / sensor_size[0]
                revIntrinsics[1,1] = revIntrinsics[0,0]
                
                fList.append(revIntrinsics[0,0])
                
                # Compute the reprojection error
                R, _ = cv2.Rodrigues(rvec)
                mean_error = charuco_reprojection_error(board,chorners,chids,R,tvec,revIntrinsics,medDist)
                image_errors.append(mean_error)
                rotations.append(np.rad2deg(rotationMatrixToEulerAngles(R)))
                translations.append(tvec)
                if i == 0:
                    image0.append(tvec)
                    q0List.append(qHatList[-1])
            else:
                print("Skipping: Failed to find charuco board in image " + str(i))
        fListOverall.append([np.mean(fList), np.min(fList), np.max(fList)])
        
        rotations = np.squeeze(rotations)
        translations = np.squeeze(translations)
        
        datasetErrors.append(np.mean(image_errors))
        
    image0 = np.squeeze(image0)

    # Should have shape numRounds X 3
    fListOverall = np.squeeze(fListOverall)
    
    pointZs = []
    for rnd in range(1,7):
        tvecs = np.load('C:/Users/aofeldman/Desktop/testCollection9-11/AFround' + str(rnd) + '/ximea_tvecs.npy')
        pointZs.append(tvecs[0][2,0])
    
    # See how well can approximate z against measured M with z = alpha * M + alpha = alpha * (M + 1)
    Mpoint = np.divide(pointZs, sensor_size[0] / image.shape[0] * np.array(fPoint[:6]))
    Mlens = np.divide(image0[:6,2], q0List[:6])     
    measuredZs = np.array([181, 125, 135.5, 147.5, 159, 169.5]) * 2.54 / 100
    
    # What if assumed a static Q such that Z = alpha M
    staticAlpha, staticZ = fitAlpha(Mpoint - 1, measuredZs)
    
    plt.figure()
    plt.scatter(Mpoint, measuredZs, label='Measured Z')
    plt.scatter(Mpoint, staticZ, label=(f'Z = {staticAlpha} M'))
    plt.title('Using a static q')
    plt.xlabel('Point Estimated M')
    plt.ylabel('Z [m]')
    plt.legend()
    
    plt.figure()
    plt.scatter(Mpoint, measuredZs, label='Measured Z')
    poly = np.polyfit(Mpoint, measuredZs, 1)
    plt.scatter(Mpoint, np.polyval(poly, Mpoint), label=(f'Z = {poly[0]} M + {poly[1]}'))
    plt.legend()
    plt.xlabel('Point Estimated M')
    plt.ylabel('Z [m]')
    plt.title('Linear fit for Z against Point Estimated M')
    
    plt.figure()
    plt.scatter(Mlens, measuredZs, label='Measured Z')
    poly = np.polyfit(Mlens, measuredZs, 1)
    plt.scatter(Mlens, np.polyval(poly, Mlens), label=(f'Z = {poly[0]} M + {poly[1]}'))
    plt.legend()
    plt.xlabel('Lens Estimated M')
    plt.ylabel('Z [m]')
    plt.title('Linear fit for Z against Lens Estimated M')
    
    # What would it look like if added a slight positive offset, say 0.056 m first?
    # measuredZs += 0.056
    
    pointAlpha, pointOutZ = fitAlpha(Mpoint, measuredZs)
    lensAlpha, lensOutZ = fitAlpha(Mlens, measuredZs)
    
    plt.figure()
    plt.scatter(Mpoint, measuredZs, label='Measured Z')
    plt.scatter(Mpoint, pointOutZ, label=(f'Point: alpha = {pointAlpha}'))
    plt.scatter(Mlens, lensOutZ, label=(f'Thin Lens: alpha = {lensAlpha}'))
    plt.xlabel('Estimated M')
    plt.ylabel('Z [m]')
    plt.title('Estimating Z as function of M')
    plt.legend()

    plt.figure()
    plt.scatter(Mpoint, measuredZs - pointOutZ, label='Measured - Point Alpha Fit')
    plt.scatter(Mlens, measuredZs - lensOutZ, label='Measured - Lens Alpha Fit')
    plt.xlabel('Estimated M')
    plt.ylabel('Difference in Z [m]')
    plt.title('Estimating Z as function of M: Error')
    
    plt.figure()
    plt.scatter(measuredZs, image0[:6, 2])
    poly = np.polyfit(measuredZs, image0[:6, 2], 1)
    plt.plot(measuredZs, np.polyval(poly, measuredZs), label=(f'LS Fit: {poly[0]:.4f} q + {poly[1]:.4f}'), linestyle='dashed')
    plt.xlabel('Measured Z [m]')
    plt.ylabel('Estimated Z [m]')
    plt.legend()
    plt.title('Thin Lens Estimated Z against Measured')
    
    plt.figure()
    plt.title('Comparing f: Point against Thin Lens')
    plt.scatter(focusList[0], fPoint, label='Point')
    #xInliers = np.hstack([focusList[0][:3], focusList[0][-2]])
    #yInliers = np.hstack([fPoint[:3], fPoint[-2]])
    xInliers = focusList[0][6:]
    yInliers = fPoint[6:]
    #pointPoly = np.polyfit(focusList[0][:4], fPoint[:4], 1)
    pointPoly = np.polyfit(xInliers, yInliers, 1)
    plt.plot(focusList[0], np.polyval(pointPoly, focusList[0]), label=(f'Inlier Point LS Fit: {pointPoly[0]:.4f} q + {pointPoly[1]:.4f}'), linestyle='dashed')
    plt.errorbar(focusList[0], fListOverall[:,0], yerr=np.array([fListOverall[:,0] - fListOverall[:,1], fListOverall[:,2] - fListOverall[:,0]]), color = 'b', fmt='o', label='Thin Lens')
    polyLens = np.polyfit(focusList[0], fListOverall[:,0], 1)
    plt.plot(focusList[0], np.polyval(polyLens, focusList[0]), label=(f'Lens Point LS Fit: {polyLens[0]:.4f} q + {polyLens[1]:.4f}'), linestyle='dashed')
    plt.xlabel('Focus Position')
    plt.ylabel('Predicted f')
    plt.legend()
        
    
    estimatedZs = []
    for rnd in range(7, 19):
        tvecs = np.load('C:/Users/aofeldman/Desktop/testCollection9-11/AFround' + str(rnd) + '/ximea_tvecs.npy')
        estimatedZs.append(tvecs[0][2,0])
    
    Mlist0 = np.divide(estimatedZs, sensor_size[0] / image.shape[0] * np.array(fPoint[6:]))
    
    MtheoryList = np.divide(image0[6:,2], fListOverall[6:, 0] * sensor_size[0] / image.shape[0])
    
    plt.figure()
    plt.scatter(focusList[0][6:], Mlist0, label='Point Estimated')
    plt.scatter(focusList[0][6:], MtheoryList, label='Thin Lens')
    plt.scatter(focusList[0][6:], 1 / f * (np.squeeze(estimatedZs) - f), label='(Point z - f)/f')
    plt.xlabel('Focus Position')
    plt.ylabel('Estimated M')
    plt.legend()
    
    plt.figure()
    plt.scatter(focusList[0][6:], Mlist0 - MtheoryList, label='M point - M lens')
    plt.xlabel('Focus Position')
    plt.ylabel('Difference in Estimated M')
    plt.legend()
    
    plt.figure()
    Fhat = 1 / pixel2world * (np.divide(f, Mlist0) + f)
    plt.scatter(focusList[0][6:], fPoint[6:], label='Point Estimated')
    plt.scatter(focusList[0][6:], Fhat, label='Theory using Point M')
    plt.scatter(focusList[0][6:], fListOverall[6:, 0], label='Thin Lens')
    plt.xlabel('Focus Position')
    plt.ylabel('Estimated f parameter')
    plt.legend()
    
    