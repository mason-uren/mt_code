# Last Modified: 7/18/2020 Changes made by Aaron
# Aaron added for personal import setup
from setupImports import setup_path
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
import scipy.stats as stats
from dynamicIntrinsicsHelpers import *

plt.close('all')

##########################
# Charuco Board Consts   #
##########################

# Aaron Changed to TV_4 was TV_3, put back to TV_3 since used big monitor again
squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

##########################
# Graphng Setup          #
##########################
start_time = time.time()

# Aaron commented out because don't actually care about running camera
#cam = ximea_recieve_camera()
#cam.start()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

# Aaron added to verify that board is indeed what are seeing in the images
dispBoard = board.draw((1000, 1000))
#cv2.imshow('board', dispBoard)
#cv2.waitKey(0)


# Should plot histograms of values after each round of elimination
def generateSamples(board, aruco_dict, image_list, subsetSize, numSamples, cameraName='ximea'):
    assert (subsetSize <= len(image_list))
    
    samplesDrawn = 0
    subsets = []
    
    # Track f, u0, v0, k1, k2, p1, p2, k3
    compiledParams = np.zeros((numSamples, 8)) 
    # Assume the probability of an exact repeat is small
    # Randomly draw samples of given size from the image_list
    while samplesDrawn < numSamples:
        print('Drawing sample ' + str(samplesDrawn) + ' of ' + str(numSamples))
        chosenInd = np.random.choice(len(image_list), subsetSize, False) # sample without replacement
        subset = [image_list[i] for i in chosenInd]
        subsets.append(subset)
        samplesDrawn += 1
        
    print('Finished drawing samples')
    
    for sample, subset in enumerate(subsets):
        print('Calibrating set ' + str(sample) + ' of ' + str(len(subsets)))
        _, cameraMatrix, distCoeffs, _, _ = calibrateCamera(board, aruco_dict, subset, cameraName)
        compiledParams[sample, 0] = cameraMatrix[0,0]
        compiledParams[sample, 1] = cameraMatrix[0,2]
        compiledParams[sample, 2] = cameraMatrix[1,2]
        compiledParams[sample, 3] = distCoeffs[0, 0]
        compiledParams[sample, 4] = distCoeffs[0, 1]
        compiledParams[sample, 5] = distCoeffs[0, 2]
        compiledParams[sample, 6] = distCoeffs[0, 3]
        compiledParams[sample, 7] = distCoeffs[0, 4]
    
    return compiledParams

def refinedEstimate(compiledParams, graph=True, numSigmas = 2):
    
    counter = 0
    oldLength = 0
    while len(compiledParams) != oldLength:
        numSamples = len(compiledParams)

        print('Counter: ' + str(counter))
        print('Number of Rows (start): ' + str(len(compiledParams)))
        
        # Perform MLE estimate of normal distribution using in-sample mean and std
        mu = np.mean(compiledParams, axis=0)
        print('Mu: ', mu)
        sigma = np.std(compiledParams, axis=0)
        print('Sigma: ', sigma)
        
        # Plot a histogram and the normal approximation for each parameter at the current round
        # BEFORE outlier removal
        if graph:
            names = ['f', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
            for col in range(compiledParams.shape[1]):
                plt.figure()
                plt.title('Counter = ' + str(counter) + ' Histogram for ' + names[col])
                plt.hist(compiledParams[:,col], 'auto', density=True)
                x = np.linspace(mu[col] - numSigmas*sigma[col], mu[col] + numSigmas*sigma[col], 100)
                plt.plot(x, stats.norm.pdf(x, mu[col], sigma[col]))        
        expandedMu = np.tile(mu, (numSamples, 1))
        expandedSigma = np.tile(sigma, (numSamples, 1))
        mask = (np.abs(compiledParams - expandedMu) > numSigmas * expandedSigma)
        # Only keep those rows which had no outlier parameter
        oldLength = len(compiledParams)
        compiledParams = compiledParams[np.where(np.sum(mask, axis=1) == 0)]
        print('New length of compiledParams: ' + str(len(compiledParams)))
        counter += 1
        
    f, u0, v0, k1, k2, p1, p2, k3 = np.mean(compiledParams, axis=0)
    
    revIntrinsics = np.zeros((3,3))
    revIntrinsics[0,0] = f
    revIntrinsics[1,1] = f
    revIntrinsics[0,2] = u0
    revIntrinsics[1,2] = v0
    revDist = np.zeros((1,5))
    revDist[0,0] = k1
    revDist[0,1] = k2
    revDist[0,2] = p1
    revDist[0,3] = p2
    revDist[0,4] = k3
    
    return compiledParams, revIntrinsics, revDist

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
import glob

# Need to separate the imperx and ximea
cameraName = 'ximea' # or imperx

numRounds = 6

focusList = []

distanceList = [181, 125, 135.5, 147.5, 159, 169.5]

intrinsicsList = []
distortionList = []
paramsList = []
rounds = []

# Read in the default intrinsics and distortion list (no bootstrap approach)

path = 'C:/Users/aofeldman/Desktop/testCollection8-31'
subfolderPrefixes = 'AFround'
relDistName = cameraName + '_distCoeffs.npy'
relIntrinsicsName = cameraName + '_intrinsics.npy'
relFocusPrefix = 'AF'
trainingRounds = set(range(numRounds+1))
testRounds = []

prevIntrinsics, prevDistortion, _, _ = loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
prevIntrinsics = prevIntrinsics[0]
prevDistortion = prevDistortion[0]

graph = True

subsetSize = 18
numSamples = 100

for AFround in range(numRounds + 1):
    #foldername = 'C:/Users/aofeldman/Desktop/testCollection7-21Refined/AFround' + str(AFround)
    foldername = 'C:/Users/aofeldman/Desktop/testCollection8-31/AFround' + str(AFround)
    # Load in all images for the given focus setting
    filenames = glob.glob(foldername + '/*.tif')
    
    imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
    filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
    imageNumbers = np.sort(imageNumbers)
    
    print('imageNumbers\n', imageNumbers)
    
    # Skip over AF rounds where did not collect any images
    if filenames:
        # Read in and mildly blur images
        image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
    
        print('AFround = ' + str(AFround))
        print('Size of Images Collected = ' + str(image_list[0].shape))
        print("Number of Images in Folder: " + str(len(image_list)))
    
        compiledParams = generateSamples(board, aruco_dict, image_list, subsetSize, numSamples)
        revParams, cameraMatrix, distCoeffs = refinedEstimate(compiledParams)
    
        paramsList.append(revParams)
        intrinsicsList.append(cameraMatrix)
        distortionList.append(distCoeffs)
        rounds.append(AFround)
        focusList.append(searchForFocus(foldername + '/' + 'AF' + str(AFround) + '.txt'))
        
        
plt.figure()
plt.title('Focus position as function of distance (board to camera centers)')
plt.xlabel('Distance [in]')
plt.ylabel('Focus Position')
plt.scatter(distanceList, focusList, label='Measured')
poly = np.polyfit(distanceList, focusList, 1)
plt.plot(distanceList, np.polyval(poly, distanceList), label=f'Linear LS Fit: {poly[0]:.4f} q + {poly[1]:.4f}')
plt.legend()     

names = ['focus position', 'distance (in)']

for i, xAxis in enumerate([focusList, distanceList]):
    plt.figure()
    plt.title('Bootstrapped f as function of ' + names[i])
    plt.xlabel(names[i])
    plt.ylabel('f obtained (pixel units)')
    plt.scatter(xAxis, [intrinsicsList[i][0,0] for i in range(len(intrinsicsList))], label='Bootstrapped')
    plt.scatter(xAxis, [prevIntrinsics[i][0,0] for i in range(len(intrinsicsList))], label='Original')
    plt.legend()
    
    plt.figure()
    plt.title('Bootstrapped u0 as function of ' + names[i])
    plt.xlabel(names[i])
    plt.ylabel('u0 obtained (pixel units)')
    plt.scatter(xAxis, [intrinsicsList[i][0,2] for i in range(len(intrinsicsList))], label='Bootstrapped')
    plt.scatter(xAxis, [prevIntrinsics[i][0,2] for i in range(len(intrinsicsList))], label='Original')
    plt.legend()
    
    plt.figure()
    plt.title('Bootstrapped v0 as function of ' + names[i])
    plt.xlabel(names[i])
    plt.ylabel('v0 obtained (pixel units)')
    plt.scatter(xAxis, [intrinsicsList[i][1,2] for i in range(len(intrinsicsList))], label='Bootstrapped')
    plt.scatter(xAxis, [prevIntrinsics[i][1,2] for i in range(len(intrinsicsList))], label='Original')
    plt.legend()
    
    plt.figure()
    plt.title('Bootstrapped k1 as function of ' + names[i])
    plt.xlabel(names[i])
    plt.ylabel('k1 obtained (pixel units)')
    plt.scatter(xAxis, [distortionList[i][0,0] for i in range(len(distortionList))], label='Bootstrapped')
    plt.scatter(xAxis, [prevDistortion[i][0,0] for i in range(len(distortionList))], label='Original')
    plt.legend()
    
    plt.figure()
    plt.title('Bootstrapped k2 as function of ' + names[i])
    plt.xlabel(names[i])
    plt.ylabel('k2 obtained (pixel units)')
    plt.scatter(xAxis, [distortionList[i][0,1] for i in range(len(distortionList))], label='Bootstrapped')
    plt.scatter(xAxis, [prevDistortion[i][0,1] for i in range(len(distortionList))], label='Original')
    plt.legend()
    
    plt.figure()
    plt.title('Bootstrapped p1 as function of ' + names[i])
    plt.xlabel(names[i])
    plt.ylabel('p1 obtained (pixel units)')
    plt.scatter(xAxis, [distortionList[i][0,2] for i in range(len(distortionList))], label='Bootstrapped')
    plt.scatter(xAxis, [prevDistortion[i][0,2] for i in range(len(distortionList))], label='Original')
    plt.legend()
    
    plt.figure()
    plt.title('Bootstrapped p2 as function of ' + names[i])
    plt.xlabel(names[i])
    plt.ylabel('p2 obtained (pixel units)')
    plt.scatter(xAxis, [distortionList[i][0,3] for i in range(len(distortionList))], label='Bootstrapped')
    plt.scatter(xAxis, [prevDistortion[i][0,3] for i in range(len(distortionList))], label='Original')
    plt.legend()
    
    plt.figure()
    plt.title('Bootstrapped k3 as function of ' + names[i])
    plt.xlabel(names[i])
    plt.ylabel('k3 obtained (pixel units)')
    plt.scatter(xAxis, [distortionList[i][0,4] for i in range(len(distortionList))], label='Bootstrapped')
    plt.scatter(xAxis, [prevDistortion[i][0,4] for i in range(len(distortionList))], label='Original')
    plt.legend()
