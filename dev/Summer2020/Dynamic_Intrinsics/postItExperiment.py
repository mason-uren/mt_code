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
from fitVaryingModel import error_for_dataset, altThinLensPose, obtainMLSfit, evaluatedModel
import pandas as pd

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

def searchForFocus(filename):
    with open(filename, 'r') as file:
        data = file.read()
        substring = 'Best focus position is'
        location = data.find(substring)
        croppedStr = data[location+len(substring):]
        # Split at spaces and find first number
        for word in croppedStr.split(): # Split at spaces
            # Delete any commas    
            word = word.replace(',', "")
            try:
                focusPosition = int(word)
                return focusPosition
            except ValueError:
                continue
    file.close()
    
# Assumes that have a higher-level folder and then subfolders for each AF round
# the subfolder name should end with the round number, so when pass in subfolderPrefixes
# to get the full subfolder name simply append the round number at end
# Similarly relFocusPrefix should give relative prefix to file where focus
# is written, so when append round number get the file
def loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds):  
    totalRounds = [trainingRounds, testRounds]
    
    intrinsicsList = [[],[]]
    distortionList = [[],[]]
    focusList = [[],[]]
    roundsList = [[],[]]
    
    for i in range(2):
        selectedRounds = totalRounds[i]
        for AFround in selectedRounds:
            foldername = path + '/' + subfolderPrefixes + str(AFround)
            
            # Skip over AF rounds where did not collect any images
            try:
                distCoeffs = np.load(foldername + '/' + relDistName)
                cameraMatrix = np.load(foldername + '/' + relIntrinsicsName)
                
                roundsList[i].append(AFround)
                
                intrinsicsList[i].append(cameraMatrix)
                distortionList[i].append(distCoeffs)
                
                focus = searchForFocus(foldername + '/' + relFocusPrefix + str(AFround) + '.txt')
                focusList[i].append(focus)
            except FileNotFoundError:
                print('Could not find intrinsics and distortion arrays, skipping round ' + str(AFround))
    return intrinsicsList, distortionList, focusList, roundsList
    
# Input: polynomials for each coefficient, focus for each image, image list, flag whether to also use thin lens
def singleBoardResults(polyModels, correspondingFocus, image_list, expectedDistances,
                       legend=[], defaultIntrinsics=None, defaultDistortion=None,
                       useThinLens=False, useDefault=False, saveResults=False):
    global pixel2world, f
    
    Tlist = np.zeros((len(image_list), len(polyModels) + useThinLens + useDefault, 3))
    Flist = np.zeros((len(image_list), len(polyModels) + useThinLens + useDefault))

    # Given the image list identify the chorners once
    for i, image in enumerate(image_list):
        parameters = cv2.aruco.DetectorParameters_create()
        
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            
            for j, polyModel in enumerate(polyModels):
                intrinsics, dist = evaluatedModel(polyModel, correspondingFocus[i])
                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
                
                Tlist[i, j, :] = np.squeeze(tvec)
                Flist[i, j] = intrinsics[0,0]
            
            if not polyModels:
                j = -1
                
            if useThinLens and defaultIntrinsics is not None:
                rvec, tvec, qHatList = altThinLensPose(pixel2world, f, defaultIntrinsics, defaultDistortion, chorners, chids, board)    
                 
                j=j+1
                
                Tlist[i, j, :] = np.squeeze(tvec)
                Flist[i, j] = qHatList[-1] / pixel2world
                
            if useDefault and defaultIntrinsics is not None:
                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, defaultIntrinsics, defaultDistortion, None, None, useExtrinsicGuess=False)
                
                j = j+1
                # Also use median now
                Tlist[i, j, :] = np.squeeze(tvec)
                Flist[i, j] = defaultIntrinsics[0,0]
                
    # Should be two dimensional where each row is an image and each column is a model
    distances = np.linalg.norm(Tlist, axis=2)
    #distances -= distances[0,:] # Subtract the first image distance off
    
    increments = range(len(image_list))
    
    # Compute the best fit line, slope indicates how much the estimate expects
    # to change going from one post-it to the next
    deltaPlist = []
    polyList = []
    for col in range(distances.shape[1]):
        poly = np.polyfit(increments, distances[:, col], 1)
        deltaPlist.append(poly[0])
        polyList.append(poly)
    
    plt.figure()
    plt.title('Comparing Estimates of Known Z Translation')
    for col in range(distances.shape[1]):
        # Subtract off the a1-estimated intercept
        plt.scatter(increments, distances[:, col] - polyList[col][1], label=legend[col])
    plt.plot(increments, expectedDistances, label='Expected Shift')
    plt.xlabel('Number of Increments')
    plt.ylabel('Estimated Shift in Board wrt Camera Distance')
    plt.xticks(increments)
    plt.legend()
    
    plt.figure()
    plt.title('Comparing Estimates of Known Z Translation')
    for col in range(distances.shape[1]):
        # Subtract off the a1-estimated intercept
        plt.scatter(increments, expectedDistances - distances[:, col] + polyList[col][1], label=legend[col])
    plt.xlabel('Number of Increments')
    plt.ylabel('Expected - Estimated Shift in Board wrt Camera Distance')
    plt.xticks(increments)
    plt.legend()
        
    if saveResults:
        # Save to excel sheet
        data = np.hstack([np.expand_dims(correspondingFocus, axis=1),
                          np.expand_dims(expectedDistances, axis=1), distances])
        
        columnNames = ['Focus Position', 'Expected Distance'] + ['Abs Dist ' + elem for elem in legend]
        indexNames = ['# Increments = ' + str(increment) for increment in increments]
        
        result = pd.DataFrame(data, index=indexNames, columns=columnNames)
                    
        result.to_excel('C:/Users/aofeldman/Desktop/incrementResults.xlsx')
    
    return deltaPlist, polyList, Tlist, Flist
    
    # Not a good visualization tool, too dependent on point used to make things
    # relative (origin p)
    
    # plt.figure()
    # plt.title('Comparing Estimates of Known Z Translation')
    # for col in range(distances.shape[1]):
    #     plt.scatter(increments, distances[:, col], label=legend[col])
    # plt.plot(increments, expectedDistances, label='Expected Shift')
    # plt.xlabel('Number of Increments')
    # plt.ylabel('Estimated Shift in Board wrt Camera Distance')
    # plt.xticks(increments)
    # plt.legend()
    
    
    # plt.figure()
    # plt.title('Comparing Estimates of Known Z Translation')
    # for col in range(distances.shape[1]):
    #     plt.scatter(increments, expectedDistances - distances[:, col], label=legend[col])
    # plt.xlabel('Number of Increments')
    # plt.ylabel('Expected - Estimated Shift in Board wrt Camera Distance')
    # plt.xticks(increments)
    # plt.legend()
    
    # for col in range(distances.shape[1]):
    #     plt.figure()
    #     plt.title('Measured versus Estimated Z: ' + legend[col])
    #     plt.scatter(expectedDistances, distances[:, col])
    #     plt.xlabel('Measured Distance Change')
    #     plt.ylabel('Estimated Distance Change')
    #     poly = np.polyfit(expectedDistances, distances[:, col], 1)
    #     plt.plot(expectedDistances, np.polyval(poly, expectedDistances), linestyle='dashed', label=(f'LS Fit: {poly[0]:.4f} measured + {poly[1]:.4f}'))
    #     plt.plot()
    #     plt.legend()
        
    # return Tlist

if __name__ == '__main__':

    plt.close('all')
    plt.rcParams["scatter.marker"] = 'x'
    
    ##########################
    # Load Images from Path  #
    ##########################
        
    # Need to separate the imperx and ximea
    cameraName = 'ximea' # or imperx
    
    sensor_size = np.array([27.6, 36.4]) * 1e-3 # * 1.0372608996079156
    
    imageShape = (6004, 7920)
    
    # f = 200 * 1e-3 # / 1.0372608996079156 # (200 * 1e-3 / 1.0372608996079156) / 0.9959986338573571
    # f = 0.18541564393493154
    # f = 0.1945562538003889
    f = 0.18897
    # f = 0.1889
    # f = 0.19310233926478237
    # f = 0.17977996
    # f = 0.183
    
    pixel2world = sensor_size[0] / imageShape[0]
    
    #path = 'C:/Users/aofeldman/Desktop/testCollectionRefined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection7-21Refined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined'
    path = 'C:/Users/aofeldman/Desktop/testCollection9-11'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 12
    
    trainingRounds = set(range(numRounds+1))
    testRounds = []
        
    # First, load the fixed models using loadFixedModels
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
    
    fPoint = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
        
    # Try fitting to different portions of the data
    # inlierSets = [np.array([1, 2, 4, 0], dtype=np.int64), 
    #               np.array([1, 2, 3, 0], dtype=np.int64), 
    #               np.array(list(range(6, len(fPoint))))]
    inlierSets = [np.array(list(range(numRounds)))]
    
    correspondingFocus = []
    for AFround in roundList[0]:    
        correspondingFocus.append(searchForFocus(path + '/' + subfolderPrefixes + str(AFround) + '/' + relFocusPrefix + str(AFround) + '.txt'))
    
    # degDicts = [{'f':deg1, 'u0':deg2, 'v0':deg2, 'k1':deg3, 'k2':deg3, 'k3':deg3} for deg1 in range(1,2) for deg2 in range(0,1) for deg3 in range(1,2)]

    degDicts = []
    
    # Since using lambda d: 1, turns into static LS so any focus (e.g. 100) will do
    polyModels = []
    for inliers in inlierSets:  
        trainingIntrinsics = [(intrinsicsList[0] + intrinsicsList[1])[i] for i in inliers]
        trainingDistortion = [(distortionList[0] + distortionList[1])[i] for i in inliers]
        for degDict in degDicts:
            polyModels.append(obtainMLSfit([correspondingFocus[i] for i in inliers],
                                           trainingIntrinsics, trainingDistortion,
                                           [100], degDict, True, False, False, lambda d: 1))
        
    #readPath = 'C:/Users/aofeldman/Desktop/Experiments8-25/increments'
    #readPath = 'C:/Users/aofeldman/Desktop/knownZ'
    readPath = 'C:/Users/aofeldman/Desktop/knownZexpanded/rightForward'
    
    filenames = glob.glob(readPath + '/*.jpg') + \
                glob.glob(readPath + '/*.tif')

    # Reorder based on initial numbering
    image_list = []
    for i in range(1, len(filenames)+1):
        image_list.append(cv2.imread(readPath + '/ximea'+str(i) + '.tif', 0))
    
    # Slightly blur all images
    image_list = [cv2.blur(img, (3,3)) for img in image_list]
    
    #expectedDistances = [-1 * 3 * 2.54 / 100 * i for i in range(len(image_list))]
    #imageFocusList = [457, 442, 425, 405, 391, 369, 350, 331, 309, 286, 264]

    expectedDistances = [3 * 2.54 / 100 * i for i in range(len(image_list))]
    imageFocusList = [searchForFocus(readPath + '/AF' + str(i) + '.txt') for i in range(1, len(image_list)+1)]
    
    # Can reverse everything
    # image_list = image_list[::-1]
    # imageFocusList = imageFocusList[::-1]
    # expectedDistances = -1 * np.array(expectedDistances)
    
    temp = np.argsort(focusList[0])
    medianInd = temp[len(temp) // 2]
    medIntrinsics = intrinsicsList[0][medianInd]
    medDist = distortionList[0][medianInd]
    medFocus = focusList[0][medianInd]
    
    #prefixes = ['441,542 excluded', '493,542 excluded', 'Only 9-11']
    #legend = [prefix + str(degDict) for prefix in prefixes for degDict in degDicts] + ['Thin Lens'] + ['Median']
    
    # legend = [str(degDict) for degDict in degDicts] + ['Thin Lens'] + ['Median']
    legend = ['Thin Lens', 'Median']
    
    deltaPlist, polyList, Tlist, Flist = \
        singleBoardResults(polyModels, imageFocusList, image_list, 
                           expectedDistances, legend, medIntrinsics, 
                           medDist, True, True, False)
    
    
    # Try bootstrapping to see how much fluctuates
    if False:
        # Should be two dimensional where each row is an image and each column is a model
        distances = np.linalg.norm(Tlist, axis=2)
        #distances -= distances[0,:] # Subtract the first image distance off
        
        increments = range(len(image_list))
        
        subsets = []
        numSamples = 100
        subsetSize = 5
        samplesDrawn = 0
        
        # Assume the probability of an exact repeat is small
        # Randomly draw samples of given size from the image_list
        while samplesDrawn < numSamples:
            print('Drawing sample ' + str(samplesDrawn) + ' of ' + str(numSamples))
            subset = np.random.choice(len(image_list), subsetSize, False) # sample without replacement
            subsets.append(subset)
            samplesDrawn += 1
        
        deltaParr = np.zeros((numSamples, len(legend)))
        for i, subset in enumerate(subsets):
            # Compute the best fit line, slope indicates how much the estimate expects
            # to change going from one post-it to the next
            for col in range(distances.shape[1]):
                poly = np.polyfit(np.array(increments)[subset], distances[subset, col], 1)
                deltaParr[i, col] = poly[0]
    
    # Gives the computed M values for each image predicted by each model
    # Each row is for a given image and across row is M for different models
    Marr = np.divide(Tlist[:,:,2], pixel2world * Flist)
    
    plt.figure()
    for col in range(Marr.shape[1]):
        plt.scatter(imageFocusList, Marr[:,col], label=legend[col])
        poly = np.polyfit(imageFocusList, Marr[:,col], 1)
        print('Best Fit Line: ' + (f'{poly[0]:.4f} focus + {poly[1]:.4f}'))
    plt.xlabel('Focus Position')
    plt.ylabel('Computed z/q = M')
    plt.title('M for Different Models Across Focus Positions')
    plt.legend()
    
    plt.figure()
    for col in range(Marr.shape[1]):
        plt.scatter(expectedDistances, Marr[:,col], label=legend[col])
        poly = np.polyfit(expectedDistances, Marr[:,col], 1)
        plt.plot(expectedDistances, np.polyval(poly, expectedDistances), linestyle='dashed',
                 label=(f'LS Fit: {poly[0]:.4f} dist + {poly[1]:.4f}'))
    plt.xlabel('Z Shift')
    plt.ylabel('Computed z/q = M')
    plt.title('M for Different Models Across Change in Z Distance')
    plt.legend()