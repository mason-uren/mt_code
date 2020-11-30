# Last Modified: 7/23/2020 Changes made by Aaron
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
import colorsys
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from dynamicIntrinsicsHelpers import *
import pandas as pd
import estimateBeam

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
def plotFit(trainingFocus, trainingIntrinsics, trainingDistortion, polynomials):
    
    # Fit to the training set
    if len(polynomials) == 8:
        f = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
    elif len(polynomials) == 9:
        fx = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
        fy = [camMatrix[1,1] for camMatrix in trainingIntrinsics]
    else:
        raise IndexError("Please provide a polynomial for each of the 8 or 9 parameters")
        
    u0 = [camMatrix[0,2] for camMatrix in trainingIntrinsics]
    v0 = [camMatrix[1,2] for camMatrix in trainingIntrinsics]
    k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
    k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
    p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
    p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
    k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]

    if len(polynomials) == 8:
        parameterNames = ['f', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [f, u0, v0, k1, k2, p1, p2, k3]
    elif len(polynomials) == 9:
        parameterNames = ['fx', 'fy', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [fx, fy, u0, v0, k1, k2, p1, p2, k3]
   
    for i, parameter in enumerate(parameterSet):
        p = polynomials[i]
        deg = len(p) - 1
        plt.figure()
        plt.title('Degree ' + str(deg) + ' fit for ' + str(parameterNames[i]))
        plt.scatter(trainingFocus, parameter)
        plt.scatter(trainingFocus, [np.polyval(p, trainingFocus[i]) for i in range(len(trainingFocus))])
        plt.legend(['Point Estimation', 'Fit'])
        plt.xlabel('Focus Position')
        plt.ylabel(parameterNames[i])    

def plotPointEstimates(trainingFocus, trainingIntrinsics, trainingDistortion, aspect1 = True, color='b'):
    # Fit to the training set
    if aspect1:
        f = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
    else:
        fx = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
        fy = [camMatrix[1,1] for camMatrix in trainingIntrinsics]

    u0 = [camMatrix[0,2] for camMatrix in trainingIntrinsics]
    v0 = [camMatrix[1,2] for camMatrix in trainingIntrinsics]
    k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
    k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
    p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
    p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
    k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]

    if aspect1:
        parameterNames = ['f', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [f, u0, v0, k1, k2, p1, p2, k3]
    else:
        parameterNames = ['fx', 'fy', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [fx, fy, u0, v0, k1, k2, p1, p2, k3]
    
    figureHandles = []
    scatterHandles = []
    for i, parameter in enumerate(parameterSet):
        handle = plt.figure()
        plt.title('Point estimates for ' + str(parameterNames[i]))
        scatter = plt.scatter(trainingFocus, parameter, color=color)
        plt.xlabel('Focus Position')
        plt.ylabel(parameterNames[i])
        figureHandles.append(handle)
        scatterHandles.append(scatter)
        
    # Also plot v0 against u0
    # Use color to indicate focus position
    lowest, highest = np.argmin(trainingFocus), np.argmax(trainingFocus)
    scale = trainingFocus[lowest], trainingFocus[highest]
    colors = [(focus - scale[0]) / (scale[1] - scale[0]) for focus in trainingFocus]
    colors = np.clip(colors, 0, 1)
    # Actually rescale it so only goes up to 0.75 and 0 is red so invert
    # Set l and s by default
    # Each row should be color (r,g,b) for row'th point
    colors = [colorsys.hls_to_rgb(0.75 * (1 - color), 0.5, 1) for color in colors]
    
    order = np.argsort(trainingFocus)
    legend_elements = [Patch(facecolor=colors[i], label=trainingFocus[i]) for i in order]
    
    plt.figure()
    plt.title('Point Estimated Optical Center')
    plt.xlabel('u0')
    plt.ylabel('v0')
    plt.scatter(u0, v0, c=colors)
    plt.legend(handles=legend_elements)
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.title('Point Estimated Optical Center')
    ax.scatter(u0, v0, trainingFocus)
    ax.set_xlabel('u0')
    ax.set_ylabel('v0')
    ax.set_zlabel('Focus Position')
    
    # Return the figure and scatter handles
    return figureHandles, scatterHandles

# Evalute the varying model at a given focus position
def evaluatedModel(polynomials, focus):
    # fx and fy independent
    if len(polynomials) == 9:
        intrinsics = np.array([[np.polyval(polynomials[0], focus), 0, np.polyval(polynomials[2], focus)],
                              [0, np.polyval(polynomials[1], focus), np.polyval(polynomials[3], focus)],
                              [0, 0, 1]])
        distortion = np.expand_dims([np.polyval(polynomials[i], focus) for i in range(4, 9)], 0)
    # fx = fy assumed
    elif len(polynomials) == 8:
        intrinsics = np.array([[np.polyval(polynomials[0], focus), 0, np.polyval(polynomials[1], focus)],
                              [0, np.polyval(polynomials[0], focus), np.polyval(polynomials[2], focus)],
                              [0, 0, 1]])
        distortion = np.expand_dims([np.polyval(polynomials[i], focus) for i in range(3, 8)], 0)
    else:
        raise IndexError("Please provide a polynomial for each of the 8 or 9 parameters")
    return intrinsics, distortion

# Does polynomial least-square fit for one parameter on data considering degrees from lowDeg
# to highDeg inclusive. deg represents the power on highest monomial term so
# have deg+1 total terms in polynomial fit
def crossValidatePolyfit(x, y, lowDeg, highDeg):
    cvErrors = []
    for deg in range(lowDeg, highDeg + 1):
        # Leave out a different point each time
        cvErr = 0
        for leftOut in range(len(x)):
            xTrain = np.delete(x, leftOut)
            xTest = x[leftOut]
            yTrain = np.delete(y, leftOut)
            yTest = y[leftOut]    
            p = np.polyfit(xTrain, yTrain, deg)
            # Compute the squared residual error on left out point
            cvErr += (np.polyval(p, xTest) - yTest)**2 / len(x)
        cvErrors.append(cvErr)
        
    # Return the list of cross-validation errors and the degree which gave
    # the smallest cross-validation error
    return cvErrors, lowDeg + np.argmin(cvErrors)        

# Compute the moving LS approximation for a given parameter at the current
# focus
# weightingFunc should be a smooth scalar in-out function of the distance
# of the test point to the training point (difference in focus)
def movingLS(trainingFocus, trainingParam, deg, weightingFunc, currFocus, regMat = None, x0 = None):
    # First compute the weight matrix
    distances = np.array(trainingFocus) - currFocus
    weighting = np.zeros(np.shape(distances))
    for i, distance in enumerate(distances):
        weighting[i] = weightingFunc(distance)
    W = np.diag(weighting)
    #print('Current focus: ' + str(currFocus))
    #print('Focus and Corresponding Weights:\n', trainingFocus, weighting)
    
    # Now, compute the polynomial basis matrix
    P = np.zeros((len(trainingFocus), deg+1))
    for row in range(P.shape[0]):
        for col in range(P.shape[1]):
            # As go across row have decreasing powers to align with
            # polyfit and polyval
            P[row, col] = trainingFocus[row] ** (P.shape[1] - 1 - col)
    
    z = np.array(trainingParam)
    
    if regMat is not None:
        if x0 is None:
            x0 = np.zeros(len(regMat))
            
        # Generalized Tikhonov Regularized LS
        return np.linalg.inv(P.T @ W @ P + regMat) @ (P.T @ W @ z + regMat @ x0)    
    else:
        # Weighted LS coefficient vector
        return np.linalg.inv(P.T @ W @ P) @ P.T @ W @ z
    
def obtainInitialFit(trainingFocus, trainingIntrinsics, trainingDistortion, degrees = {}, aspect1 = True, graph=True, verbose=True):
    # Fit to the training set
    if aspect1:
        f = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
    else:
        fx = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
        fy = [camMatrix[1,1] for camMatrix in trainingIntrinsics]
    u0 = [camMatrix[0,2] for camMatrix in trainingIntrinsics]
    v0 = [camMatrix[1,2] for camMatrix in trainingIntrinsics]
    k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
    k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
    p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
    p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
    k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]

    if aspect1:
        parameterNames = ['f', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [f, u0, v0, k1, k2, p1, p2, k3]
    else:    
        parameterNames = ['fx', 'fy', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [fx, fy, u0, v0, k1, k2, p1, p2, k3]
   
    polynomials = []
    
    for i, parameter in enumerate(parameterSet):
        try:
            deg = degrees[parameterNames[i]]
        except KeyError:
            # By default use quadratic for fx, fy (or f), constant for u0, v0, k1, k2, p1, p2, k3 linear
            if parameterNames[i] in ['f', 'fx', 'fy']:
                deg = 2
            elif parameterNames[i] == 'k3':
                deg = 1
            else: 
                deg = 0
        p = np.polyfit(trainingFocus, parameter, deg)
        if verbose:
            print('Parameter: ' + str(parameterNames[i]))
            print('Initial Fit: ' + str(p))
            
        if graph:
            plt.figure()
            plt.title('Degree ' + str(deg) + ' fit for ' + str(parameterNames[i]))
            plt.scatter(trainingFocus, parameter)
            plt.scatter(trainingFocus, [np.polyval(p, trainingFocus[i]) for i in range(len(trainingFocus))])
            plt.legend(['Point Estimation', 'Fit'])
            plt.xlabel('Focus Position')
            plt.ylabel(parameterNames[i])
            
        polynomials.append(p)
    return polynomials
        
def obtainMLSfit(trainingFocus, trainingIntrinsics, trainingDistortion, currFocus, degrees = {}, aspect1 = True, graph=True, verbose=True, weightingFunc = lambda d: 1 / d**2, regMatDict = {}, x0Dict = {}):
    # Fit to the training set
    if aspect1:
        f = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
    else:
        fx = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
        fy = [camMatrix[1,1] for camMatrix in trainingIntrinsics]
    u0 = [camMatrix[0,2] for camMatrix in trainingIntrinsics]
    v0 = [camMatrix[1,2] for camMatrix in trainingIntrinsics]
    k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
    k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
    p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
    p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
    k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]

    if aspect1:
        parameterNames = ['f', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [f, u0, v0, k1, k2, p1, p2, k3]
    else:    
        parameterNames = ['fx', 'fy', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [fx, fy, u0, v0, k1, k2, p1, p2, k3]
   
    polynomials = []
    
    for i, parameter in enumerate(parameterSet):
        try:
            deg = degrees[parameterNames[i]]
        except KeyError:
            # By default use quadratic for fx, fy (or f), constant for u0, v0, k1, k2, p1, p2, k3 linear
            if parameterNames[i] in ['f', 'fx', 'fy']:
                deg = 2
            elif parameterNames[i] == 'k3':
                deg = 1
            else: 
                deg = 0
        
        try:
            regMat = regMatDict[parameterNames[i]]
            # If is scalar, create array by lambda * I
            if np.shape(regMat) == ():
                regMat = regMat * np.identity(deg+1)    
        except KeyError:
            regMat = None
        
        try:
            x0 = x0Dict[parameterNames[i]]
        except KeyError:
            x0 = None
        
        p = movingLS(trainingFocus, parameter, deg, weightingFunc, currFocus, regMat, x0)
        
        # TODO: Temporary change to see if can improve results by using better
        # center
        #if parameterNames[i] == 'u0':
        #    p = np.array([3.99915487e+03])
        #elif parameterNames[i] == 'v0':
        #    p = np.array([3.07215819e+03])
        
        if verbose:
            print('Parameter: ' + str(parameterNames[i]))
            print('Initial Fit: ' + str(p))
            
        if graph:
            plt.figure()
            plt.title('Degree ' + str(deg) + ' fit for ' + str(parameterNames[i]))
            plt.scatter(trainingFocus, parameter)
            plt.scatter(trainingFocus, [np.polyval(p, trainingFocus[i]) for i in range(len(trainingFocus))])
            plt.scatter([currFocus], [np.polyval(p, currFocus)])
            plt.legend(['Point Estimation', 'Fit', 'Current Focus Prediction'])
            plt.xlabel('Focus Position')
            plt.ylabel(parameterNames[i])
            
        polynomials.append(p)
    return polynomials

# Interesting weighting functions to try:
# lambda d: np.exp(-1e-3 * d**2)
# lambda d: np.exp(-1e-4 * d**2) gets smoother as make  1e-k smaller
# lamda d: np.sqrt(1 + 1e-3 * d**2) pretty closely adheres to static form
# lambda d: np.sqrt(1 + 1e-2 * d**2)
# lambda d: 1 / (1 + 1e-3 * d**2) gets smoother as make 1e-k smaller
# lambda d: 1 / np.sqrt(1 + 1e-3 * d**2) like above but smoother

def plotMLSfit(trainingFocus, trainingIntrinsics, trainingDistortion, \
               currFocusList = np.linspace(300, 500, 200), degrees = {}, \
                   aspect1 = True, verbose=True, legend = [], weightingFuncList = [], regPairedDicts = []):    
    if not weightingFuncList:
        weightingFuncList = [lambda d: 1]
    
    if aspect1:
        f = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
    else:
        fx = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
        fy = [camMatrix[1,1] for camMatrix in trainingIntrinsics]
    u0 = [camMatrix[0,2] for camMatrix in trainingIntrinsics]
    v0 = [camMatrix[1,2] for camMatrix in trainingIntrinsics]
    k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
    k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
    p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
    p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
    k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]

    if aspect1:
        parameterNames = ['f', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [f, u0, v0, k1, k2, p1, p2, k3]
    else:    
        parameterNames = ['fx', 'fy', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [fx, fy, u0, v0, k1, k2, p1, p2, k3]
    
    results = []
    for weightingFunc in weightingFuncList:
        if regPairedDicts:
            for (regMat, x0) in regPairedDicts:
                evaluationPoints = [[] for i in range(len(parameterNames))]
                
                for currFocus in currFocusList:
                    polynomials = obtainMLSfit(trainingFocus, trainingIntrinsics, trainingDistortion, currFocus, degrees, aspect1, False, False, weightingFunc, regMat, x0)
                    for i, p in enumerate(polynomials):
                        evaluationPoints[i].append(np.polyval(p, currFocus))
                
                results.append(evaluationPoints)
        else:
            evaluationPoints = [[] for i in range(len(parameterNames))]
                
            for currFocus in currFocusList:
                polynomials = obtainMLSfit(trainingFocus, trainingIntrinsics, trainingDistortion, currFocus, degrees, aspect1, False, False, weightingFunc)
                for i, p in enumerate(polynomials):
                    evaluationPoints[i].append(np.polyval(p, currFocus))
            
            results.append(evaluationPoints)

    for i, parameterName in enumerate(parameterNames):
        plt.figure()
        plt.title('Moving fit with degree ' + str(len(polynomials[i]) - 1) + ' for ' + parameterName)
        plt.scatter(trainingFocus, parameterSet[i], 30)
        for _, evaluationPoints in enumerate(results):
            plt.scatter(currFocusList, evaluationPoints[i], 3)
        plt.legend(legend)
    return results

# Assumes that have called loadFixedModels before and that testRounds
# does not include any empty folders

# Modified so can pass in any number of polynomial models with varying degree
# Modified so also returns the median estimate errors
def testResults(path, subfolderPrefixes, testRounds, testFocus, testIntrinsics, testDistortion,
                medIntrinsics = None, medDist = None,  lensBaseIndex=None, *polyModels, legend=[], useLens=True, graph=True, cameraName='ximea'):
    overallErrList = []
    rotationsList = []
    translationsList = []
    # Now test on the left out data
    for i, AFround in enumerate(testRounds):
        meanErrList = []

        foldername = path + '/' + subfolderPrefixes + str(AFround)
    
        # Load in all images for the given focus setting
        filenames = glob.glob(foldername + '/' + '*.tif')
        
        imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
        filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
        imageNumbers = np.sort(imageNumbers)
        
        # Read in and mildly blur images
        image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
    
        print('Testing on round: ' + str(AFround))
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
        
        # Point estimate
        fixedDist = testDistortion[i]
        fixedIntrinsics = testIntrinsics[i]
        focus = testFocus[i]
        fixedImgErr, fixedMeanErr, pointRs, pointTs = error_for_dataset(chornersList, chidsList, fixedIntrinsics, fixedDist)
        meanErrList.append(fixedMeanErr)
        rotationsList.append(pointRs)
        translationsList.append(pointTs)
        
        if graph:
            plt.figure()
            plt.title('Testing on AFround'+ str(AFround) + ' comparison of image errors')
            plt.plot(imageNumbers, fixedImgErr)
            plt.xlabel('Image Number')
            plt.ylabel('Re-projection error')
        
        # Median estimate
        if medIntrinsics is not None and medDist is not None:
            medImgErr, medMeanErr, medRs, medTs = error_for_dataset(chornersList, chidsList, medIntrinsics, medDist)
            if graph:
                plt.plot(imageNumbers, medImgErr)
                
            meanErrList.append(medMeanErr)
            rotationsList.append(medRs)
            translationsList.append(medTs)
            
        if useLens:
            # Try combining thin lens with a model, see if improves results
            if lensBaseIndex is not None:
                intrinsics, dist = evaluatedModel(polyModels[lensBaseIndex], focus)
            else:
                intrinsics = medIntrinsics
                dist = medDist
            lensImgErr, lensMeanErr, lensRs, lensTs = error_for_dataset(chornersList, chidsList, intrinsics, dist, True)
            if graph:
                plt.plot(imageNumbers, lensImgErr)
            
            meanErrList.append(lensMeanErr)
            rotationsList.append(lensRs)
            translationsList.append(lensTs)
        
        for polyModel in polyModels:
            intrinsics, dist = evaluatedModel(polyModel, focus)
            imgErr, meanErr, polyRs, polyTs = error_for_dataset(chornersList, chidsList, intrinsics, dist)
            meanErrList.append(meanErr)
            if graph:
                plt.plot(imageNumbers, imgErr)
            rotationsList.append(polyRs)
            translationsList.append(polyTs)
            
        if graph and legend:
            plt.legend(legend)
        
        overallErrList.append(meanErrList)

    return overallErrList, rotationsList, translationsList

# Note: can always do a non-moving fit by using lambda d: 1
def crossValidateMoving(path, subfolderPrefixes, relDistName, relIntrinsicsName, \
                        relFocusPrefix, numRounds, degDicts, legend = [], weightingFuncList = [], regPairedDicts = []):
    if not weightingFuncList:
        #weightingFuncList = [lambda d: np.exp(-1e-4 * d**2)]
        weightingFuncList = [lambda d: 1]

    
    totalRounds = set(range(numRounds+1))
    # Take the set difference
    trainingRounds = list(totalRounds)
    testRounds = []
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
    
    # Get the intrinsics corresponding to the median focus
    temp = np.argsort(focusList[0])
    medianInd = temp[len(temp) // 2]
    medIntrinsics = intrinsicsList[0][medianInd]
    medDist = distortionList[0][medianInd]
    medFocus = focusList[0][medianInd]

    leftOut = []
    # List of lists where each inner list contains the mean errors for the models when leaving out a given point
    meanErrList = []
    #avgDistErr = []
    deltaTlist = []
    deltaRlist = []
    for leaveOut in range(numRounds+1):
        print('Leaving out: ' + str(leaveOut))
        totalRounds = set(range(numRounds+1))
        # Take the set difference
        trainingRounds = list(totalRounds - {leaveOut})
        #trainingRounds = list({1,2,3,5} - {leaveOut})
        # trainingRounds = list({1,2,3,5}) # For now, train even on leaveOut
        testRounds = [leaveOut]
        intrinsicsList, distortionList, focusList, roundList = \
            loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
        # If chose to leave out an empty round, then move on
        if not roundList[1]:
            continue
        else:
            leftOut.append(leaveOut)
            # Collection of the different polynomial fits as dictated by degDicts
            polyModels = []
            lensBaseIndex = None
            for ind, degDict in enumerate(degDicts):
                for weightingFunc in weightingFuncList:
                    if regPairedDicts:
                        for (regMatDict, x0Dict) in regPairedDicts:
                            polyModels.append(obtainMLSfit(focusList[0], intrinsicsList[0], distortionList[0], focusList[1], degDict, True, False, False, weightingFunc, regMatDict, x0Dict))
                    else:
                        if degDict == {'f':1, 'u0':0, 'v0':0, 'k1':1, 'k2':1,'k3':1}:
                            lensBaseIndex = ind
                            print('Set lensBaseIndex for ' + str(degDict))
                        polyModels.append(obtainMLSfit(focusList[0], intrinsicsList[0], distortionList[0], focusList[1], degDict, True, False, False, weightingFunc))
                        
            print('Started testing with: ' + str(len(polyModels)) + ' models')
            overallErrList, rotationsList, translationsList = testResults(path, subfolderPrefixes, 
                roundList[1], focusList[1], intrinsicsList[1], distortionList[1], 
                medIntrinsics, medDist, lensBaseIndex, *polyModels, legend=legend, useLens=True, graph=False, 
                cameraName='ximea')
            meanErrors = overallErrList[0]
            print('meanErrors were: ' + str(meanErrors))
            meanErrList.append(meanErrors)
            
            # Compute average distance between point and med/poly estimate
            #distErrors = [np.mean(np.linalg.norm(translationsList[i] - translationsList[0], axis=1)) for i in range(1,len(translationsList))]
            
            #avgDistErr.append(distErrors)
            #pdb.set_trace()
            deltaT = [np.mean(np.abs(translationsList[i] - translationsList[0]), axis=0) for i in range(1, len(translationsList))]
            deltaR = [np.mean(np.abs(rotationsList[i] - rotationsList[0]), axis=0) for i in range(1, len(rotationsList))]
            
            deltaTlist.append(deltaT)
            deltaRlist.append(deltaR)
            
    return leftOut, meanErrList, deltaRlist, deltaTlist, medFocus

def compareLBeam(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, relLbeam, numRounds, degDicts, aspect1, weightingFuncList = [], regPairedDicts = []):
    if not weightingFuncList:
        #weightingFuncList = [lambda d: np.exp(-1e-4 * d**2)]
        weightingFuncList = [lambda d: 1]
        
    leftOut = []
    overallDistList = []
    overallRotList = []
    overallXList = []
    overallYList = []
    overallZList = []
    fitIntrinsics = []
    fitDist = []
    
    totalRounds = set(range(numRounds+1))
    # Take the set difference
    trainingRounds = list(totalRounds)
    testRounds = []
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
    
    # Get the intrinsics corresponding to the median focus
    temp = np.argsort(focusList[0])
    medianInd = temp[len(temp) // 2]
    medIntrinsics = intrinsicsList[0][medianInd]
    medDist = distortionList[0][medianInd]
    medFocus = focusList[0][medianInd]
    
    for leaveOut in range(numRounds+1):
        # List of lists where each inner list contains the predictions for a given model on each L-beam image
        distList = []
        rotList = []
        xList = []
        yList = []
        zList = []
        print('Leaving out: ' + str(leaveOut))
        totalRounds = set(range(numRounds+1))
        # Take the set difference
        trainingRounds = list(totalRounds - {leaveOut})
        testRounds = [leaveOut]
        intrinsicsList, distortionList, focusList, roundList = \
            loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
        # If chose to leave out an empty round, then move on
        if not roundList[1]:
            continue
        else:
            # Load in and slightly blur images
            filenames = glob.glob(path + '/' + subfolderPrefixes + str(leaveOut) + '/' + relLbeam + '/*.tif')
                        
            if filenames:
                leftOut.append(leaveOut)

                filenames.sort()
                
                # Aaron added in reading in as grayscale
                image_list = [cv2.imread(img, 0) for img in filenames]
                # No need since now not on monitor
                #image_list = [cv2.blur(img, (3,3)) for img in image_list]
                                    
                print("Number of Images in Folder: " + str(len(image_list)))
            else:
                continue
            
            # Get the focus at the test round
            focus = searchForFocus(path + '/' + subfolderPrefixes + str(leaveOut) + '/' + relFocusPrefix + str(leaveOut) + '.txt')
            
            # Collection of the different polynomial fits as dictated by degDicts
            polyModels = []
            #for degDict in degDicts:
            #    polyModels.append(obtainInitialFit(focusList[0], intrinsicsList[0], distortionList[0], degDict, True, False, False))
            
            for degDict in degDicts:
                for weightingFunc in weightingFuncList:
                    if regPairedDicts:
                        for (regMatDict, x0Dict) in regPairedDicts:
                            polyModels.append(obtainMLSfit(focusList[0], intrinsicsList[0], distortionList[0], focusList[1], degDict, aspect1, False, False, weightingFunc, regMatDict, x0Dict))
                    else:
                        polyModels.append(obtainMLSfit(focusList[0], intrinsicsList[0], distortionList[0], focusList[1], degDict, aspect1, False, False, weightingFunc))

            # Add the point estimate and then the median estimate
            predictedIntrinsics = intrinsicsList[1]
            predictedDist = distortionList[1]
            
            predictedIntrinsics.append(medIntrinsics)
            predictedDist.append(medDist)
            
            for polynomial in polyModels:    
                intrinsics, dist = evaluatedModel(polynomial, focus)
                predictedIntrinsics.append(intrinsics)
                predictedDist.append(dist)
            
            ##########################
            # Charuco Board Consts   #
            ##########################
            
            # Aaron changed this to NewFiducial for the latest boards
            squareLength = boards['NewFiducial']['squareLength']
            markerLength = boards['NewFiducial']['markerLength']
            # In the images, seems like board is actually rotated so that have 12 across 
            # by 8 down 
            charucoX = boards['NewFiducial']['charucoX']
            charucoY = boards['NewFiducial']['charucoY']
            
            ##########################
            # Graphng Setup          #
            ##########################
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
            
            board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
            
            for i in range(len(predictedIntrinsics)):
                intrinsics = predictedIntrinsics[i]
                dist = predictedDist[i]
                distances, rotations, translations = estimateBeam.estimateBeam(image_list, intrinsics, dist, aruco_dict, board, True, True, range(4,8))
                
                distList.append(distances)
                rotList.append(rotations)
                xList.append(np.array(translations)[:,0])
                yList.append(np.array(translations)[:,1])
                zList.append(np.array(translations)[:,2])
            
            overallDistList.append(distList)
            overallRotList.append(rotList)
            overallXList.append(xList)
            overallYList.append(yList)
            overallZList.append(zList)
            fitIntrinsics.append(predictedIntrinsics)
            fitDist.append(predictedDist)
            
    return leftOut, overallDistList, overallRotList, overallXList, overallYList, overallZList, medFocus, fitIntrinsics, fitDist


def newApproachLBeam(path, subfolderPrefixes, relDistName, relIntrinsicsName, 
                     relFocusPrefix, relLbeam, numRounds, sensor_size, f, qList, defaultIntrinsics = None, dist = None):
    
    leftOut = []
    overallDistList = []
    overallRotList = []
    overallXList = []
    overallYList = []
    overallZList = []
    fitIntrinsics = []
    fitDist = []
    
    totalRounds = set(range(numRounds+1))
    # Take the set difference
    trainingRounds = list(totalRounds)
    testRounds = []
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
    
    # Get the intrinsics corresponding to the median focus
    temp = np.argsort(focusList[0])
    medianInd = temp[len(temp) // 2]
    medIntrinsics = intrinsicsList[0][medianInd]
    medDist = distortionList[0][medianInd]
    medFocus = focusList[0][medianInd]
    
    if defaultIntrinsics is None:
        defaultIntrinsics = np.copy(medIntrinsics)
    
    if dist is None:
        dist = np.copy(medDist)
    
    for leaveOut in range(numRounds+1):
        # List of lists where each inner list contains the predictions for a given model on each L-beam image
        distList = []
        rotList = []
        xList = []
        yList = []
        zList = []
        print('Leaving out: ' + str(leaveOut))
        totalRounds = set(range(numRounds+1))
        # Take the set difference
        trainingRounds = list(totalRounds - {leaveOut})
        testRounds = [leaveOut]
        intrinsicsList, distortionList, focusList, roundList = \
            loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
        # If chose to leave out an empty round, then move on
        if not roundList[1]:
            continue
        else:
            # Load in and slightly blur images
            filenames = glob.glob(path + '/' + subfolderPrefixes + str(leaveOut) + '/' + relLbeam + '/*.tif')
                        
            if filenames:
                leftOut.append(leaveOut)

                filenames.sort()
                
                # Aaron added in reading in as grayscale
                image_list = [cv2.imread(img, 0) for img in filenames]
                # No need since now not on monitor
                #image_list = [cv2.blur(img, (3,3)) for img in image_list]
                                    
                print("Number of Images in Folder: " + str(len(image_list)))
            else:
                continue
            
            # Get the focus at the test round
            focus = searchForFocus(path + '/' + subfolderPrefixes + str(leaveOut) + '/' + relFocusPrefix + str(leaveOut) + '.txt')
            
            # Add the point estimate and then the median estimate
            predictedIntrinsics = intrinsicsList[1]
            predictedDist = distortionList[1]
            
            predictedIntrinsics.append(medIntrinsics)
            predictedDist.append(medDist)
            
            ##########################
            # Charuco Board Consts   #
            ##########################
            
            # Aaron changed this to NewFiducial for the latest boards
            squareLength = boards['NewFiducial']['squareLength']
            markerLength = boards['NewFiducial']['markerLength']
            # In the images, seems like board is actually rotated so that have 12 across 
            # by 8 down 
            charucoX = boards['NewFiducial']['charucoX']
            charucoY = boards['NewFiducial']['charucoY']
            
            ##########################
            # Graphng Setup          #
            ##########################
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
            
            board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
            
            for i in range(len(predictedIntrinsics) + 1):
                if i != len(predictedIntrinsics):
                    intrinsics = predictedIntrinsics[i]
                    dist = predictedDist[i]
                    distances, rotations, translations = estimateBeam.estimateBeam(image_list, intrinsics, dist, aruco_dict, board, True, True, range(4,8))
                else:
                    # Now, perform estimate using the new approach
                    distances, rotations, translations = thinLensEstimateBeam(image_list, defaultIntrinsics, dist, aruco_dict, board, pixel2world, f)
        
                distList.append(distances)
                rotList.append(rotations)
                xList.append(np.array(translations)[:,0])
                yList.append(np.array(translations)[:,1])
                zList.append(np.array(translations)[:,2])
            
            overallDistList.append(distList)
            overallRotList.append(rotList)
            overallXList.append(xList)
            overallYList.append(yList)
            overallZList.append(zList)
            fitIntrinsics.append(predictedIntrinsics)
            fitDist.append(predictedDist)
            
    return leftOut, overallDistList, overallRotList, overallXList, overallYList, overallZList, medFocus, fitIntrinsics, fitDist


# Assuming arguments in radians
# Interval is of form [start, end) and should be of length 2pi
def wrapAngle(theta, start, end, radians=True):
    if radians:
        assert (np.abs((end - start) - 2 * np.pi) < 1e-6)
        
        while theta < start:
            theta += 2 * np.pi 
            
        while theta >= end:
            theta -= 2 * np.pi
    else:
        assert (np.abs((end - start) - 2 * 180) < 1e-6)
        
        while theta < start:
            theta += 2 * 180 
            
        while theta >= end:
            theta -= 2 * 180
            
    return theta

def predictKfromQ(q1, q2, k1, k2, q, f):
    alpha = (q2 - q) / (q2 - q1) * (q1 - f) / (q - f)
    
    return alpha * k1 + (1 - alpha) * k2

# Assume that k is linear with respect to magnification so that 
# given (M1, k1) and (M2, k2) can predict k overall
def predictKfromM(M1, M2, k1, k2, M):
    slope = (k1 - k2) / (M1 - M2)
    intercept = (M1 * k2 - M2 * k1) / (M1 - M2)
    return slope * M + intercept

if __name__ == '__main__':

    plt.close('all')
    plt.rcParams["scatter.marker"] = 'o'
    
    ##########################
    # Load Images from Path  #
    ##########################
        
    # Need to separate the imperx and ximea
    cameraName = 'ximea' # or imperx
    
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
    
    # Vertical, Horizontal
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    
    f = 200 * 1e-3 # / 1.0372608996079156
    # f = 0.1889
    # Vertical, Horizontal
    imageShape = (6004, 7920)

    pixel2world = sensor_size[0] / imageShape[0] 
    
    #path = 'C:/Users/aofeldman/Desktop/testCollectionRefined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection7-21Refined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection8-31'
    #path = 'C:/Users/aofeldman/Desktop/testCollectionCombined'
    path = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 19
        
    MOVE_CROSS_VALIDATE = False
    
    PLOT_MOVING = False
    
    PLOT_POINT = False
    
    PREDICT_K = False
    
    NEW_APPROACH = True
    
    L_BEAM = False
        
    if PLOT_POINT:
        trainingRounds = set(range(0, numRounds+1))
        testRounds = []
        
        # First, load the fixed models using loadFixedModels
        intrinsicsList, distortionList, focusList, roundList = \
            loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
        
        # Differentiate data collected on 8-31 and 9-11
        if path.split('/')[-1] == 'testCollectionCombined':
            colors = ['b'] * 6 + ['g'] * (len(intrinsicsList[0]) - 6)
            plotPointEstimates(focusList[0], intrinsicsList[0], distortionList[0], color=colors)
            # Get best fit line for the 9-11 points
            fPoint = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
            polyLatest = np.polyfit(focusList[0][6:], fPoint[6:], 1)
            # Exclude points
            inliers = np.where((np.array(focusList[0][:6]) != 493) * (np.array(focusList[0][:6]) != 542))
            polyEarlier = np.polyfit(np.array(focusList[0])[inliers], np.array(fPoint)[inliers], 1)
        else:
            plotPointEstimates(focusList[0], intrinsicsList[0], distortionList[0])
    
    if PREDICT_K:
        trainingRounds = set(range(0, numRounds+1))
        testRounds = []
        
        # First, load the fixed models using loadFixedModels
        intrinsicsList, distortionList, focusList, roundList = \
            loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
        
        trainingIntrinsics = intrinsicsList[0]
        trainingDistortion = distortionList[0]
        
        F = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
        qList = pixel2world * np.array(F)
        k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
        k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
        p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
        p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
        k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]

        Mlist = []
        
        for i, AFround in enumerate(roundList[0]):
            print('Reading in round ' + str(AFround))
            foldername = path + '/' + subfolderPrefixes + str(AFround)
        
            # Load in all images for the given focus setting
            filenames = glob.glob(foldername + '/' + '*.tif')
            
            imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
            filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
            imageNumbers = np.sort(imageNumbers)
            
            # Read in and mildly blur images
            image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
            
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
            
            pointImgErr, pointMeanErr, pointRs, pointTs = error_for_dataset(chornersList, chidsList, intrinsicsList[0][i], distortionList[0][i])
        
            Mlist.append(np.mean(np.squeeze(pointTs)[:,2]) / qList[i])
                   
        for i, k in enumerate([k1, k2]):
            f = 0.1889
            
            orderedFocus = np.argsort(focusList[0])
            indices = [orderedFocus[1], orderedFocus[8]]
            [qa, qb] = [qList[indices[i]] for i in range(2)]
            # [Ma, Mb] = [np.array(Mlist)[indices[i]] for i in range(2)]
            [ka, kb] = [k[indices[i]] for i in range(2)]
            
            # Now, make predictions for all the q values
            # predKlist = [predictKfromM(Ma, Mb, ka, kb, M) for M in Mlist]
            predKlist = [predictKfromQ(qa, qb, ka, kb, q, f) for q in qList]
    
            plt.figure()
            poly = np.polyfit(Mlist, k, 1)
            plt.scatter(Mlist, np.polyval(poly, Mlist), linestyle='dashed', label=(f'{poly[0]} M + {poly[1]}'))
            plt.scatter(Mlist, k, label='Point Estimated k' +str(i+1))
            plt.xlabel('Point Estimated Magnification M')
            plt.ylabel('k' + str(i+1))
            plt.title('Estimating k' +str(i+1) + ' via Linear Relationship with Magnification')
            plt.legend()
            
            plt.figure()
            plt.scatter(focusList[0], k, label='Point Estimated k' + str(i+1))
            plt.scatter(focusList[0], predKlist, label='Prediction using Point')
            # plt.scatter(focusList[0], theoryK, label='Prediction using Theory')
            plt.xlabel('Focus Position')
            plt.ylabel('k' +str(i+1))
            plt.title('Theoretically Predicted against Point Estimated k' +str(i+1))
            plt.axvline(x = focusList[0][indices[0]], linestyle='dashed', label='Calibration Point 1')
            plt.axvline(x = focusList[0][indices[1]], linestyle='dashed', label='Calibration Point 2')
            plt.legend()
            
            joined = (np.vstack([Mlist, qList, k]).T)[orderedFocus]
            columnNames = ['M', 'q', 'k']
            indexNames = ['Round ' + str(j+1) + ': Focus = ' + str(np.array(focusList[0])[orderedFocus[j]]) for j in range(len(orderedFocus))]
            
            result = pd.DataFrame(joined, index=indexNames, columns=columnNames)
                        
            result.to_excel('C:/Users/aofeldman/Desktop/fittingK' + str(i+1) + '.xlsx')

            
    # TODO: MLS should allow for different weight functions for different parameters
    
    if PLOT_MOVING:
        trainingRounds = set(range(numRounds+1))
        # trainingRounds = {1,2,3,5}
        testRounds = []
        
        # First, load the fixed models using loadFixedModels
        intrinsicsList, distortionList, focusList, roundList = \
            loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    

        weightingFuncList = [lambda d: 1]
        
        aspect1 = True
        regularize = False
        
        # Look into using higher order for distortion, seems like could be promising
        if aspect1:
            degDicts = [{'f':deg1, 'u0':deg2, 'v0':deg2, 'k1':deg3, 'k2':deg3, 'k3':deg3} for deg1 in range(1,2) for deg2 in range(0,2) for deg3 in range(1,2)]
        else:
            degDicts = [{'fx':deg1, 'fy':deg1, 'u0':deg2, 'v0':deg2, 'k1':deg3, 'k2':deg3, 'k3':deg3} for deg1 in range(2,4) for deg2 in range(0,1) for deg3 in range(2,3)]

        if regularize:
            uCenter = np.array([3960])
            vCenter = np.array([3004])
            x0Dict = {'u0':uCenter, 'v0':vCenter}
            regPairedDicts = [ ({'u0':0, 'v0':0}, x0Dict),
                              ({'u0':0.1, 'v0':0.1}, x0Dict), 
                              ({'u0':1, 'v0':1}, x0Dict),
                              ({'u0':10, 'v0':10}, x0Dict),
                              ({'u0':100, 'v0':100}, x0Dict)]
            legend = [str(degDicts[i]) + ' ' + str(regPairedDicts[j]) for i in range(len(degDicts)) for j in range(len(regPairedDicts))]
           
        else:
            regPairedDicts = []
            legend = [str(degDicts[i]) for i in range(len(degDicts))]

        #degDicts = [{'f':deg1, 'u0':deg2, 'v0':deg2} for deg1 in range(0, 3) for deg2 in range(0,2)]
        
        #weightingFuncList = [lambda d: 1, lambda d: np.exp(-1e-4 * d**2), 
        #                     lambda d: np.sqrt(1 + 1e-2 * d**2), lambda d: np.sqrt(1 + 1e-3 * d**2), 
        #                     lambda d: 1 / (1 + 1e-2 * d**2), lambda d: 1 / (1 + 1e-3 * d**2), lambda d: 1 / (1 + 1e-4 * d**2),
        #                     lambda d: 1 /np.sqrt(1 + 1e-3 * d**2)]
        
        #legend = ['Point Estimates', 'Static LS', 'Gaussian 1e-4', 'Multiquad 1e-2', 'Multiquad 1e-3', 'Inv Quad 1e-2', 'Inv Quad 1e-3', 'Inv Quad 1e-4', 'Inv Multiquad 1e-3']

        #weightingFuncList = [lambda d: 1, lambda d: np.exp(-1e-4 * d**2), 
        #                     lambda d: 1 / (1 + 1e-2 * d**2), lambda d: 1 / (1 + 1e-3 * d**2), lambda d: 1 / np.sqrt(1 + 1e-3 * d**2)]
        #legend = ['Point Estimates', 'Static LS', 'Gaussian 1e-4', 'Inverse Quadratic 1e-2','Inverse Quadratic 1e-3', 'Inverse Multiquadratic 1e-3']

        for i in range(len(degDicts)):
            plotMLSfit(focusList[0], intrinsicsList[0], distortionList[0], \
                       np.linspace(300, 600, 300), degDicts[i], True, True, legend, weightingFuncList, regPairedDicts)
         
    if MOVE_CROSS_VALIDATE:
        #degDicts = [{'f':deg1, 'u0':deg2, 'v0':deg2} for deg1 in range(0, 3) for deg2 in range(0,1)]
        
        #weightingFuncList = [lambda d: 1, lambda d: np.exp(-1e-3 * d**2), lambda d: np.exp(-1e-4 * d**2), 
        #                     lambda d: np.sqrt(1 + 1e-2 * d**2), lambda d: np.sqrt(1 + 1e-3 * d**2), 
        #                     lambda d: 1 / (1 + 1e-2 * d**2), lambda d: 1 / (1 + 1e-3 * d**2), lambda d: 1 / (1 + 1e-4 * d**2)]
        #weightingFuncList = [lambda d: 1, lambda d: np.exp(-1e-4 * d**2), lambda d: 1 / d**2]
        
        weightingFuncList = [lambda d: 1]
        
        aspect1 = True
        regularize = False
        
        # Look into using higher order for distortion, seems like could be promising
        if aspect1:
            degDicts = [{'f':deg1, 'u0':deg2, 'v0':deg2, 'k1':deg3, 'k2':deg3, 'k3':deg3} for deg1 in range(1,2) for deg2 in range(0,2) for deg3 in range(1,2)]
        else:
            degDicts = [{'fx':deg1, 'fy':deg1, 'u0':deg2, 'v0':deg2, 'k1':deg3, 'k2':deg3, 'k3':deg3} for deg1 in range(2,4) for deg2 in range(0,1) for deg3 in range(2,3)]

        if regularize:
            uCenter = np.array([3960])
            vCenter = np.array([3004])
            x0Dict = {'u0':uCenter, 'v0':vCenter}
            regPairedDicts = [ ({'u0':0, 'v0':0}, x0Dict),
                              ({'u0':0.1, 'v0':0.1}, x0Dict), 
                              ({'u0':1, 'v0':1}, x0Dict),
                              ({'u0':10, 'v0':10}, x0Dict),
                              ({'u0':100, 'v0':100}, x0Dict)]
            legend = ['Point Estimation', 'Median Estimation', 'Thin Lens'] + \
            [str(degDicts[i]) + ' ' + str(regPairedDicts[j]) for i in range(len(degDicts)) for j in range(len(regPairedDicts))]
           
        else:
            regPairedDicts = []
            legend = ['Point Estimation', 'Median Estimation', 'Thin Lens'] + \
                [str(degDicts[i]) for i in range(len(degDicts))]
        
        #leftOut, meanErrList, avgDistErr, medFocus = \
        leftOut, meanErrList, deltaRlist, deltaTlist, medFocus = \
            crossValidateMoving(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, numRounds, degDicts, legend, weightingFuncList, regPairedDicts)
            
        plt.figure()
        # Each row corresponds to a given leaving out round
        # Each column corresponds to a particular model
        meanErrList = np.array(meanErrList)
              
        correspondingFocus = []
        for AFround in leftOut:    
            correspondingFocus.append(searchForFocus(path + '/' + subfolderPrefixes + str(AFround) + '/' + relFocusPrefix + str(AFround) + '.txt'))
        
        #plt.scatter(leftOut, meanErrList[:, 0])
        plt.scatter(correspondingFocus, meanErrList[:, 0]) # point
        plt.scatter(correspondingFocus, meanErrList[:, 1]) # median
        plt.scatter(correspondingFocus, meanErrList[:, 2]) # thin lens
        for i, degDict in enumerate(degDicts):
            #plt.scatter(leftOut, meanErrList[:, i+1])
            plt.scatter(correspondingFocus, meanErrList[:, i+3])
        plt.legend(legend)
        #plt.xlabel('Left Out Dataset')
        plt.xlabel('Left Out Dataset Focus')
        plt.ylabel('Mean Re-projection Error on Left Out Dataset')
        plt.title('Left Out Re-projection Errors for Different Models')

        print('Average scores across all left out datasets: ' + str(np.mean(meanErrList, 0)))
        bestModelInd = np.argmin(np.mean(meanErrList[:, 1:], 0))
        print('Most successful model was: ' + legend[bestModelInd+1])
        
        # First axis is round, second axis is model, third axis is R/T component
        deltaRlist = np.squeeze(deltaRlist)
        deltaTlist = np.squeeze(deltaTlist)
        
        print('Shape of deltaRlist: ', np.shape(deltaRlist))
        print('Shape of deltaTlist: ', np.shape(deltaTlist))
        
        # measuredZs = np.array([181, 125, 135.5, 147.5, 159, 169.5]) * 2.54 / 100
        
        components = ['Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z']
        for i in range(len(components)):
            plt.figure()
            if i < 3:
                plt.scatter(correspondingFocus, deltaRlist[:, 0, i]) # median
                plt.scatter(correspondingFocus, deltaRlist[:, 1, i]) # thin lens
                for j, degDict in enumerate(degDicts):
                    plt.scatter(correspondingFocus, deltaRlist[:, j+2, i])
                    #plt.scatter(correspondingFocus, avgDistErr[:, i+1])
            else:
                plt.scatter(correspondingFocus, 1e3 * deltaTlist[:, 0, i - 3]) # median
                plt.scatter(correspondingFocus, 1e3 * deltaTlist[:, 1, i - 3]) # thin lens
                for j, degDict in enumerate(degDicts):
                    plt.scatter(correspondingFocus, 1e3 * deltaTlist[:, j+2, i-3])
                    #plt.scatter(correspondingFocus, avgDistErr[:, i+1])
            plt.legend(legend[1:])
            plt.xlabel('Point Estimate Focus Position')
            plt.ylabel('Mean Abs Error:' + str(components[i]) + ' relative to Point ' + '[deg]' * (i < 3) + '[mm]' * (i >= 3))
            plt.title('Comparing ' + str(components[i]) + ' Predictions Against Point Estimate')
            
        
        trim = np.where(np.array(correspondingFocus) != medFocus)
        trimRlist = deltaRlist[trim]
        trimTlist = deltaTlist[trim]
                
        # Take mean across rounds for each model
        meanRerrList = np.mean(trimRlist, axis=0)
        meanTerrList = np.mean(trimTlist, axis=0)
        
        xScale = 1
        labels = legend[1:-2] + ['Constant Center Fit', 'Linear Center Fit']
        # Plot this value using bar graph
        components = ['Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z']
        for i in range(len(components)):
            fig, ax = plt.subplots()
            if i < 3:
                x = np.arange(len(labels)) * xScale
                ax.bar(x, meanRerrList[:, i])
                ax.set_xticks(x)
                ax.set_xticklabels(labels)
                ax.set_ylabel('Mean Delta Across Positions for ' + components[i] + ' [deg]')
            else:
                x = np.arange(len(labels)) * xScale
                ax.bar(x, 1e3 * meanTerrList[:,i-3])
                ax.set_xticks(x)
                ax.set_xticklabels(labels)
                ax.set_ylabel('Mean Delta for ' + components[i] + ' [mm]')
            plt.title('Comparing ' + str(components[i]) + ' Predictions Against Point Estimate')
        
        # row = round, col = model
        avgReproj = np.mean(meanErrList, axis=0)
        fig, ax = plt.subplots()
        labels = [legend[0]] + labels # Include point
        
        x = np.arange(len(labels)) * xScale
        ax.bar(x, avgReproj)
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_ylabel('Mean Re-Projection Error Across Positions')
        
        # plt.figure()
        # # Each row corresponds to a given leaving out round
        # # Each column corresponds to a particular model
        # avgDistErr = np.squeeze(avgDistErr)
        # plt.scatter(correspondingFocus, avgDistErr[:, 0]) # median
        # plt.scatter(correspondingFocus, avgDistErr[:, 1]) # thin lens
        # for i, degDict in enumerate(degDicts):
        #     plt.scatter(correspondingFocus, avgDistErr[:, i+2])
        #     #plt.scatter(correspondingFocus, avgDistErr[:, i+1])
        # plt.legend(legend[1:])
        # #plt.xlabel('Left Out Dataset')
        # plt.xlabel('Left Out Dataset Focus')
        # plt.ylabel('Mean Distance Error (relative to Point) on Left Out Dataset')
        # plt.title('Comparing Distance Predictions Against Point Estimate')
        
    if NEW_APPROACH:
        legend = ['Point Estimation', 'Median Estimation', 'New Approach']
         
        leftOut, overallDistList, overallRotList, overallXList, overallYList, overallZList, medFocus, fitIntrinsics, fitDist = \
            newApproachLBeam(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, relLbeam, numRounds, sensor_size, f, qList)
                
        print('Median Focus Used: ' + str(medFocus))
        
        correspondingFocus = []
        for AFround in leftOut:
            correspondingFocus.append(searchForFocus(path + '/' + subfolderPrefixes + str(AFround) + '/' + relFocusPrefix + str(AFround) + '.txt'))
        
        names = ['Distance [m]', 'Rotation [deg]', 'X [m]', 'Y [m]', 'Z [m]']
        resultsList = [overallDistList, overallRotList, overallXList, overallYList, overallZList]
        
        #names = ['Distance [m]', 'Rotation [deg]']
        #resultsList = [overallDistList, overallRotList]
        
        for i, overallList in enumerate(resultsList):
            modelResults = [[] for k in range(len(legend))]
            
            for _, roundResults in enumerate(overallList):
                for j, individualResults in enumerate(roundResults):
                    modelResults[j].append([np.mean(individualResults), np.min(individualResults), np.max(individualResults)])
            
            plt.figure()
            plt.title('Comparing Model Predictions of ' + str(names[i]) + ' Between Boards')
            for k, model in enumerate(modelResults):
                # Each row is a given round, each column is either mean, min, or max
                model = np.array(model) 
                # Plot the error bar for the point estimate
                if k == 0:
                    lowerErr = model[:, 0] - model[:, 1]
                    upperErr = model[:, 2] - model[:, 0]
                    plt.errorbar(correspondingFocus, model[:,0], yerr=np.array([lowerErr, upperErr]), color = 'b', fmt='o', label=legend[k])
                # Plot the error bar for the median estimate
                elif k == 1:
                    lowerErr = model[:, 0] - model[:, 1]
                    upperErr = model[:, 2] - model[:, 0]
                    # Slight horizontal offset so can see both bars
                    plt.errorbar(np.array(correspondingFocus) + 2, model[:,0], yerr=np.array([lowerErr, upperErr]), color = 'g', fmt='o', label=legend[k])
                else:
                    #plt.scatter(correspondingFocus, model[:,0])
                    lowerErr = model[:, 0] - model[:, 1]
                    upperErr = model[:, 2] - model[:, 0]
                    # Slight horizontal offset so can see both bars
                    plt.errorbar(np.array(correspondingFocus) + 4, model[:,0], yerr=np.array([lowerErr, upperErr]), color = 'r', fmt='o', label=legend[k])
            plt.legend()
            plt.xlabel('Focus Position')
            plt.ylabel('Predicted ' + str(names[i]))
            
            avgErrors = []
            plt.figure()
            plt.title('Comparing Model Predictions of ' + str(names[i]) + ' Between Boards against Point Estimation')
            for model in modelResults[1:]:
                model = np.array(model)
                errList = np.abs(model[:, 0] - np.array(modelResults[0])[:,0])
                plt.scatter(correspondingFocus, errList)
                avgErrors.append(np.mean(errList))
            bestModel = np.argmin(avgErrors)
            print('Best performing model for ' + names[i] + ' was: ' + legend[bestModel + 1])
            print('Corresponding average error was: ' + str(avgErrors[bestModel]))
            plt.legend(legend[1:])
            plt.xlabel('Focus Position')
            plt.ylabel('Absolute Deviation from Point Estimate ' + str(names[i]))
    
    if L_BEAM:
        aspect1 = True
        regularize = False
        
        # Look into using higher order for distortion, seems like could be promising
        if aspect1:
            #degDicts = [{'f':deg1, 'u0':deg2, 'v0':deg2, 'k1':deg3, 'k2':deg3, 'k3':deg3} for deg1 in range(2,4) for deg2 in range(0,1) for deg3 in range(2,4)]
            degDicts = [{'f':deg1, 'u0':deg2, 'v0':deg2, 'k1':deg3, 'k2':deg3, 'k3':deg3} for deg1 in range(2,3) for deg2 in range(0,1) for deg3 in range(0,1)]
        else:
            degDicts = [{'fx':deg1, 'fy':deg1, 'u0':deg2, 'v0':deg2, 'k1':deg3, 'k2':deg3, 'k3':deg3} for deg1 in range(2,4) for deg2 in range(0,1) for deg3 in range(2,3)]

        if regularize:
            uCenter = np.array([3960])
            vCenter = np.array([3004])
            x0Dict = {'u0':uCenter, 'v0':vCenter}
            regPairedDicts = [ ({'u0':0.1, 'v0':0.1}, x0Dict), 
                              ({'u0':1, 'v0':1}, x0Dict),
                              ({'u0':10, 'v0':10}, x0Dict),
                              ({'u0':100, 'v0':100}, x0Dict)]
            legend = ['Point Estimation', 'Median Estimation'] + \
            [str(degDicts[i]) + ' ' + str(regPairedDicts[j]) for i in range(len(degDicts)) for j in range(len(regPairedDicts))]
           
        else:
            regPairedDicts = []
            legend = ['Point Estimation', 'Median Estimation'] + \
                [str(degDicts[i]) for i in range(len(degDicts))]
            
        #legend = ['Point Estimation'] + [str(degDicts[i]) for i in range(len(degDicts))] +\
        #    ['Moving ' + str(degDicts[i]) for i in range(len(degDicts))]
        
        #weightingFuncList = [lambda d: 1, lambda d: 1 / (1 + 1e-2 * d**2), lambda d: 1 / (1 + 1e-3 * d**2)]
        
        #legend = ['Point Estimation', 'Median Estimation'] + \
        #    [str(degDicts[i]) for i in range(len(degDicts))] + \
        #    ['Inverse Quadratic 1e-2 ' + str(degDicts[i]) for i in range(len(degDicts))] + \
        #    ['Inverse Quadratic 1e-3' + str(degDicts[i]) for i in range(len(degDicts))]
        
        weightingFuncList = [lambda d: 1] # Only test static LS

        #legend = ['Point Estimation', 'Median Estimation'] + \
        #    [str(degDicts[i]) for i in range(len(degDicts))]
         
        leftOut, overallDistList, overallRotList, overallXList, overallYList, overallZList, medFocus, fitIntrinsics, fitDist = \
            compareLBeam(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, relLbeam, numRounds, degDicts, aspect1, weightingFuncList, regPairedDicts)
        
        print('Median Focus Used: ' + str(medFocus))
        
        correspondingFocus = []
        for AFround in leftOut:
            correspondingFocus.append(searchForFocus(path + '/' + subfolderPrefixes + str(AFround) + '/' + relFocusPrefix + str(AFround) + '.txt'))
        
        names = ['Distance [m]', 'Rotation [deg]', 'X [m]', 'Y [m]', 'Z [m]']
        resultsList = [overallDistList, overallRotList, overallXList, overallYList, overallZList]
        
        #names = ['Distance [m]', 'Rotation [deg]']
        #resultsList = [overallDistList, overallRotList]
        
        for i, overallList in enumerate(resultsList):
            modelResults = [[] for k in range(len(legend))]
            
            for _, roundResults in enumerate(overallList):
                for j, individualResults in enumerate(roundResults):
                    modelResults[j].append([np.mean(individualResults), np.min(individualResults), np.max(individualResults)])
            
            # Need to reverse so that error bar works correctly
            plt.figure()
            plt.title('Comparing Model Predictions of ' + str(names[i]) + ' Between Boards')
            for k, model in enumerate(modelResults[::-1]):
                # Each row is a given round, each column is either mean, min, or max
                model = np.array(model) 
                # Plot the error bar for the point estimate
                if k == len(modelResults)-1:
                    lowerErr = model[:, 0] - model[:, 1]
                    upperErr = model[:, 2] - model[:, 0]
                    plt.errorbar(correspondingFocus, model[:,0], yerr=np.array([lowerErr, upperErr]), color = 'b', fmt='o')
                # Plot the error bar for the median estimate
                elif k == len(modelResults)-2:
                    lowerErr = model[:, 0] - model[:, 1]
                    upperErr = model[:, 2] - model[:, 0]
                    # Slight horizontal offset so can see both bars
                    plt.errorbar(np.array(correspondingFocus) + 2, model[:,0], yerr=np.array([lowerErr, upperErr]), color = 'g', fmt='o')
                else:
                    plt.scatter(correspondingFocus, model[:,0])
            plt.legend(legend[::-1])
            plt.xlabel('Focus Position')
            plt.ylabel('Predicted ' + str(names[i]))
            
            avgErrors = []
            plt.figure()
            plt.title('Comparing Model Predictions of ' + str(names[i]) + ' Between Boards against Point Estimation')
            for model in modelResults[1:]:
                model = np.array(model)
                errList = np.abs(model[:, 0] - np.array(modelResults[0])[:,0])
                plt.scatter(correspondingFocus, errList)
                avgErrors.append(np.mean(errList))
            bestModel = np.argmin(avgErrors)
            print('Best performing model for ' + names[i] + ' was: ' + legend[bestModel + 1])
            print('Corresponding average error was: ' + str(avgErrors[bestModel]))
            plt.legend(legend[1:])
            plt.xlabel('Focus Position')
            plt.ylabel('Absolute Deviation from Point Estimate ' + str(names[i]))
            
            paramList = ['focus position','f','k1','k2','k3']
            
            for param in paramList:
                plt.figure()
                plt.title('Correlating Fit relative to Median ' + param + ' Difference in ' + str(names[i]) + ' Estimate')
                medianModel = np.array(modelResults[1])
                for k, model in enumerate(modelResults[2:]):
                    # Each row is a given round, each column is either mean, min, or max
                    model = np.array(model)
                    
                    # The median intrinsics should obviously be constant
                    # assert(fitIntrinsics[0][1] == fitIntrinsics[1][1])
                    points = []
                    for rnd in range(len(model)):
                        # Get the difference in camera parameters used by this model
                        # relative to the median
                        diffIntrinsics = fitIntrinsics[rnd][k+2] - fitIntrinsics[rnd][1]
                        diffDist = fitDist[rnd][k+2] - fitDist[rnd][1]
                        if param == 'focus position':
                            diffParam = correspondingFocus[rnd] - medFocus
                        elif param == 'f':
                            diffParam = diffIntrinsics[0, 0]
                        elif param == 'k1':
                            diffParam = diffDist[0,1]                            
                        elif param == 'k2':
                            diffParam = diffDist[0,2]                            
                        elif param == 'k3':
                            diffParam = diffDist[0,3]                            
                        
                        diffEst = model[rnd, 0] - medianModel[rnd, 0]
                        points.append([diffParam, diffEst])
                    points = np.array(points)
                    plt.scatter(points[:,0], points[:,1])
                    
                    #print('\n' + legend[k+2] + ' Fit versus Median estimate for ' + names[i] + ' against param ' + param)
                    # Compute the correlation
                    #print("Pearson correlation coefficient and p value: ")
                    #coeff, pVal = stats.pearsonr(points[:,0], points[:,1])
                    #print(coeff, pVal)
                    
                    #print("Spearman correlation coefficient and p value: ")
                    #coeff, pVal = stats.spearmanr(points[:,0], points[:,1])
                    #print(coeff, pVal)
                    #print("Kendall Tau correlation coefficient and p value: ")
                    #coeff, pVal = stats.kendalltau(points[:,0], points[:,1])
                    #print(coeff, pVal)
    
                plt.legend(legend[2:])
                plt.xlabel('Difference of current intrinsics ' + param + ' from median intrinsics')
                plt.ylabel('Difference of estimated ' + str(names[i]) + ' for fit relative to median')
                
                
                plt.figure()
                plt.title('Correlating Median relative to Point ' + param + ' and Difference in ' + str(names[i]) + ' Estimate')
                points = []
                for rnd in range(len(medianModel)):
                    # Get the difference in camera parameters used by this model
                    # relative to the median
                    diffIntrinsics = fitIntrinsics[rnd][1] - fitIntrinsics[rnd][0]
                    diffDist = fitDist[rnd][1] - fitDist[rnd][0]
                    if param == 'focus position':
                        diffParam = correspondingFocus[rnd] - medFocus
                    elif param == 'f':
                        diffParam = diffIntrinsics[0, 0]
                    elif param == 'k1':
                        diffParam = diffDist[0,1]                            
                    elif param == 'k2':
                        diffParam = diffDist[0,2]                            
                    elif param == 'k3':
                        diffParam = diffDist[0,3]                            
                        
                    diffEst = medianModel[rnd, 0] - np.array(modelResults[0])[rnd, 0]
                    points.append([diffParam, diffEst])
                points = np.array(points)
                plt.scatter(points[:,0], points[:,1])
                print('\nMedian relative to Point for estimating ' + str(names[i]) + ' against param: ' + param)
                # Compute the correlation
                print("Pearson correlation coefficient and p value: ")
                coeff, pVal = stats.pearsonr(points[:,0], points[:,1])
                print(coeff, pVal)
                
                #print("Spearman correlation coefficient and p value: ")
                #coeff, pVal = stats.spearmanr(points[:,0], points[:,1])
                #print(coeff, pVal)
                #print("Kendall Tau correlation coefficient and p value: ")
                #coeff, pVal = stats.kendalltau(points[:,0], points[:,1])
                #print(coeff, pVal)
                
                plt.xlabel('Difference of median intrinsics ' + param + ' from point estimate intrinsics')
                plt.ylabel('Difference of estimated ' + str(names[i]) + ' for median relative to point estimate')
                