# Last Modified: 6/30/2020 Changes made by Aaron
# Used for running cross-validation and generating learning curves on a
# calibration dataset for a more thorough analysis

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
from sklearn.model_selection import KFold
import random
import matplotlib.pyplot as plt
import re

##########################
# Charuco Board Consts   #
##########################

# Aaron Changed to TV_4 was TV_3, put back to TV_3 since used big monitor again
squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

##########################
# Helper functions       #
##########################

# Compute mean reprojection error over a dataset given the calibration results
# Assumes blurred already
def error_for_dataset(image_list, intrinsics, dist, cameraName='imperx'):
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
            mean_error = charuco_reprojection_error(board,chorners,chids,R,tvec,intrinsics,dist)
            image_errors.append(mean_error)
        else:
            print("Skipping: Failed to find charuco board in image " + str(i))
    return image_errors, np.mean(image_errors)

def calibrateCamera(board, aruco_dict, image_list, cameraName='imperx'):
    calcorners = []
    calids = []
    
    for i, image in enumerate(image_list):
        parameters = cv2.aruco.DetectorParameters_create()
        if cameraName == 'imperx':
            parameters.minMarkerPerimeterRate = 0.025
        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        if (ids is not None and len(ids) > 8):
            ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, image, board)
    
            calcorners.append(chcorners)
            calids.append(chids)
            ids = np.reshape(ids, (ids.shape[0],))
            #print(len(chcorners))
        
    # Proceed with default model
    rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        calcorners, calids, board, (image_list[0].shape[1],image_list[0].shape[0]), None, None)
    
    return cameraMatrix, distCoeffs


# Assumes already permuted the image_list, n_splits gives number of splits
# to divide the data into, each CV run will leave one of the n_splits
# out as the test set
def computeCVerror(image_list, board, numSplits=5, 
                   aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
                   cameraName='imperx'):
    foldErrors = []
    kf = KFold(n_splits=numSplits)
    for train_index, test_index in kf.split(image_list):
        trainImages = [image_list[i] for i in train_index]
        testImages = [image_list[i] for i in test_index]
        # Now, calibrate on the trainImages
        intrinsics, dist = calibrateCamera(board, aruco_dict, trainImages)
        print('Finished calibrating camera')
       
        # Now, evaluate on testImages
        _, meanErr = error_for_dataset(image_list, intrinsics, dist)
        foldErrors.append(meanErr)
    return np.mean(foldErrors), np.std(foldErrors)

# Generate a plot showing mean reprojection error for in and out of sample
# frac is the fraction of data reserved for the test set
# increment is the step size used when varying the number of training examples
# graph dictates whether to plot the curve

def computeLearningCurve(image_list, frac, increment, graph=True):
    inSample = []
    outSample = []
    # Withold a validation set of the last part of the data
    tempTestSize = round((1-frac) * len(image_list))
    #testSet = image_list[testSize:]
    # Now vary the training set in increments up to the full remaining size
    # Compute the cutoff where to stop and not bleed over into the test
    stopPoint = len(image_list[:tempTestSize]) // increment
    trainingSizes = [increment*i for i in range(1, stopPoint+1)]
    
    # Take all of the remaining examples as test set
    testSet = image_list[stopPoint*increment:]
    
    print('Test Set Size: ' + str(len(testSet)))
    print('Max Training Set Size: ' + str(max(trainingSizes)))
    for trainingSize in trainingSizes:
        print('Training Size = ' + str(trainingSize))
        trainSet = image_list[:trainingSize]
        intrinsics, dist = calibrateCamera(board, aruco_dict, trainSet)
        # Compute error on the training set and test set
        _, Ein = error_for_dataset(trainSet, intrinsics, dist)
        _, Eout = error_for_dataset(testSet, intrinsics, dist)
        inSample.append(Ein)
        outSample.append(Eout)
    if graph:
        plt.figure(figsize=(9, 6))
        plt.title('Learning Curve for Calibration')
        plt.plot(trainingSizes, inSample, trainingSizes, outSample)
        plt.legend(['In Sample', 'Out Sample'])
        plt.xlabel('Number of Training Images')
        plt.ylabel('Re-projection Error')
    
    return trainingSizes, inSample, outSample

# Passed in separate training and tests sets so no splitting
def computeValidationLearningCurve(training_images, test_images, increment, graph=True):
    inSample = []
    outSample = []
    stopPoint = len(training_images) // increment
    trainingSizes = [increment*i for i in range(1, stopPoint+1)]
    
    print('Test Set Size: ' + str(len(test_images)))
    print('Max Training Set Size: ' + str(max(trainingSizes)))
    for trainingSize in trainingSizes:
        print('Training Size = ' + str(trainingSize))
        trainSet = training_images[:trainingSize]
        intrinsics, dist = calibrateCamera(board, aruco_dict, trainSet)
        # Compute error on the training set and test set
        _, Ein = error_for_dataset(trainSet, intrinsics, dist)
        _, Eout = error_for_dataset(test_images, intrinsics, dist)
        inSample.append(Ein)
        outSample.append(Eout)
    if graph:
        plt.figure(figsize=(9, 6))
        plt.title('Learning Curve for Calibration')
        plt.plot(trainingSizes, inSample, trainingSizes, outSample)
        plt.legend(['In Sample', 'Out Sample'])
        plt.xlabel('Number of Training Images')
        plt.ylabel('Re-projection Error')
    
    return trainingSizes, inSample, outSample
    
##########################
# Main code              #
##########################

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

cameraName = 'imperx' 

filenames = glob.glob('C:/Users/aofeldman/Desktop/FullDataset/*.tif')

image_list = [cv2.imread(img, 0) for img in filenames]

print('Size of images: ', image_list[0].shape[0])

print("Number of Images in Folder: " + str(len(image_list)))

# Slightly blur all images in dataset to prevent aliasing
image_list = [cv2.blur(image, (3,3)) for image in image_list]

RUN_CV = False

RUN_LEARNING = False

RUN_SPECIFIC_LEARNING = True

if RUN_CV:
    numRuns = 3
    
    roundCVerrors = []
    roundCVvars = []
    
    for run in range(numRuns):
        print('On run ' + str(run))
        # Permute image_list
        random.shuffle(image_list)
        err, var = computeCVerror(image_list, board)
        print('CV error: ' + str(err))
        print('CV STD: ' + str(var))
        roundCVerrors.append(err)
        roundCVvars.append(var)
    
    print('Mean CV errors were ' + str(roundCVerrors))
    print('STD of CV errors were ' + str(roundCVvars))
    
if RUN_LEARNING:
    numRuns = 3
    
    for run in range(numRuns):
        print('On run ' + str(run))
        # Permute image_list
        random.shuffle(image_list)
        trainingSizes, inSample, outSample = computeLearningCurve(image_list, 1/5, 5)
        
if RUN_SPECIFIC_LEARNING:
    numRuns = 3
    
    leftNames = glob.glob('C:/Users/aofeldman/Desktop/OutSample/Left/*.tif')
    rightNames = glob.glob('C:/Users/aofeldman/Desktop/OutSample/right/*.tif')
    
    validationNames = leftNames + rightNames
    
    filenames = glob.glob('C:/Users/aofeldman/Desktop/FullDataset/*.tif')
    
    trainingNames = []
    for filename in filenames:
        # Extract just the image name
        # Split at both / and \\
        imageName = re.split('/|\\\\', filename)[-1]
        print(imageName)
        # If it appears in any validation set skip it
        if not sum([(imageName in validationName) for validationName in validationNames]):
            trainingNames.append(filename)
        
    print('Number of images in left set: ' + str(len(leftNames)))
    print('Number of images in right set: ' + str(len(rightNames)))
    print('Number of images across all validation sets: ' + str(len(validationNames)))
    print('Number of images in training set: ' + str(len(trainingNames)))
    
    # Load the actual images and blur
    trainSet = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in trainingNames]
    leftSet = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in leftNames]
    rightSet = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in rightNames]
    
    # Randomly order the trainSet
    for run in range(numRuns):
        print('On run ' + str(run))
        random.shuffle(trainSet)
        
        leftTrainingSizes, leftIn, leftOut = computeValidationLearningCurve(trainSet, leftSet, 5, False)
        rightTrainingSizes, rightIn, rightOut = computeValidationLearningCurve(trainSet, rightSet, 5, False)
        assert leftTrainingSizes == rightTrainingSizes
        
        # Plot on same plot for comparison
        plt.figure(figsize=(9, 6))
        plt.title('Comparing Learning Curves for Left versus Right Images')
        plt.plot(leftTrainingSizes, leftIn, leftTrainingSizes, leftOut, rightTrainingSizes, rightOut)
        plt.legend(['In Sample', 'Left Out Sample', 'Right Out Sample'])
        plt.xlabel('Number of Training Images')
        plt.ylabel('Re-projection Error')
            