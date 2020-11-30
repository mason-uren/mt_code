# Last Modified: 8/4/2020 Changes made by Aaron
# Used for detecting and estimating the relative pose of the two boards of the
# L-beam in Imperx images. Detection parameters may need to change depending on
# the dataset.

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

##########################
# Helper Functions       #
##########################
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
    
    return cameraMatrix, distCoeffs, rvecs, tvecs

def rollMat(theta):
    return np.array([[1, 0, 0], 
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta), np.cos(theta)]])

def scaleDown(image, scale):
    width = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)
    dim = (width, height)
    # resize image
    return cv2.resize(image, dim)

counter = 1
def findBoards(image, aruco_dict, board, intrinsics, dist, verbose = False, graph=False, splitRange = range(5,6), parameters=cv2.aruco.DetectorParameters_create()):
    global counter
    
    # Empirically observed decent parameters 
    parameters = cv2.aruco.DetectorParameters_create()
    #parameters.minMarkerPerimeterRate = 0.01
    #parameters.polygonalApproxAccuracyRate = 0.08
    #parameters.adaptiveThreshWinSizeStep = 5
    #parameters.adaptiveThreshWinSizeMin = 13
    #parameters.adaptiveThreshWinSizeMin = 17
    #parameters.adaptiveThreshConstant = 1
    
    # Consider a vertical split at different locations across the image
    height, width = np.shape(image)
    
    #splits = [round(0.1*i*width) for i in range(4, 7)]
    
    splits = [round(0.1*i*width) for i in splitRange]

    # For each split try to find a board on both sides 
    # Add a buffer so that cannot detect one board as two
    buffer = width // 30
    
    maxCorners = 1 # So that way to update must have detected in both sides
    poses = []
    errors = []
    chosenSplit = -1
    
    colorImgLeft = None
    colorImgRight = None
    
    # Draw lines on the image in the given locations
    for index, split in enumerate(splits):
        image_copy = image.copy()
        xLeft = split-buffer
        xRight = split+buffer
        if verbose:
            print('Starting a new split')
            print('Index ' + str(index) + ' into splits: ' + str(splits))
            print('split: ' + str(split))
            print('xLeft: ' + str(xLeft))
            print('xRight: ' + str(xRight))
        if graph:
            cv2.line(image_copy, (xLeft, 0), (xLeft, height), (255, 0, 0), 5)
            cv2.line(image_copy, (xRight, 0), (xRight, height), (255, 0, 0), 5)
        
        #cv2.imshow('split '+str(split), scaleDown(image_copy, 0.25))
        #cv2.waitKey(0)
        
        # Now, want to independently find boards in the left and right portions
        # of the image
        imageLeft = image[:, :xLeft]
        imageRight = image[:, xRight:]

        numCorners = [0, 0]
        calcorners = []
        calids = []
        # Keep track of number of charuco corners gotten in left and right
        for isRight, halfImage in enumerate([imageLeft, imageRight]):
            markerCorners, ids, rejectedCorners = cv2.aruco.detectMarkers(halfImage, aruco_dict, None, None, parameters)
            #markerCorners, ids, rejectedCorners, recoveredInd = cv2.aruco.refineDetectedMarkers(halfImage, board, markerCorners, ids, rejectedCorners)
            
            if ids is not None and len(ids) > 5:
                # markerCorners is a list where each item corresponds to a
                # given marker and is a (1,4,2) float32 array.
                # Offset the markerCorners by the amount that was shifted
                # Each point in markerCorners[i] is of form (x,y)
                for i, cornerArr in enumerate(markerCorners):
                    if isRight:
                        # First dimension is the "fake" one
                        cornerArr[0, :, 0] += xRight
                    markerCorners[i] = cornerArr
                    #if graph:
                    #    #Draw the new markerCorners to make sure they make sense
                    #    markerPoints = np.squeeze(np.copy(markerCorners[i]).astype(int))
                    #    for row in range(len(markerPoints)):
                    #        point = markerPoints[row, :]
                    #        image_copy[point[1]-25:point[1]+25, point[0]-25:point[0]+25] = 255
                    #        cv2.imshow('added points', scaleDown(image_copy, 0.15))
                    #        cv2.waitKey(0)
                
                ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                if ret:
                    numCorners[isRight] = len(chcorners)
                    calcorners.append(chcorners)
                    calids.append(chids)
                    ids = np.reshape(ids, (ids.shape[0],))
                else:    
                    numCorners[isRight] = 0
                    calcorners.append([])
                    calids.append([])
            else:
                numCorners[isRight] = 0
                calcorners.append([])
                calids.append([])
                
        # Compute the minimum of numCorners, if it exceeds current best
        # use it instead for estimation
        temp = min(numCorners)
        if temp > maxCorners:
            if verbose:
                print('Updating best split')
            maxCorners = temp
            # Now, do board pose estimation for each board
            poses = []
            errors = []
            for i in range(2):
                chcorners = calcorners[i]
                chids = calids[i]
                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chcorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
                #retval, rvec, tvec, inliers = estimate_pose_ransac(board, chcorners, chids, intrinsics, dist)
                #pdb.set_trace()
                #print('rvec:', rvec, 'rvec2:', rvec2)
                #print('tvec:', tvec, 'tvec2:', tvec2)
                #print('Fraction of data used as inliers: ' + str(len(inliers) / len(chcorners)))
                
                if retval:
                    poses.append([rvec, tvec])
                    # Compute the reprojection error
                    R, _ = cv2.Rodrigues(rvec)
                    mean_error = charuco_reprojection_error(board,chcorners,chids,R,tvec,intrinsics,dist)
                    errors.append(mean_error)
                    if graph:
                        colorImg = cv2.cvtColor(image_copy, cv2.COLOR_GRAY2BGR)

                        # Draw the chessboard corners
                        points = np.squeeze(np.copy(chcorners).astype(int))
                        for row in range(len(points)):
                            point = points[row, :]
                            colorImg[point[1]-25:point[1]+25, point[0]-25:point[0]+25] = (0, 0, 255)
                        
                        # Draw the axis
                        cv2.aruco.drawAxis(colorImg, intrinsics, dist, rvec, tvec, 0.2)
                        #cv2.imshow('Axis drawn', scaleDown(colorImg, 0.2))
                        #cv2.waitKey(0)
                        if i == 0:
                            colorImgLeft = np.copy(colorImg)
                        else:
                            colorImgRight = np.copy(colorImg)
                else:
                    print('Failed to estimate pose')
            chosenSplit = split
    
    if graph and chosenSplit != -1:
        for i, img in enumerate([colorImgLeft, colorImgRight]):
            cv2.imwrite('C:/Users/aofeldman/Desktop/LbeamTest/results/' + str(counter) + (1 - i) * 'L' + i * 'R' + '.jpg', img)
        counter += 1
                        
    return poses, errors, chosenSplit

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def estimateBeam(image_list, intrinsics, dist, aruco_dict, board, verbose = True, useNormAngle = True, splitRange = range(5,6)):
    # Controls minor change regarding how compute the desired relative
    # rotation and translation
    decoupled = False
    
    estimatedDistances = []
    estimatedRolls = []
    estimatedTranslations = []
    
    numImagesSkipped = 0
    for imgIndex, image in enumerate(image_list):
        if verbose:
            print('\n\n')
            print('Starting image: ' + str(imgIndex))
            
        poses, errors, chosenSplit = findBoards(image, aruco_dict, board, intrinsics, dist, False, False, splitRange)
        
        if poses is not None and len(poses) == 2:
            if decoupled:
                # Passive rotation that takes a point described in left board frame to now
                # be described in the camera frame
                leftR, _ = cv2.Rodrigues(poses[0][0])
                # Same as above but for right board frame
                rightR, _ = cv2.Rodrigues(poses[1][0])
                # Given a point described in the right board frame, first transform it so
                # that it is described in the camera frame (by applying rightR) and then
                # transform that point described in the camera frame to be described in the
                # left board frame (by appling inverse of leftR). left2rightRot gives
                # the passive rotation that describes how a given point in the right board
                # frame would be expressed in the left board frame without shift of origin.
                # Equivalently, this expresses the rotation from the left board frame axes
                # to the right board frame axes.
                
                if useNormAngle:
                    # Compute the angle between the boards via the angle between
                    # the normal vectors
                    perpLeft = np.matmul(leftR, np.array([0,0,1]))
                    perpLeft *= 1 / np.linalg.norm(perpLeft)
                    perpRight = np.matmul(rightR, np.array([0,0,1]))
                    perpRight *=  1 / np.linalg.norm(perpRight)
                    angleShift = np.arccos(np.dot(perpLeft, perpRight))
                    #print('Angle Shift: ' + str(np.rad2deg(angleShift)))
                    degRoll = np.rad2deg(angleShift)
                else:
                    left2rightRot = np.matmul(leftR.T, rightR)
                    # roll about x, pitch about y, yaw about z
                    roll, pitch, yaw = rotationMatrixToEulerAngles(left2rightRot)
                    # With new boards seems like should extract pitch
                    # degRoll = np.rad2deg(roll)
                    degRoll = np.rad2deg(pitch)
                    
                # Now compute the estimated translation
                
                # leftT describes the translation of the left board from the origin of the
                # camera frame, using the axes of the camera. Similar for rightT
                leftT = poses[0][1]
                rightT = poses[1][1]    
                # Note that distance is invariant under affine transformation so we
                # can decouple estimation of rotation and translation
                T = leftT - rightT
                normedT = np.linalg.norm(T)
                
            else:
                # Look at the transform taking points from the left board system to the
                # right board system
                leftR, _ = cv2.Rodrigues(poses[0][0])
                tempLeft = np.hstack([leftR, poses[0][1]])
                leftWrtCam = np.vstack([tempLeft,[[0,0,0,1]]])
                
                rightR, _ = cv2.Rodrigues(poses[1][0])
                tempRight = np.hstack([rightR, poses[1][1]])
                rightWrtCam = np.vstack([tempRight,[[0,0,0,1]]])
                
                rightWrtLeft = np.matmul(np.linalg.inv(leftWrtCam), rightWrtCam)   
                
                Rot = rightWrtLeft[:-1,:-1]
                T = rightWrtLeft[:-1,-1]
                
                # With new boards, seems like should extract pitch
                roll, pitch, yaw = rotationMatrixToEulerAngles(Rot)
                #degRoll = np.rad2deg(roll)
                degRoll = np.rad2deg(pitch)
                normedT = np.linalg.norm(T)
                
                print('roll, pitch, yaw: ', roll, pitch, yaw)
                
                if useNormAngle:
                    # Compute the angle between the boards via the angle between
                    # the normal vectors
                    perpLeft = np.matmul(leftWrtCam[:-1,:-1], np.array([0,0,1]))
                    perpLeft *= 1 / np.linalg.norm(perpLeft)
                    perpRight = np.matmul(rightWrtCam[:-1,:-1], np.array([0,0,1]))
                    perpRight *=  1 / np.linalg.norm(perpRight)
                    angleShift = np.arccos(np.dot(perpLeft, perpRight))
                    #print('Angle Shift: ' + str(np.rad2deg(angleShift)))
                    degRoll = np.rad2deg(angleShift)
                    
            #if np.abs(degRoll - 58) > 10 or np.abs(normedT - 0.68) > 0.1:
            #    print("Outlier detection")
            #    # Repeat but this time with graphing        
            #    #poses, errors, chosenSplit = findBoards(image, aruco_dict, board, intrinsics, dist, False, True)
            

            estimatedRolls.append(degRoll)
            if verbose:
                print('Rotating the left board by ' + str(degRoll) + ' deg gives the right board orientation')
            estimatedDistances.append(normedT)
            if verbose:
                print('Distance between the left and right boards is ' + str(normedT) + ' m')
            estimatedTranslations.append(T)    
            
        else:
            print('Failed to find boards: skipping image ' + str(imgIndex))
            numImagesSkipped += 1
    if verbose:
        print('Sample size: ' + str(len(image_list) - numImagesSkipped))
        print('Mean of rotation (deg): ' + str(np.mean(estimatedRolls)))
        print('Standard Deviation of rotation (deg): ' + str(np.std(estimatedRolls)))
        print('Mean of Distance (m): ' + str(np.mean(estimatedDistances)))
        print('Standard Deviation of Distance (m): ' + str(np.std(estimatedDistances)))
    return estimatedDistances, estimatedRolls, estimatedTranslations

def LBeamLearningCurve(image_list, LbeamImages, increment, aruco_dict, fiducial, board, graph=True, cameraName = 'ximea'):
    # Now vary the training set in increments up to the full remaining size
    # Compute the cutoff where to stop and not bleed over into the test
    stopPoint = len(image_list) // increment
    trainingSizes = [increment*i for i in range(1, stopPoint+1)]
    
    # Want to plot the mean and the max, min bars for each time that estimate
    names = ['Distance [m]', 'Rotation [deg]']
    lowerErr = [[],[]]
    upperErr = [[],[]]
    means = [[],[]]
    intrinsicsList = []
    distList = []
    
    print('Max Training Set Size: ' + str(max(trainingSizes)))
    for trainingSize in trainingSizes:
        print('Training Size = ' + str(trainingSize))
        trainSet = image_list[:trainingSize]
        intrinsics, dist, _, _ = calibrateCamera(board, aruco_dict, trainSet, cameraName)
        estimatedDistances, estimatedRolls, estimatedTranslations = \
            estimateBeam(LbeamImages, intrinsics, dist, aruco_dict, fiducial, True, True, range(4, 8))
        intrinsicsList.append(intrinsics)
        distList.append(dist)
        
        for i, result in enumerate([estimatedDistances, estimatedRolls]):
            means[i].append(np.mean(result))
            upperErr[i].append(np.max(result) - means[i][-1])
            lowerErr[i].append(means[i][-1] - np.min(result))

    if graph:
        for i, name in enumerate(names):
            plt.figure(figsize=(9,6))
            plt.title('Learning Curve for L-Beam Estimation: ' + name)
            plt.errorbar(trainingSizes, means[i], yerr=np.array([lowerErr[i], upperErr[i]]), fmt='o')
            plt.xlabel('Training Set Size')
            plt.ylabel('Estimated ' + name)
            
    return means, lowerErr, upperErr, intrinsicsList, distList


if __name__ == '__main__':
    ##########################
    # Charuco Board Consts   #
    ##########################
    
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
    
    prefix = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined/AFround12/'
    
    # Load the appropriate intrinsics, distCoeffs
    intrinsics = np.load(prefix + cameraName + '_intrinsics.npy')
    dist = np.load(prefix + cameraName + '_distCoeffs.npy')
    
    
    #foldername = 'L-beam'
    #foldername = 'oneLight'
    #foldername = 'LbeamTest'
    #foldername = 'BrightLbeamTest'
    #foldername = 'LbeamTest2'
    #foldername = 'MediumLbeamTest'
    #foldername = 'GainExperiments'
    foldername = 'internalVariation/gain12'
    #foldername = 'testCollection8-10Refined/AFround4/L-Beam'
    filenames = glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.jpg') + \
                glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.tif')
    
    filenames.sort()
    
    # Aaron added in reading in as grayscale
    image_list = [cv2.imread(img, 0) for img in filenames]
    # Slightly blur all images 
    #image_list = [cv2.blur(img, (3,3)) for img in image_list]
    
    # Try histogram equalization
    # image_list = [cv2.equalizeHist(img, 0) for img in image_list]
    
    print(image_list[0].shape[0])
    
    print("Number of Images in Folder: " + str(len(image_list)))
    
    estimatedDistances, estimatedRolls, estimatedTranslations = \
        estimateBeam(image_list, intrinsics, dist, aruco_dict, board, True, True, range(4,8))

    # TODO:
    # Compare variances under 1. Use norm and 12 mm 2. Don't use norm and 12 mm
    # 3. Use norm and 12.7 mm 4. Don't use norm and 12.7 mm
    # Need to actually check if the modified parameters I am using for detection
    # Also impact estimation

    # 1: (modified detection parameters)
    # Mean of rotation (deg): 0.5714898080055255
    # Standard Deviation of rotation (deg): 0.2854214070761459
    # Mean of Distance (m): 0.32697731775477235
    # Standard Deviation of Distance (m): 0.0004983292298196308
    
    # 1: (default detection parameters)
    # Mean of rotation (deg): 0.5678405079051441
    # Standard Deviation of rotation (deg): 0.28894658008018603
    # Mean of Distance (m): 0.32695119302889186
    # Standard Deviation of Distance (m): 0.0004665869273423205
    
    # 2: (default detection parameters)
    # Mean of rotation (deg): 0.4181248055553094
    # Standard Deviation of rotation (deg): 0.29023549240410856
    # Mean of Distance (m): 0.32695119302889186
    # Standard Deviation of Distance (m): 0.0004665869273423205



    # # DEAD CODE:
    # # # Write out the ground truth transformation describing the right board wrt
    # # # left board (transforms points described in the right board frame to be described in the left board frame)
    # # # Could actually fit the extrinsics for this but as of now just measured
    
    # # # Translation vector describing the translation of the right board 
    # # T = np.array([54.45125e-2, 40.64e-2, 0])
    
    # # #theta = np.deg2rad(-(90-50.44))
    # # theta = np.deg2rad(90-50.44)
    
    # # Rot = rollMat(theta)
    
    # # Write in homogoneous coordinates
    # #temp = np.hstack([Rot, np.expand_dims(T, 1)])
    # #true_extrinsics = np.vstack((temp,[[0,0,0,1]]))
    
    # # Controls minor change regarding how compute the desired relative 
    # # rotation and translation
    # decoupled = False
    
    # estimatedDistances = []
    # estimatedRolls = []
    
    # numImagesSkipped = 0
    # for imgIndex, image in enumerate(image_list):
    #     print('\n\n')
    #     print('Starting image: ' + str(imgIndex))
    #     poses, errors, chosenSplit = findBoards(image, aruco_dict, board, intrinsics, dist)
        
    #     if poses is not None and len(poses) == 2:
    #         if decoupled:
    #             # Passive rotation that takes a point described in left board frame to now
    #             # be described in the camera frame
    #             leftR, _ = cv2.Rodrigues(poses[0][0])
    #             # Same as above but for right board frame
    #             rightR, _ = cv2.Rodrigues(poses[1][0])
    #             # Given a point described in the right board frame, first transform it so
    #             # that it is described in the camera frame (by applying rightR) and then
    #             # transform that point described in the camera frame to be described in the
    #             # left board frame (by appling inverse of leftR). left2rightRot gives
    #             # the passive rotation that describes how a given point in the right board
    #             # frame would be expressed in the left board frame without shift of origin.
    #             # Equivalently, this expresses the rotation from the left board frame axes
    #             # to the right board frame axes.
    #             left2rightRot = np.matmul(leftR.T, rightR)
    #             # roll about x, pitch about y, yaw about z
    #             roll, pitch, yaw = rotationMatrixToEulerAngles(left2rightRot)
                
    #             degRoll = np.rad2deg(roll)
                
    #             # Now compute the estimated translation
                
    #             # leftT describes the translation of the left board from the origin of the
    #             # camera frame, using the axes of the camera. Similar for rightT
    #             leftT = poses[0][1]
    #             rightT = poses[1][1]    
    #             # Note that distance is invariant under affine transformation so we
    #             # can decouple estimation of rotation and translation
    #             normedT = np.linalg.norm(leftT - rightT)
    #         else:
    #             # Look at the transform taking points from the left board system to the
    #             # right board system
    #             leftR, _ = cv2.Rodrigues(poses[0][0])
    #             tempLeft = np.hstack([leftR, poses[0][1]])
    #             leftWrtCam = np.vstack([tempLeft,[[0,0,0,1]]])
                
    #             rightR, _ = cv2.Rodrigues(poses[1][0])
    #             tempRight = np.hstack([rightR, poses[1][1]])
    #             rightWrtCam = np.vstack([tempRight,[[0,0,0,1]]])
                
    #             rightWrtLeft = np.matmul(np.linalg.inv(leftWrtCam), rightWrtCam)   
                
    #             Rot = rightWrtLeft[:-1,:-1]
    #             T = rightWrtLeft[:-1,-1]
                
    #             roll, pitch, yaw = rotationMatrixToEulerAngles(Rot)
    #             degRoll = np.rad2deg(roll)
    #             normedT = np.linalg.norm(T)
                
    #         estimatedRolls.append(degRoll)
    #         print('Rotating the left board by ' + str(degRoll) + ' deg gives the right board orientation')
    #         estimatedDistances.append(normedT)
    #         print('Distance between the left and right boards is ' + str(normedT) + ' m')
    #     else:
    #         print('Failed to find boards: skipping image ' + str(imgIndex))
    #         numImagesSkipped += 1
    
    # print('Sample size: ' + str(len(image_list) - numImagesSkipped))
    # print('Mean of rotation (deg): ' + str(np.mean(estimatedRolls)))
    # print('Standard Deviation of rotation (deg): ' + str(np.std(estimatedRolls)))
    # print('Mean of Distance (m): ' + str(np.mean(estimatedDistances)))
    # print('Standard Deviation of Distance (m): ' + str(np.std(estimatedDistances)))
    
    # # DEAD CODE:
    # # # Test that got ground truth correctly 
    # # # Take the right board points expressed in the right chessboard frame
    # # # Apply the transform so that they are expressed in the left chessboard frame
    # # # Then, use project points using the left board computed rvec, tvec and see
    # # # if get roughly coloring of the pixels of the right frame
    
    # # # Make the chessboardCorners homogenous
    # # extra = np.ones(len(board.chessboardCorners))
    # # # Transpose so each column is one homogenous point
    # # homPoints = np.hstack([board.chessboardCorners, np.expand_dims(extra, 1)]).T
    # # # Transform to left board frame and ignore the last row which is just 1 because homogenous
    # # leftPoints = np.matmul(true_extrinsics, homPoints)[:-1,:]
    # # leftRvec = poses[0][0]
    # # leftTvec = poses[0][1]
    # # imagePoints, _ = cv2.projectPoints(leftPoints, leftRvec, leftTvec, intrinsics, dist)
    # # # Note that imagePoints will be (x,y)
    # # colorImg = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    # # for point in imagePoints[:, 0, :]: # imagePoints has 1-size inner dimension
    # #     point = point.astype(int)
    # #     cv2.circle(colorImg, (point[0], point[1]), 4, (0, 0, 255), 8)
    # # cv2.imshow('Truth check', scaleDown(colorImg, 0.2))
    # # cv2.waitKey(0)
    
    

    