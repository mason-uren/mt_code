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
from dynamicIntrinsicsHelpers import loadFixedModels


def findBoards(image, aruco_dict, board, intrinsics, dist, splitRange = range(5,6)):
    parameters = cv2.aruco.DetectorParameters_create()
    
    # Consider a vertical split at different locations across the image
    height, width = np.shape(image)
        
    splits = [round(0.1*i*width) for i in splitRange]

    # For each split try to find a board on both sides 
    # Add a buffer so that cannot detect one board as two
    buffer = width // 30
    
    maxCorners = 1 # So that way to update must have detected in both sides
    poses = []
    errors = []
    boardOrigins = []
    chosenSplit = -1
    
    # Draw lines on the image in the given locations
    for index, split in enumerate(splits):
        image_copy = image.copy()
        xLeft = split-buffer
        xRight = split+buffer
       
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
            maxCorners = temp
            # Now, do board pose estimation for each board
            poses = []
            errors = []
            boardOrigins = []
            for i in range(2):
                chcorners = calcorners[i]
                chids = calids[i]
                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chcorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
                
                #print('tvec: ', tvec)
                
                if retval:
                    poses.append([rvec, tvec])
                    # Compute the reprojection error
                    R, _ = cv2.Rodrigues(rvec)
                    mean_error = charuco_reprojection_error(board,chcorners,chids,R,tvec,intrinsics,dist)
                    errors.append(mean_error)
                    
                    imagePoints, _ = cv2.projectPoints(np.array([[0.0,0.0,0.0]]), rvec, tvec, intrinsics, dist)
                    boardOrigins.append(np.squeeze(imagePoints) - intrinsics[0:2, 2])
                    
                    # if i == 1:
                    #     imagePoints = np.squeeze(imagePoints)
                    #     color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    #     color_image[int(imagePoints[1])-20:int(imagePoints[1])+20, int(imagePoints[0])-20:int(imagePoints[0])+20] = (0, 0, 255)
                    #     color_image[int(intrinsics[1,2])-20:int(intrinsics[1,2])+20, int(intrinsics[0,2])-20:int(intrinsics[0,2])+20] = (0, 255, 0)
                    #     color_image = cv2.resize(color_image, (int(0.15 * color_image.shape[1]), int(0.15 * color_image.shape[0])))
                    #     cv2.imshow('detected origin', color_image)
                    #     cv2.waitKey(0)
                else:
                    print('Failed to estimate pose')
            chosenSplit = split
                        
    return poses, errors, boardOrigins, chosenSplit


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

# Given a single image compute the slope M between p and q
def computeM(image, board, aruco_dict, sensor_size, f, intrinsics, dist, qList, graph = False):
    # Convert from pixel to world units
    heightConversion = sensor_size[0] / image_list[0].shape[0]
    widthConversion = sensor_size[1] / image_list[0].shape[1]
    
    qx = np.array(qList) * widthConversion
    qy = np.array(qList) * heightConversion
    
    intrinsicsList = []
    for q in qList:
        cameraMatrix = np.copy(intrinsics)
        cameraMatrix[0, 0] = q
        cameraMatrix[1, 1] = q
        intrinsicsList.append(cameraMatrix)
    
    p = []
    
    for j, intrinsics in enumerate(intrinsicsList):
        print('Progress: ' + str(j / len(intrinsicsList)))
        
        parameters = cv2.aruco.DetectorParameters_create()
    
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
            
            p.append(tvec[2])
    
    poly = np.polyfit(qx, p, 1)
    
    p = np.squeeze(p)
    
    if graph:
        plt.figure()
        plt.title('Estimated p versus qx')
        plt.scatter(qx, p, label='Measured')
        plt.plot(qx, np.polyval(poly, qx), label=(f'Linear LS Fit: {poly[0][0]:.4f} q + {poly[1][0]:.4f}'))
        plt.xlabel('qx [m]')
        plt.ylabel('p [m]')
        plt.legend()
        
    # M = poly[0]
    return qx, p, poly

# qRange should be in pixel units
# sensor size should be in m (height, width)
# f should focal length (zoom) in m
def theoryFit(image_list, board, aruco_dict, sensor_size, f, defaultIntrinsics, dist, qList):

    # Convert from pixel to world units
    heightConversion = sensor_size[0] / image_list[0].shape[0]
    widthConversion = sensor_size[1] / image_list[0].shape[1]
    
    qx = np.array(qList) * widthConversion
    qy = np.array(qList) * heightConversion
    
    intrinsicsList = []
    for q in qList:
        cameraMatrix = np.copy(defaultIntrinsics)
        cameraMatrix[0, 0] = q
        cameraMatrix[1, 1] = q
        intrinsicsList.append(cameraMatrix)
    
    hx = []
    hy = []
    hPrimeX = []
    hPrimeY = []
    p = []
    roll = []
    pitch = []
    yaw = []
    errs = []
    
    for j, intrinsics in enumerate(intrinsicsList):
        print('Progress: ' + str(j / len(intrinsicsList)))
        rotations = []
        translations = []
        boardOrigins = []
        reproj_err = 0
        for image in image_list:
            # Estimate the pose of the charuco board in the image
            parameters = cv2.aruco.DetectorParameters_create()
    
            markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
            
            # if we dont find enough points skip
            if (ids is not None and len(ids) > 8):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                
                # findOrigin = np.squeeze(ids) == 0
                # if np.sum(findOrigin) == 0:
                #     print('Skipping board origin estimate, failed to detect origin marker')
                # else:
                #     originMarkerLoc = np.where(np.squeeze(ids) == 0)[0][0]
                #     originMarker = np.squeeze(markerCorners)[originMarkerLoc]
                #     origin = originMarker[0]
                #     color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                #     color_image[int(origin[1])-20:int(origin[1])+20, int(origin[0])-20:int(origin[0])+20] = (0, 0, 1)
                #     cv2.imshow('Origin plotted', cv2.resize(color_image, (int(0.15 * color_image.shape[1]), int(0.15 * color_image.shape[0]))))
                #     cv2.waitKey(0)
                    
                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
                rotations.append(rvec)
                translations.append(tvec)
                # Get 2D position of board center using project points
                # TODO: Measuring hPrimeX in this way makes it dependent on the rvec, tvec detection whereas if do it based on
                # a corner detection would not be
                imagePoints, _ = cv2.projectPoints(np.array([[0.0,0.0,0.0]]), rvec, tvec, intrinsics, dist)
                boardOrigins.append(np.squeeze(imagePoints) - intrinsics[0:2, 2])
                
                reproj_err += charuco_reprojection_error(board, chorners, chids, rvec, tvec, intrinsics, dist)

            else:
                print("Skipping: Failed to find charuco board in image " + str(i))
        
        # Track average reprojection error as function of q
        errs.append(reproj_err / len(image_list))
        # Each row is of form: roll, pitch, yaw
        # Each column is against the different q
        rotations = np.array([np.rad2deg(rotationMatrixToEulerAngles(cv2.Rodrigues(rotations[i])[0])) for i in range(len(rotations))])
        avgR = np.mean(rotations, axis=0)
        minR = np.min(rotations, axis=0)
        maxR = np.max(rotations, axis=0)
        roll.append([avgR[0], minR[0], maxR[0]])
        pitch.append([avgR[1], minR[1], maxR[1]])
        yaw.append([avgR[2], minR[2], maxR[2]])
        
        # Ultimately, should probably do something better where account for the difference in orientation across the frames
        # For now, will simply use the translations as if orientations are the same
        avgT = np.mean(translations, axis=0) # Take mean going down a column
        maxT = np.max(translations, axis=0)
        minT = np.min(translations, axis=0)
        hy.append([avgT[0], minT[0], maxT[0]])
        hx.append([avgT[1], minT[1], maxT[1]])
        
        p.append([avgT[2], minT[2], maxT[2]])
        
        avgHPrime = np.mean(boardOrigins, axis=0)
        maxHPrime = np.max(boardOrigins, axis=0)
        minHPrime = np.min(boardOrigins, axis=0)
                
        hPrimeY.append([avgHPrime[0] * heightConversion, minHPrime[0] * heightConversion, maxHPrime[0] * heightConversion])
        hPrimeX.append([avgHPrime[1] * widthConversion, minHPrime[1] * widthConversion, maxHPrime[1] * widthConversion])

    hx = np.squeeze(np.array(hx))
    hy = np.squeeze(np.array(hy))
    p = np.squeeze(np.array(p))
    hPrimeX = np.squeeze(np.array(hPrimeX))
    hPrimeY = np.squeeze(np.array(hPrimeY))
    roll = np.squeeze(np.array(roll))
    pitch = np.squeeze(np.array(pitch))
    yaw = np.squeeze(np.array(yaw))
    
    return hx, hy, p, hPrimeX, hPrimeY, qx, qy, roll, pitch, yaw, errs
    
def theoryBeam(image, defaultIntrinsics, dist, aruco_dict, board, qList, splitRange = range(5,6)):
    individualPoses = []
    boardImgPoses = []

    intrinsicsList = []
    for q in qList:
        cameraMatrix = np.copy(defaultIntrinsics)
        cameraMatrix[0, 0] = q
        cameraMatrix[1, 1] = q
        intrinsicsList.append(cameraMatrix)
    
    for i, intrinsics in enumerate(intrinsicsList):
        print('Progress: ' + str(i / len(intrinsicsList)))
        poses, errors, boardOrigins, chosenSplit = findBoards(image, aruco_dict, board, intrinsics, dist, splitRange)
        individualPoses.append(poses)
        boardImgPoses.append(boardOrigins)

    return individualPoses, boardImgPoses

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
    plt.close('all')
    
    SINGLE_IMAGE = True
    
    BEAM = False
    
    if BEAM:
        squareLength = boards['NewFiducial']['squareLength']
        markerLength = boards['NewFiducial']['markerLength']
        charucoX = boards['NewFiducial']['charucoX']
        charucoY = boards['NewFiducial']['charucoY']
        
        cameraName = 'ximea'
        
        sensor_size = np.array([27.6, 36.4]) * 1e-3
        
        f = 200 * 1e-3
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        
        board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
            
        path = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined'
        subfolderPrefixes = 'AFround'
        relDistName = cameraName + '_distCoeffs.npy'
        relIntrinsicsName = cameraName + '_intrinsics.npy'
        relFocusPrefix = 'AF'
        numRounds = 19
        
        pointRound = 18
        # Load the appropriate point estimate intrinsics, distCoeffs
        prefix = path + '/' + subfolderPrefixes + str(pointRound) + '/'
        intrinsics = np.load(prefix + relIntrinsicsName)
        dist = np.load(prefix + relDistName)
        
        image = cv2.imread(prefix + 'L-Beam/ximea26.tif', 0)
                
        trainingRounds = set(range(numRounds+1))
        testRounds = []
    
        # First, load the fixed models using loadFixedModels
        intrinsicsList, distortionList, focusList, roundList = \
            loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
        qList = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
        
        # Convert from pixel to world units
        heightConversion = sensor_size[0] / image.shape[0]
        widthConversion = sensor_size[1] / image.shape[1]
        
        qx = np.array(qList) * widthConversion
        qy = np.array(qList) * heightConversion
        
        individualPoses, boardImgPoses = theoryBeam(image, intrinsics, dist, aruco_dict, board, qList)
        
        leftR = []
        leftT = []
        rightR = []
        rightT = []
        relR = []
        relT = []
        
        for poses in individualPoses:
            # Look at the transform taking points from the left board system to the
            # right board system
            leftRot, _ = cv2.Rodrigues(poses[0][0])
            leftR.append(np.rad2deg(rotationMatrixToEulerAngles(leftRot)))
            leftT.append(poses[0][1])
            
            rightRot, _ = cv2.Rodrigues(poses[1][0])
            rightR.append(np.rad2deg(rotationMatrixToEulerAngles(rightRot)))
            rightT.append(poses[1][1])
            
            tempLeft = np.hstack([leftRot, poses[0][1]])
            leftWrtCam = np.vstack([tempLeft,[[0,0,0,1]]])
            
            tempRight = np.hstack([rightRot, poses[1][1]])
            rightWrtCam = np.vstack([tempRight,[[0,0,0,1]]])
            
            rightWrtLeft = np.matmul(np.linalg.inv(leftWrtCam), rightWrtCam)   
            
            Rot = rightWrtLeft[:-1,:-1]
            T = rightWrtLeft[:-1,-1]
            
            relR.append(np.rad2deg(rotationMatrixToEulerAngles(Rot)))
            relT.append(T)
                
        # Each column of <>R corresponds to roll, pitch, yaw
        # Each column of <>T corresponds to x, y, z
        # Each row corresponds to a different q
        leftR = np.squeeze(np.array(leftR))
        leftT = np.squeeze(np.array(leftT))
        rightR = np.squeeze(np.array(rightR))
        rightT = np.squeeze(np.array(rightT))
        relR = np.squeeze(np.array(relR))
        relT = np.squeeze(np.array(relT))
        
        names = ['left', 'right', 'relative']
        rotations = [leftR, rightR, relR]
        translations = [leftT, rightT, relT]
        
        hPrimeLeft = []
        hPrimeRight = []
        for boardImgPose in boardImgPoses:
            hPrimeLeft.append(np.array([widthConversion, heightConversion]) * boardImgPose[0])
            hPrimeRight.append(np.array([widthConversion, heightConversion]) * boardImgPose[1])
        
        # Columns are u, v and each row is a different q
        hPrimeLeft = np.squeeze(hPrimeLeft)
        hPrimeRight = np.squeeze(hPrimeRight)
        
        hPrimes = [hPrimeLeft, hPrimeRight]

        
        for i in range(2):
            plt.figure()
            plt.title('hPrimeX [m] for ' + names[i] + ' against qx')
            plt.scatter(qx, hPrimes[i][:,0], label='Measured')
            plt.xlabel('qx [m]')
            plt.ylabel('hPrimeX [m]')
            plt.legend()
            
            plt.figure()
            plt.title('hPrimeY [m] for ' + names[i] + ' against qy')
            plt.scatter(qy, hPrimes[i][:,1], label='Measured')
            plt.xlabel('qy [m]')
            plt.ylabel('hPrimeY [m]')
            plt.legend()
        
        
        for i in range(3):
            plt.figure()
            plt.title('hx [m] for ' + names[i] + ' against qx')
            plt.scatter(qx, translations[i][:,0], label='Measured')
            if i != 2:
                plt.scatter(qx, np.divide(hPrimes[i][:,0] * f, (qx - f)), label='Theory without p')
                plt.scatter(qx, np.divide(hPrimes[i][:,0] * translations[i][:,2], qx), label='Theory with p')
                plt.axvline(x=qx[-3], linestyle='dashed', label='Point Estimated q')
            plt.xlabel('qx [m]')
            plt.ylabel('hx [m]')
            plt.legend()
            
            plt.figure()
            plt.title('hy [m] for ' + names[i] + ' against qy')
            plt.scatter(qy, translations[i][:,1], label='Measured')
            if i != 2:
                plt.scatter(qy, np.divide(hPrimes[i][:,1] * f, (qy - f)), label='Theory without p')
                plt.scatter(qy, np.divide(hPrimes[i][:,1] * translations[i][:,2], qy), label='Theory with p')
                plt.axvline(x=qy[-3], linestyle='dashed', label='Point Estimated q')
            plt.xlabel('qy [m]')
            plt.ylabel('hy [m]')
            plt.legend()
            
            plt.figure()
            plt.title('p [m] for ' + names[i] + ' against qx')
            plt.scatter(qx, translations[i][:,2], label='Measured')
            plt.xlabel('qx [m]')
            plt.ylabel('p [m]')
            plt.legend()
            
            plt.figure()
            plt.title('roll [deg] for ' + names[i] + ' against qx')
            plt.scatter(qx, rotations[i][:,0], label='Measured')
            plt.xlabel('qx [m]')
            plt.ylabel('roll [deg]')
            plt.legend()
            
            plt.figure()
            plt.title('pitch [deg] for ' + names[i] + ' against qx')
            plt.scatter(qx, rotations[i][:,1], label='Measured')
            plt.xlabel('qx [m]')
            plt.ylabel('pitch [deg]')
            plt.legend()
            
            plt.figure()
            plt.title('yaw [deg] for ' + names[i] + ' against qx')
            plt.scatter(qx, rotations[i][:,2], label='Measured')
            plt.xlabel('qx [m]')
            plt.ylabel('yaw [deg]')
            plt.legend()
        
        plt.figure()
        plt.title('Relative distance between boards [m] against qx')
        plt.scatter(qx, np.linalg.norm(translations[2], axis=1), label='Measured')
        plt.legend()
        
    if SINGLE_IMAGE:
        
        ##########################
        # Charuco Board Consts   #
        ##########################
            
        squareLength = boards['TV_3']['squareLength']
        markerLength = boards['TV_3']['markerLength']
        charucoX = boards['TV_3']['charucoX']
        charucoY = boards['TV_3']['charucoY']
    
        ##########################
        # Camera Info            #
        ##########################
        cameraName = 'ximea'
        
        sensor_size = np.array([27.6, 36.4]) * 1e-3
        
        f = 200 * 1e-3
        
        ##########################
        # Graphng Setup          #
        ##########################
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        
        board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
            
        path = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined'
        subfolderPrefixes = 'AFround'
        relDistName = cameraName + '_distCoeffs.npy'
        relIntrinsicsName = cameraName + '_intrinsics.npy'
        relFocusPrefix = 'AF'
        relLbeam = 'L-Beam'
        numRounds = 19
        
        pointRound = 18
        # Load the appropriate point estimate intrinsics, distCoeffs
        prefix = path + '/' + subfolderPrefixes + str(pointRound) + '/'
        intrinsics = np.load(prefix + relIntrinsicsName)
        dist = np.load(prefix + relDistName)
        
        # Load the appropriate test images
        foldername = 'boardImages'
        filenames = filenames = glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.jpg') + \
                    glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.tif')
    
        image_list = [cv2.imread(img, 0) for img in filenames]
        
        print('Number of test images: ' + str(len(image_list)))
        print('Image shape: ', image_list[0].shape)
        
        trainingRounds = set(range(numRounds+1))
        testRounds = []
    
        # First, load the fixed models using loadFixedModels
        intrinsicsList, distortionList, focusList, roundList = \
            loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
        qList = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
        
        # Point estimate focus motor position to use is 547, focusList[0][-3]
    
        #center = intrinsics[0, 0]
        #qList = np.linspace(0.8 * center, 1.2 * center, 10).tolist()
        hx, hy, p, hPrimeX, hPrimeY, qx, qy, roll, pitch, yaw, errors = theoryFit(image_list, board, aruco_dict, sensor_size, f, intrinsics, dist, qList)
        
        # No minus sign because openCV flips the pinhole model forward
        # Now, plot each of the above quantities against q, converting to world units
        plt.figure()
        plt.title('hx as a function of qx')
        plt.scatter(qx, np.divide(hPrimeX[:,0] * f, (qx - f)), label='Theory without p')
        plt.scatter(qx, np.divide(hPrimeX[:,0] * p[:,0], qx), label='Theory with p')
        plt.axvline(x=qx[-3], linestyle='dashed', label='Point Estimated q')
        # To create error bars, need to take lower = mean - min and upper = max - mean
        plt.errorbar(qx, hx[:,0], yerr=np.array([hx[:,0] - hx[:,1], hx[:,2] - hx[:,0]]), fmt='o', color='g', label='Measured')
        poly = np.polyfit(qx, hx[:,0], 0) # constant 
        plt.plot(qx, np.polyval(poly, qx), label='Constant LS Fit')
        print('Constant LS Fit for hx: ', poly)
        plt.xlabel('qx [m]')
        plt.ylabel('hx [m]')
        plt.legend()
        
        plt.figure()
        plt.title('hy as a function of qy')
        plt.scatter(qy, np.divide(hPrimeY[:,0] * f, (qy - f)), label='Theory without p')
        plt.scatter(qy, np.divide(hPrimeY[:,0] * p[:,0], qy), label='Theory with p')
        plt.axvline(x=qy[-3], linestyle='dashed', label='Point estimated q')
        plt.errorbar(qy, hy[:,0], yerr=np.array([hy[:,0] - hy[:,1], hy[:,2] - hy[:,0]]), fmt='o', color='g', label='Measured')
        poly = np.polyfit(qy, hy[:,0], 0) # constant 
        plt.plot(qy, np.polyval(poly, qy), label='Constant LS Fit')
        print('Constant LS Fit for hy: ', poly)
        plt.xlabel('qy [m]')
        plt.ylabel('hy [m]')
        plt.legend()
    
        plt.figure()
        plt.title('p as a function of qx')
        plt.errorbar(qx, p[:,0], yerr=np.array([p[:,0] - p[:,1], p[:,2] - p[:,0]]), fmt='o')
        plt.axvline(x=qx[-3], linestyle='dashed', label='Point Estimated q')
        poly = np.polyfit(qx, p[:,0], 1)
        A = poly[0]
        plt.plot(qx, np.polyval(poly, qx), label='Linear LS Fit')
        print('Linear LS fit for p against qx: ', poly)
        plt.xlabel('qx [m]')
        plt.ylabel('p [m]')
        plt.legend()
        
        plt.figure()
        plt.title('p as a function of qy')
        plt.errorbar(qy, p[:,0], yerr=np.array([p[:,0] - p[:,1], p[:,2] - p[:,0]]), fmt='o')
        plt.axvline(x=qy[-3], linestyle='dashed', label='Point estimated q')
        poly = np.polyfit(qy, p[:,0], 1)
        plt.plot(qy, np.polyval(poly, qy), label='Linear LS Fit')
        print('Linear LS fit for p against qy: ', poly)
        plt.xlabel('qy [m]')
        plt.ylabel('p [m]')
        plt.legend()
        
        plt.figure()
        plt.title('hPrimeX as a function of qx')
        plt.axvline(x=qx[-3], linestyle='dashed', label='Point estimated q')
        plt.errorbar(qx, hPrimeX[:,0], yerr=np.array([hPrimeX[:,0] - hPrimeX[:,1], hPrimeX[:,2] - hPrimeX[:,0]]), fmt='o')
        poly = np.polyfit(qx, hPrimeX[:,0], 1)
        plt.plot(qx, np.polyval(poly, qx), label='Linear LS Fit')
        print('Linear LS fit for hPrimeX against qx: ', poly)
        plt.xlabel('qx [m]')
        plt.ylabel('hPrimeX [m]')
        plt.legend()
        
        plt.figure()
        plt.title('hPrimeY as a function of qy')
        plt.axvline(x=qy[-3], linestyle='dashed', label='Point estimated q')
        plt.errorbar(qy, hPrimeY[:,0], yerr=np.array([hPrimeY[:,0] - hPrimeY[:,1], hPrimeY[:,2] - hPrimeY[:,0]]), fmt='o')
        poly = np.polyfit(qy, hPrimeY[:,0], 1)
        plt.plot(qy, np.polyval(poly, qy), label='Linear LS Fit')
        print('Linear LS fit for hPrimeY against qy: ', poly)
        plt.xlabel('qy [m]')
        plt.ylabel('hPrimeY [m]')
        plt.legend()
    
        plt.figure()
        plt.title('Check 1/p + 1/q = 1/f')
        constraint = 1 / f - np.divide(1, p[:,0]) - np.divide(1, qx)
        plt.scatter(qx, constraint, label='Measured')
        plt.axvline(x=qx[-3], linestyle='dashed', label='Point estimated q')
        plt.plot(qx, np.zeros(len(qx)), color='r', label='Zero line')
        #poly = np.polyfit(qx, constraint, 1)
        #plt.plot(qx, np.polyval(poly, qx), label='Linear LS Fit')
        #print('Linear LS fit for 1/f - 1 / p - 1 / q against qx: ', poly)
        plt.scatter(qx, 1 / f - np.divide((1 / A + 1), qx), label='1 / f - (1 / M + 1) / q')
        predQ = f * (1 / A + 1)
        plt.axvline(x=predQ, linestyle='dashed', color='g', label='Predicted true q')
        plt.xlabel('qx [m]')
        plt.ylabel('1 / f - 1 / p -1 / q')
        plt.legend()
        
        plt.figure()
        plt.title('Estimated u vs qx')
        u = np.array(qList) * hPrimeX[:,0] + intrinsics[0,2]
        plt.scatter(qx, u, label='Measured')
        plt.axvline(x=qx[-3], linestyle='dashed', label='Point estimated q')
        plt.xlabel('qx [m]')
        plt.ylabel('Estimated u')
        
        plt.figure()
        plt.title('Estimated v vs qy')
        v = np.array(qList) * hPrimeY[:,0] + intrinsics[1,2]
        plt.scatter(qy, v, label='Measured')
        plt.axvline(x=qy[-3], linestyle='dashed', label='Point estimated q')
        plt.xlabel('qy [m]')
        plt.ylabel('Estimated v')
        
        names = ['roll', 'pitch', 'yaw']
        for i, component in enumerate([roll, pitch, yaw]):
            plt.figure()
            plt.title(names[i] + ' as a function of qx')
            plt.axvline(x=qx[-3], linestyle='dashed', label='Point estimated q')
            plt.scatter(qx, component[:,0], label='Measured')
            if i != 2:
                poly = np.polyfit(qx, component[:,0], 1)
                plt.plot(qx, np.polyval(poly, qx), label='Linear LS Fit')
                print('Linear LS fit for ' + names[i] + ' against qx: ', poly)
            else:
                poly = np.polyfit(qx, component[:,0], 2)
                plt.scatter(qx, np.polyval(poly, qx), label='Quadratic LS Fit')
                print('Quadratic LS fit for ' + names[i] + ' against qx: ', poly)
            plt.xlabel('qx [m]')
            plt.ylabel(names[i] + ' [deg]')
            plt.legend()
        
        plt.figure()
        plt.title('Re-projection Error vs qx')
        plt.scatter(qx, errors)
        plt.axvline(x=qx[-3], linestyle='dashed', label='Point estimated q')
        plt.xlabel('qx [m]')
        plt.ylabel('Re-projection Error (mean per pixel across dataset)')