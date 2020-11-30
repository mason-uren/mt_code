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
import scipy.optimize
import itertools
import cv2
import re

def scaleDown(image, fraction):
    return cv2.resize(image, (int(fraction * image.shape[1]), int(fraction * image.shape[0])))

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
    

# Given various filenames of images whose name ends in a number,
# identify what the image numbers are and the image names with no
# folder prefix or extension
def getImageNamesNumbers(filenames):
    # Split based on '/', take the last chunk. 
    # Then, split based on '\\' and take the last chunk
    # Then, split on . and take the second to last chunk
    # from this just take the last letter
    imageNames = []
    imageNumbers = []
    for file in filenames:
        noPrefix = (file.split('/')[-1].split('\\')[-1].split('.')[-2])
        
        number = re.sub('\D', '', noPrefix)
        
        imageNumbers.append(int(number))
        imageNames.append(noPrefix)
    order = np.argsort(imageNumbers)
    
    return np.array(imageNames)[order], np.array(imageNumbers)[order]
    
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

# Calculates 3x3 active Rotation Matrix given euler angles.
# Follows convention: roll first, pitch second, yaw last
def eulerAnglesToRotationMatrix(theta):
    
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
        
        
                    
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                    
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

# Constructs 2X2 active rotation matrix
def rotationMatrix2D(theta):
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def altThinLensPose(pixel2world, f, defaultIntrinsics, dist, chorners, chids, board, maxRounds=5):    
    chorners = np.squeeze(chorners)
    chids = np.squeeze(chids)
    
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    
    rnd = 0
    
    revIntrinsics = np.copy(defaultIntrinsics)
    
    qHatList = []
        
    while rnd < maxRounds:
        retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, revIntrinsics, dist)
        qHat = f * tvec[2, 0] / (tvec[2, 0] - f) 
        
        revIntrinsics[0, 0] = qHat / pixel2world # image.shape[0] / sensor_size[0]
        revIntrinsics[1,1] = revIntrinsics[0,0]
        qHatList.append(qHat)
        
        rnd += 1
    
    return np.squeeze(rvec), np.squeeze(tvec), qHatList

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

# Can either pass in grayscale or BGR images
def plotImagePoints(image, imagePoints, dim=10, color=(0, 0, 255)):
    # GRAYSCALE
    if len(np.shape(image)) == 2:
        colorImg = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        colorImg = np.copy(image)
        
    points = np.squeeze(np.round(imagePoints).astype(int))
    for point in points:
        colorImg[point[1]-dim:point[1]+dim, point[0]-dim:point[0]+dim, :] = color
    
    return colorImg
    
# Given an image with two boards in it (L-Beam), return the chorners, chids, markerCorners, markerIds
# for both boards using the split which maximizes the minimum number of corners
# detected for either board
# Can turn on graphing functionality to display the resulting optimal split
# and the detected chorners on either board, can specify a write path to save
# this image (should be full name including extension)
def findBoards(image, aruco_dict, board, verbose = False, graph=False, writePath = None, splitRange = range(4,7), parameters=cv2.aruco.DetectorParameters_create()):
    
    # Consider a vertical split at different locations across the image
    height, width = np.shape(image)
        
    splits = [round(0.1*i*width) for i in splitRange]

    # For each split try to find a board on both sides 
    # Add a buffer so that cannot detect one board as two
    buffer = width // 30
    
    maxCorners = 1 # So that way to update must have detected in both sides
    chosenSplit = -1
    
    colorImgLeft = None
    colorImgRight = None
    
    bestChorners = []
    bestChids = []
    bestMarkerCorners = []
    bestMarkerIds = []
    
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
        
        # Now, want to independently find boards in the left and right portions
        # of the image
        imageLeft = image[:, :xLeft]
        imageRight = image[:, xRight:]

        numCorners = [0, 0]
        calcorners = []
        calids = []
        detCorners = []
        markerIds = []
        
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
                    detCorners.append(markerCorners)
                    markerIds.append(ids)
                else:    
                    numCorners[isRight] = 0
                    calcorners.append([])
                    calids.append([])
            else:
                numCorners[isRight] = 0
                calcorners.append([])
                calids.append([])
        
        # Compute the minimum of numCorners, if it exceeds current best
        # save the information
        temp = min(numCorners)
        if temp > maxCorners:
            if verbose:
                print('Updating best split')
            maxCorners = temp
            chosenSplit = split

            bestChorners = calcorners
            bestChids = calids
            bestMarkerCorners = detCorners
            bestMarkerIds = markerIds
            
    if graph and chosenSplit != -1:
        colorImg = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        xLeft = chosenSplit - buffer
        xRight = chosenSplit + buffer
        
        # Plot the buffer lines
        cv2.line(colorImg, (xLeft, 0), (xLeft, height), (255, 0, 0), 5)
        cv2.line(colorImg, (xRight, 0), (xRight, height), (255, 0, 0), 5)
    
        # Plot the chorners
        for chorners in bestChorners:
            colorImg = plotImagePoints(colorImg, chorners)
            
        # Plot the marker corners
        for markerCorners in bestMarkerCorners:
            imagePoints = np.reshape(markerCorners, (-1, 2))
            colorImg = plotImagePoints(colorImg, imagePoints, 10, (0, 255, 0))
        
        cv2.imshow('Best Split, markers, and chorners', scaleDown(colorImg, 0.15))
        cv2.waitKey(0)
        
        if writePath is not None:
            if verbose:
                print('Writing results')
            cv2.imwrite(writePath, colorImg)
                        
    return bestChorners, bestChids, bestMarkerCorners, bestMarkerIds, chosenSplit

def compose4x4Extrinsics(rvec, tvec):
    if len(np.shape(tvec)) == 1:
        T = np.expand_dims(tvec, 1)
    else:
        T = tvec
    R = cv2.Rodrigues(rvec)[0]
    
    temp = np.hstack([R, T])
    return np.vstack([temp,[[0,0,0,1]]])

# Given two poses [rvec, tvec], computes the pose of the second wrt the fist
# Have used L-Beam terminology of poseLeft and poseRight to specify that
# returns the pose of the right board wrt the left (rvec, tvec).
# Also returns the positive angle between board normals and the distance 
def computeRelPose(poseLeft, poseRight):
    
    leftWrtCam = compose4x4Extrinsics(*poseLeft)
    rightWrtCam = compose4x4Extrinsics(*poseRight)
    
    # Transform taking points defined in right board coordinate
    # to left board coordinate. Equivalently (by passing in I),
    # this transformation describes the orientation of the right
    # board wrt left board.
    rightWrtLeft = np.matmul(np.linalg.inv(leftWrtCam), rightWrtCam)   
    
    Rot = rightWrtLeft[:-1,:-1]
    tvec = rightWrtLeft[:-1,-1]
    
    rvec = cv2.Rodrigues(Rot)[0]
    
    distance = np.linalg.norm(tvec)
    
    # Compute the angle between the boards via the angle between
    # the normal vectors
    # Should agree with np.linalg.norm(rvec)
    perpLeft = np.matmul(leftWrtCam[:-1,:-1], np.array([0,0,1]))
    perpLeft *= 1 / np.linalg.norm(perpLeft)
    perpRight = np.matmul(rightWrtCam[:-1,:-1], np.array([0,0,1]))
    perpRight *=  1 / np.linalg.norm(perpRight)
    angleShift = np.arccos(np.dot(perpLeft, perpRight))
    #print('Angle Shift: ' + str(np.rad2deg(angleShift)))
    degRoll = np.rad2deg(angleShift)
                
    return rvec, tvec, degRoll, distance

def predictPoseFromIntrinsics(cameraMatrix, dist, chorners, chids, board):
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    chorners = np.squeeze(chorners)
        
    retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, cameraMatrix, dist)
    
    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)
    
    return rvec, tvec

# objPoints assumed to be NX3
def projectPoints(K, dist, rot, T, objPoints):
    trans = np.squeeze(T)
    if np.shape(rot) != (3,3):
        R = cv2.Rodrigues(rot)[0]
    else:
        R = rot
    
    xCam = R @ objPoints.T + np.tile(trans, (len(objPoints), 1)).T
    xCam[0, :] /= xCam[2,:]
    xCam[1, :] /= xCam[2,:]
    xCam[2, :] /= xCam[2,:]

    rSquared = np.square(np.linalg.norm(xCam[:-1,:], axis=0))
    radialX = xCam[0,:] * (1 + dist[0,0] * rSquared + dist[0,1] * rSquared**2 + dist[0,4] * rSquared**3)
    tanX = 2 * dist[0,2] * xCam[0,:] * xCam[1,:] + dist[0,3] * (rSquared + 2 * np.square(xCam[0,:]))
    radialY = xCam[1,:] * (1 + dist[0,0] * rSquared + dist[0,1] * rSquared**2 + dist[0,4] * rSquared**3)
    tanY = 2 * dist[0,3] * xCam[0,:] * xCam[1,:] + dist[0,2] * (rSquared + 2 * np.square(xCam[1,:]))
    
    xCam[0,:] = radialX + tanX
    xCam[1,:] = radialY + tanY
    
    xCam[0,:] = K[0,0] * xCam[0,:] + K[0,2]
    xCam[1,:] = K[1,1] * xCam[1,:] + K[1,2]
    
    imgPoints = xCam[:-1,:].T

    return imgPoints

def findChorners(image, board, aruco_dict, minIds=8, parameters = cv2.aruco.DetectorParameters_create()):

    markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
    
    # if we dont find enough points skip
    if (ids is not None and len(ids) > minIds):
        ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
    else:
        ret = False
        chorners = None
        chids = None
    
    return ret, chorners, chids, markerCorners, ids


# Given a single image compute the slope M between p and q
def computeM(chorners, chids, board, pixel2world, intrinsics, dist, qList, graph = False):
    # Convert from pixel to world units
    
    qx = np.array(qList) * pixel2world
    
    intrinsicsList = []
    for q in qList:
        cameraMatrix = np.copy(intrinsics)
        cameraMatrix[0, 0] = q
        cameraMatrix[1, 1] = q
        intrinsicsList.append(cameraMatrix)
    
    p = []
        
    for j, intrinsics in enumerate(intrinsicsList):        
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
            p.append(tvec[2, 0])
    
    poly = np.polyfit(qx, p, 1)
        
    Mstar, predPoints, _ = LSM(qx, p)
    
    if graph:
        plt.figure()
        plt.title('Estimated p versus qx')
        plt.scatter(qx, p, label='Measured')
        plt.plot(qx, np.polyval(poly, qx), label=(f'Linear LS Fit: {poly[0]:.4f} q + {poly[1]:.4f}'))
        plt.plot(qx, predPoints, label=(f'p=Mq Fit: {Mstar:.4f} q'))
        plt.xlabel('qx [m]')
        plt.ylabel('p [m]')
        plt.legend()        
    
    # M = poly[0] (with intercept)
    return qx, p, poly, Mstar

# TODO: Does not work well, think about a means of improving this!
# Perhaps using the original marker size makes it unstable because of the large scale difference
# So, could use a more similarly scaled "aligned" frame to transform to and see if this helps
# Perhaps could use estimateHomography, and do so using more than just the 4 points
# Perhaps distortion is causing problems too (even though is relatively local)
# Can use board.nearestMarkerIdx[0] to get the 2 marker ids closest to origin
def findOrigin(ids, markerCorners, nearestMarkerIds, board, graph = False, image = None):
    originCoord = None
    count = 0
    for markerId in np.squeeze(nearestMarkerIds):
        # Check if markerId was detected
        findOrigin = (np.squeeze(ids) == markerId)
        if np.sum(findOrigin) != 0:
            markerIdx = np.where(findOrigin == 1)[0][0]
            # Should be the 4 corners of the marker
            markerLoc = np.squeeze(markerCorners)[markerIdx]
            
            # Get the x,y components of the corresponding points in the board
            # (so aligned) frame
            alignedPoints = board.objPoints[markerId]
            # Should have shape 4,3 and then ignore z
            alignedPoints = np.squeeze(alignedPoints)[:, :-1]
            
            alignedPoints = np.ndarray.astype(alignedPoints, np.float32)
            markerLoc = np.ndarray.astype(markerLoc, np.float32)
            
            H = cv2.getPerspectiveTransform(alignedPoints, markerLoc)
                        
            #pdb.set_trace()
            if np.abs(np.linalg.det(H)) > 1e-6:
                originProj = np.squeeze(cv2.perspectiveTransform(np.array([[[0.0,0.0]]]), H))
                if originCoord is None:
                    originCoord = originProj
                else:
                    originCoord += originProj
                count += 1
    
    if originCoord is not None:
        originCoord *= 1 / count
        
        if graph and image is not None:
            color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            color_image[int(originCoord[1])-20:int(originCoord[1])+20, int(originCoord[0])-20:int(originCoord[0])+20] = (0, 0, 1)
            cv2.imshow('Origin Plotted', cv2.resize(color_image, (int(0.15 * color_image.shape[1]), int(0.15 * color_image.shape[0]))))
            cv2.waitKey(0)

        return True, originCoord
    
    else:
        return False, None

# TODO: Assume image was undistorted before giving corners, and then should pass dist=np.zeros((1,5))
# Deprecated theoretical thin-lens based approach for initially 
# estimating all of tvec without using solvePnP directly
def estimatePoseFancy(qList, pixel2world, f, defaultIntrinsics, dist, chorners, chids, markerCorners, ids, board, useIntercept = False, nearestMarkerIds = []):
    ITERATE = False
        
    foundOrigin = False
    if len(nearestMarkerIds):
        foundOrigin, originCoord = findOrigin(ids, markerCorners, nearestMarkerIds, board)
        
    if foundOrigin:
        hPrime = pixel2world * (originCoord - defaultIntrinsics[:2, 2])
    
    # camMatList = []
    # for q in qList:
    #     cameraMatrix = np.copy(defaultIntrinsics)
    #     cameraMatrix[0, 0] = q
    #     cameraMatrix[1, 1] = q
    #     camMatList.append(cameraMatrix)
    # qx = pixel2world * np.array(qList)
    # #qy = sensor_size[1] / image_list[0].shape[1] * np.array(qList)
    
    qx, p, poly, Mstar = computeM(chorners, chids, board, pixel2world, defaultIntrinsics, dist, qList)
    
    chids = np.squeeze(chids)
    chorners = np.squeeze(chorners)
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    
    # p = []
    
    # for camMat in camMatList:
    #     retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, camMat, dist)
        
    #     R, _ = cv2.Rodrigues(rvec)
    #     #print('q: ', camMat[0,0] * sensor_size[0] / image.shape[0])
    #     #print('Resulting estimated Rotation: roll, pitch, yaw ', np.rad2deg(rotationMatrixToEulerAngles(R)))
    #     p.append(tvec[2][0])
    
    if useIntercept:
        M = poly[0]
        a0 = poly[1]
    else:
        M = Mstar
    
    # if useIntercept:
    #     poly = np.polyfit(qx, p, 1)
    
    #     M = poly[0]
    #     a0 = poly[1]
    # else:
    #     # M, _, _ = LSM(qx, p)
    #     M = Mstar
    
    qHat = f * (1 / M + 1)
    
    # Ignore that a0 exists, alternatively could assume that p = Mq + a0
    # and still apply thin lens to solve
    
    # if not useIntercept:    
    #     qHat = f * (1 / M + 1)
    
    # else:
    #     # Assume p = Mq + a0 and plug into 1/p+1/q = 1/f
    #     num = -a0 / f + M + 1
    #     disc = np.sqrt((a0/f - M - 1)**2 + 4 * M *a0 / f)
    #     den = 2 / f * M
        
    #     root1 = (num + disc) / den
    #     root2 = (num - disc) / den

    #     #For root to be valid must be at least as large as f
    #     qHat = None
    #     distance = math.inf
    #     theoreticalQ = f / M + f
    #     for root in [root1, root2]:
    #         temp = np.abs(root - theoreticalQ)
    #         if root > f and temp < distance:
    #             dist = temp
    #             qHat = root 
        
    revIntrinsics = np.copy(defaultIntrinsics)
    fEff = qHat / pixel2world
    revIntrinsics[0,0] = fEff
    revIntrinsics[1,1] = fEff
    
    if foundOrigin:
        tvec = np.zeros(3)
        tvec[:2] = M * hPrime
        tvec[2] = M * qHat
        print('prior tvec: ', tvec)
        tvec = np.expand_dims(tvec, 1)
        # Ultimately shift to this but also requires an rvec initial guess if have tvec initial guess
        #retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, revIntrinsics, dist, None, tvec, True)
        retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, revIntrinsics, dist)
        print('post tvec: ', tvec)
    else:
        print('Used normal solvePnP')
        # TODO: Should ultimately shift this to be None and undistort beforehand
        retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, revIntrinsics, dist)
    
    print('prior M: ', M)
    print('post M: ', tvec[2, 0] / qHat)
    R, _ = cv2.Rodrigues(rvec)
    print('Rotation (roll, pitch, yaw): ', np.rad2deg(rotationMatrixToEulerAngles(R)))
    
    Mlist = [M, tvec[2, 0] / qHat]
    qHatList = [qHat]

    maxRounds = 5
    
    rnd = 0
    if ITERATE:
        while rnd < maxRounds:
            qHat = f / Mlist[-1] + f
            qHatList.append(qHat)
            revIntrinsics = np.copy(defaultIntrinsics)
            fEff = qHat / pixel2world
            revIntrinsics[0,0] = fEff
            revIntrinsics[1,1] = fEff           
            retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, revIntrinsics, dist)
            Mlist.append(tvec[2, 0] / qHat)
            print('Round is: ' + str(rnd))
            print('tvec: ', tvec)
            rnd += 1
    
    return rvec, tvec, fEff, Mlist
    
def LSM(qVec, pVec):
    if len(np.shape(qVec)) != 2:
        A = np.expand_dims(qVec, axis=1)
    else:
        A = np.copy(qVec)
        
    if len(np.shape(qVec)) != 2:
        y = np.expand_dims(pVec, axis=1)
    else:
        y = np.copy(pVec)
    
    Mstar = (np.linalg.inv(A.T @ A) @ A.T @ y).item()
    predPoints = Mstar * qVec
    errors = pVec - predPoints
    return Mstar, predPoints, errors

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
    
    return rms, cameraMatrix, distCoeffs, rvecs, tvecs

# point = (x,y), line = (nx, ny, d0) st. nx x' + ny y' = d0 for x',y' on line
def projPointLine(point, line, graph=False):
    nx, ny, d0 = line
    x, y = point
    
    basis = np.array([-ny / nx, 1])
    particularSolution = np.array([0, d0 / ny])
    shiftedPoint = point - particularSolution
    
    projected = np.dot(basis, shiftedPoint) / np.dot(basis, basis) * basis
        
    projected += particularSolution
    # yPrime = (y - ny / nx * x + ny * d0 / nx**2) / (ny**2 / nx**2 + 1)
    # xPrime = (d0 - ny * yPrime) / nx
    
    if graph:
        plt.figure()
        xSpacing = np.linspace(x - 1000, x + 1000, 2000) 
        plt.plot(xSpacing, (d0 - nx * xSpacing) / ny, label='Line')
        b = projected[1] - ny / nx * projected[0]
        plt.plot(xSpacing, ny / nx * xSpacing + b, label='Perpindicular')
        plt.scatter(projected[0], projected[1], label='Projection')
        plt.scatter(x, y, label='Original')
        plt.legend()
        plt.axis('square')
    
    return projected

def thinLensEstimateBeam(image_list, defaultIntrinsics, dist, aruco_dict, board, pixel2world, f, verbose=False):
    estimatedRolls = []
    estimatedDistances = []
    estimatedTranslations = []
    
    for i, image in enumerate(image_list):    
        if verbose:
            print('On Image: ' + str(i))
        bestChorners, bestChids, _, _, chosenSplit = findBoards(image, aruco_dict, board, False, False)
        
        numImagesSkipped = 0
        
        # Need to ensure that found both boards and that have a reasonable
        # number of chorners to work with
        if chosenSplit != -1 and len(bestChorners[0]) >= 4 and len(bestChorners[1]) >= 4:
            poses = []
            Ms = []
        
            for j in range(2):
                chorners = bestChorners[j]
                chids = bestChids[j]
                
                rvec, tvec, qHatList = altThinLensPose(pixel2world, f, defaultIntrinsics, dist, chorners, chids, board)
                Ms.append(tvec[2] / qHatList[-1])
                
                poses.append([rvec, tvec])
            
            rvec, tvec, degRoll, distance = computeRelPose(*poses)
            
            estimatedRolls.append(degRoll)
            estimatedDistances.append(distance)
            estimatedTranslations.append(tvec)  
            
            if verbose:
                print('Rotating the left board by ' + str(degRoll) + ' deg gives the right board orientation')
                print('Distance between the left and right boards is ' + str(distance) + ' m')
        
        else:
            print('Failed to find boards: skipping image ' + str(i))
            numImagesSkipped += 1
    
    # Should print using the altThinLensPose (and the default intrinsics from round AFround3 or 8-31)
    # Sample size: 20
    # Mean of rotation (deg): 0.575096362445919
    # Standard Deviation of rotation (deg): 0.33213241708997654
    # Mean of Distance (m): 0.32575182019076865
    # Standard Deviation of Distance (m): 0.0006502066540464371
    if verbose:
        print('Sample size: ' + str(len(image_list) - numImagesSkipped))
        print('Mean of rotation (deg): ' + str(np.mean(estimatedRolls)))
        print('Standard Deviation of rotation (deg): ' + str(np.std(estimatedRolls)))
        print('Mean of Distance (m): ' + str(np.mean(estimatedDistances)))
        print('Standard Deviation of Distance (m): ' + str(np.std(estimatedDistances)))

    return estimatedDistances, estimatedRolls, estimatedTranslations
    

# Compute mean reprojection error over a dataset given the calibration results
# Assumes blurred already

def error_for_dataset(chornersList, chidsList, intrinsics, dist, useThinLens=False):
    global f, pixel2world, board
    image_errors = []
    rotations = []
    translations = []
    
    revIntrinsics = np.copy(intrinsics)
    
    for chorners, chids in zip(chornersList, chidsList):
        if useThinLens:
            rvec, tvec, qHatList = altThinLensPose(pixel2world, f, revIntrinsics, dist, chorners, chids, board)    
            revIntrinsics[0,0] = qHatList[-1] / pixel2world
            revIntrinsics[1,1] = revIntrinsics[0,0]
        else:
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, revIntrinsics, dist, None, None, useExtrinsicGuess=False)
        # Compute the reprojection error
        R, _ = cv2.Rodrigues(rvec)
        mean_error = charuco_reprojection_error(board,chorners,chids,R,tvec,revIntrinsics,dist)
        image_errors.append(mean_error)
        roll, pitch, yaw = np.rad2deg(rotationMatrixToEulerAngles(R))
        rotations.append([wrapAngle(roll, 0, 2*180, False), pitch, yaw])
        translations.append(tvec)

    rotations = np.squeeze(rotations)
    translations = np.squeeze(translations)
    return image_errors, np.mean(image_errors), rotations, translations