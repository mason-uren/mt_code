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
from fitVaryingModel import loadFixedModels
import scipy.optimize
import itertools

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

# TODO: Does not work well, think about a means of improving this!
# Perhaps could use estimateHomography, and do so using more than just the 4 points
# Possible that have bad measurement of marker size
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
            cv2.imshow('Origin Plotted', cv2.resize(color_image, (int(0.15 * color_image.shape[0]), int(0.15 * color_image.shape[0]))))
            cv2.waitKey(0)

        return True, originCoord
    
    else:
        return False, None


# Assumes image was undistorted before giving corners
def estimatePoseFancy(qList, sensor_size, image, f, defaultIntrinsics, chorners, chids, markerCorners, ids, board, nearestMarkerIds = []):
    pixel2world = sensor_size[0] / image.shape[0]
    
    foundOrigin = False
    if len(nearestMarkerIds):
        foundOrigin, originCoord = findOrigin(ids, markerCorners, nearestMarkerIds, board)
    
    if foundOrigin:
        hPrime = pixel2world * (originCoord - defaultIntrinsics[:2, 2])
    
    camMatList = []
    for q in qList:
        cameraMatrix = np.copy(intrinsics)
        cameraMatrix[0, 0] = q
        cameraMatrix[1, 1] = q
        camMatList.append(cameraMatrix)
    qx = pixel2world * np.array(qList)
    #qy = sensor_size[1] / image_list[0].shape[1] * np.array(qList)
    
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    chorners = np.squeeze(chorners)
    
    p = []
    
    for camMat in camMatList:
        retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, camMat, None)
        
        R, _ = cv2.Rodrigues(rvec)
        #print('q: ', camMat[0,0] * sensor_size[0] / image.shape[0])
        #print('Resulting estimated Rotation: roll, pitch, yaw ', np.rad2deg(rotationMatrixToEulerAngles(R)))
        p.append(tvec[2])

    poly = np.polyfit(qx, p, 1)
    
    M = poly[0]
    print('Computed M was: ', M)
    
    qHat = f * (1 / M + 1)
    
    revIntrinsics = np.copy(defaultIntrinsics)
    fEff = qHat / pixel2world
    revIntrinsics[0,0] = fEff
    revIntrinsics[1,1] = fEff
    
    if foundOrigin:
        tvec = np.zeros(3)
        tvec[:2] = M * hPrime
        tvec[2] = M * qHat
        print('prior tvec: ', tvec)
        retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, revIntrinsics, None, None, tvec, True)
    
    else:
        retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, revIntrinsics, None)
    
    return rvec, tvec, fEff, M


def optimizeM(M, scaledBoard, unscaledBoard):
    return np.sqrt(np.mean(np.linalg.norm(M * scaledBoard - unscaledBoard, axis=1)**2))

# Both openCV and optimizeM approach give consistent answers as add u,v offset to detected chorners
# rvec estimated by openCV does change a bit which is expected since right now have not handled distortion

# openCV and optimizeM are not giving consistent answers when apply 2D rotation using
#offset = np.expand_dims(np.tile(chorners[0], chorners.shape[:-1]), axis=1)
#modifiedChorners = np.copy(chorners) - offset
#modifiedChorners = np.expand_dims((rotation @ np.squeeze(modifiedChorners).T).T, axis=1)
#modifiedChorners = modifiedChorners + offset
# optimizeM is much more impacted than openCV however


# Given a starting set of board chessboard corners in board frame applies a 
# transformation of form alpha * R @ (x + T) where x is the original board 
# corner in board frame
# shape(T) = (3,) T in meters
# roll, pitch, yaw in deg
def transformBoard(chessboardCorners, roll, pitch, yaw, T, alpha=1):
    modCorners = np.copy(chessboardCorners)
    #squaresX, squaresY = board.getChessboardSize()
    #squareLength = alpha * board.getSquareLength()
    #markerLength *= alpha * board.getMarkerLength()
    
    #modBoard = cv2.aruco.CharucoBoard_create(squaresX, squaresY, squareLength, markerLength, aruco_dict)
    modCorners += T
    R = eulerAnglesToRotationMatrix(np.deg2rad([roll, pitch, yaw]))
    modCorners = (R @ modCorners.T).T
    return alpha * modCorners
    
if __name__ == '__main__':
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
    
    trainingRounds = set(range(numRounds+1))
    testRounds = []

    # First, load the fixed models using loadFixedModels
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
    qList = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
    
    # Load the appropriate test images
    foldername = 'ximeaData'
    filenames = filenames = glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.jpg') + \
                glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.tif')
    
    image_list = [cv2.imread(img, 0) for img in filenames]
    
    # TODO: For now, only look at first image
    image_list = [image_list[0]]
    
    print('Number of test images: ' + str(len(image_list)))
    
    rolls = [30 * i for i in range(12)]
    pitchs = [30 * i for i in range(12)]
    yaws = [30 * i for i in range(12)]
    #offset = [squareLength * i for i in range(0, 50, 10)]
    #Ts = np.array(list(itertools.product(offset, offset, [0] * len(offset)))) # N x 3
    Ts = [np.zeros(3)]
    #transforms = list(itertools.product(rolls, pitchs, yaws, Ts))
    transforms = [(0, 0, 0, np.zeros(3), i) for i in range(1, 11)]
    
    camMatList = []
    for q in qList:
        cameraMatrix = np.copy(intrinsics)
        cameraMatrix[0, 0] = q
        cameraMatrix[1, 1] = q
        camMatList.append(cameraMatrix)
    qx = sensor_size[0] / image_list[0].shape[0] * np.array(qList)
    qy = sensor_size[1] / image_list[0].shape[1] * np.array(qList)

    for image in image_list:
        Ms = []
        # Estimate the pose of the charuco board in the image
        parameters = cv2.aruco.DetectorParameters_create()

        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            
            objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
            chorners = np.squeeze(chorners)
            
            for i, transform in enumerate(transforms):                
                if len(transforms) > 100 and i % int(len(transforms) / 100) == 0:
                    print('\nProgress: ' + str(i / len(transforms)))
                #print('Rotated board frame by: roll, pitch yaw of ', transform[0:3])
                #print('Translated board frame by: x, y, z', transform[-1])
                
                transformedPoints = transformBoard(objectPoints, *transform)
                
                p = []
                
                for camMat in camMatList:

                    retval, rvec, tvec = cv2.solvePnP(transformedPoints, chorners, camMat, dist)
                    
                    R, _ = cv2.Rodrigues(rvec)
                    #print('q: ', camMat[0,0] * sensor_size[0] / image.shape[0])
                    #print('Resulting estimated Rotation: roll, pitch, yaw ', np.rad2deg(rotationMatrixToEulerAngles(R)))
                    p.append(tvec[2])
            
                poly = np.polyfit(qx, p, 1)
                
                M = poly[0]
                #print('OpenCV computed M: ', M)        
                Ms.append(M)
                


# if __name__ == '__main__':
#     squareLength = boards['TV_3']['squareLength']
#     markerLength = boards['TV_3']['markerLength']
#     charucoX = boards['TV_3']['charucoX']
#     charucoY = boards['TV_3']['charucoY']
    
#     ##########################
#     # Camera Info            #
#     ##########################
#     cameraName = 'ximea'
    
#     sensor_size = np.array([27.6, 36.4]) * 1e-3
    
#     f = 200 * 1e-3
    
#     ##########################
#     # Graphng Setup          #
#     ##########################
#     aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    
#     board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
        
#     path = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined'
#     subfolderPrefixes = 'AFround'
#     relDistName = cameraName + '_distCoeffs.npy'
#     relIntrinsicsName = cameraName + '_intrinsics.npy'
#     relFocusPrefix = 'AF'
#     relLbeam = 'L-Beam'
#     numRounds = 19
    
#     pointRound = 18
#     # Load the appropriate point estimate intrinsics, distCoeffs
#     prefix = path + '/' + subfolderPrefixes + str(pointRound) + '/'
#     intrinsics = np.load(prefix + relIntrinsicsName)
#     dist = np.load(prefix + relDistName)
    
#     trainingRounds = set(range(numRounds+1))
#     testRounds = []

#     # First, load the fixed models using loadFixedModels
#     intrinsicsList, distortionList, focusList, roundList = \
#         loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
#     qList = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
    
#     # Load the appropriate test images
#     foldername = 'ximeaData'
#     filenames = filenames = glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.jpg') + \
#                 glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.tif')
    
#     image_list = [cv2.imread(img, 0) for img in filenames]
    
#     print('Number of test images: ' + str(len(image_list)))
    
#     for image in image_list:
#             # Estimate the pose of the charuco board in the image
#             parameters = cv2.aruco.DetectorParameters_create()
    
#             markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
            
#             # if we dont find enough points skip
#             if (ids is not None and len(ids) > 8):
                
#                 # Construct N X 2 matrix where each row is a u, v shift to apply to the detected chorners
#                 shifts = np.zeros((10, 2))
#                 shifts[:, 0] = np.linspace(-3000, 3000, 10)
#                 shifts[:, 1] = np.linspace(-3000, 3000, 10)
                
#                 rotations = [rotationMatrix(30*i) for i in range(12)]
                
#                 #for j in range(shifts.shape[0]):
#                 for rotation in rotations:
#                     ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                    
#                     #offset = np.expand_dims(np.tile(shifts[j, :], chorners.shape[:-1]), axis=1)
#                     #modifiedChorners = np.copy(chorners) - offset
                    
#                     offset = np.expand_dims(np.tile(chorners[0], chorners.shape[:-1]), axis=1)
#                     modifiedChorners = np.copy(chorners) - offset
                    
#                     modifiedChorners = np.expand_dims((rotation @ np.squeeze(modifiedChorners).T).T, axis=1)
#                     modifiedChorners = modifiedChorners + offset
                    
#                     # Should function as if no distortion, apply undistort once at start
#                     retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(modifiedChorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
                    
#                     # First, identify the corresponding chessboard corners and convert to pixel units
#                     #alignedPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))[:, :-1] * image.shape[0] / sensor_size[0]
#                     #imagePoints = np.squeeze(chorners)
                
#                     # TODO: I suspect that my homography transformation is incorrect. If are only rotating the board about its origin, 
#                     # then the origin of the board
#                     # should not change in the image
                
#                     R, _ = cv2.Rodrigues(rvec)
                    
#                     print('Rotation: ', np.rad2deg(rotationMatrixToEulerAngles(R)))
                    
#                     Rboard = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
                    
#                     # Passively rotate from board frame to camera frame using R, then apply Rboard to passively rotate from camera frame to board norm
#                     # aligned with optical axis frame. Rboard R is passive so R.T Rboard.T = R.T Rboard for active
                    
#                     # Should have 3 rows and each column is [xi, yi, 1]
#                     rotatedPoints = np.vstack([np.squeeze(modifiedChorners).T, np.ones(len(modifiedChorners))])
                    
#                     #alignedPoints = intrinsics @ R.T @ Rboard @ np.linalg.inv(intrinsics) @ rotatedPoints
                    
#                     objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    
#                     #alignedPoints = intrinsics @ R.T @ Rboard @ objectPoints
                    
#                     # R described passive rotation from board to camera frame, so R.T (-1 * rvec) describes active rotation from board to camera frame
#                     alignedPoints, _ = cv2.projectPoints(objectPoints, -1 * rvec, np.array([[0.0,0.0,0.0]]), intrinsics, dist)                                
#                     #alignedPoints = intrsinics @ R.T @ Rboard @ board.chessboardCorners[]
#                     #alignedPoints[0, :] = np.divide(alignedPoints[0, :], alignedPoints[2, :])
#                     #alignedPoints[1, :] = np.divide(alignedPoints[1, :], alignedPoints[2, :])
#                     #alignedPoints[2, :] = 1
                                    
#                     #modifiedChorners = np.expand_dims(alignedPoints[:-1, :].T, 1)
                    
#                     # Should do this plotting after compute the scale value
#                     # highlightImage = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
#                     # for row in range(alignedPoints.shape[0]):
#                     #     y = int(alignedPoints[row, :, 1])
#                     #     x = int(alignedPoints[row, :, 0])
#                     #     highlightImage[y - 20: y + 20, x - 20: x + 20] = (0, 0, 255)
                    
#                     # cv2.imshow('highlightImage', cv2.resize(highlightImage, (int(0.15 * highlightImage.shape[1]), int(0.15 * highlightImage.shape[0]))))
#                     # cv2.waitKey(0)
    
    
#                     # Position both so that upper left is at 0, 0
#                     scaledBoard = np.squeeze(modifiedChorners)
#                     scaledBoard -= scaledBoard[0]
                    
#                     objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
#                     # Rotate into camera orientation
#                     objectPoints = (np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]) @ objectPoints.T).T
#                     unscaledBoard = objectPoints[:, :-1] * image.shape[0] / sensor_size[0]
#                     unscaledBoard -= unscaledBoard[0]
                    
#                     res = scipy.optimize.minimize_scalar(optimizeM, args=(scaledBoard, unscaledBoard))
                    
#                     print('Optimization success?', res.success)
#                     print('Optimization solution: M = ', res.x)
#                     print('Resulting average reprojection error: ', res.fun)
                    
#                     # TODO: Need to actually incorporate this into current code, otherwise is using the perfect image each time
#                     # Convert from pixel to world units
#                     heightConversion = sensor_size[0] / image_list[0].shape[0]
#                     widthConversion = sensor_size[1] / image_list[0].shape[1]
                    
#                     qx = np.array(qList) * widthConversion
#                     qy = np.array(qList) * heightConversion
                    
#                     intrinsicsList = []
#                     for q in qList:
#                         cameraMatrix = np.copy(intrinsics)
#                         cameraMatrix[0, 0] = q
#                         cameraMatrix[1, 1] = q
#                         intrinsicsList.append(cameraMatrix)
                    
#                     p = []
                    
#                     for j, camMat in enumerate(intrinsicsList):
#                         print('Progress: ' + str(j / len(intrinsicsList)))

#                         retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(modifiedChorners, chids, board, camMat, dist, None, None, useExtrinsicGuess=False)
                            
#                         p.append(tvec[2])
                    
#                     poly = np.polyfit(qx, p, 1)
                    
#                     print('OpenCV computed M: ', poly[0])                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
#                     #qx, p, poly = computeM(image, board, aruco_dict, sensor_size, f, intrinsics, dist, qList)
#                     #print('openCV M: ' + str(poly[0]))
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
#                 #np.mean(np.divide(unscaledBoard, scaledBoard))
#                 #np.std(np.divide(unscaledBoard, scaledBoard))
                
#                 # Consider what the expected signs for the components of T are
#                 # Can use the resulting R from the appropriate solution to rotate
#                 # the object points
#                 # alignedPoint = H imagePoint 
                
#                 # H, mask = cv2.findHomography(imagePoints, alignedPoints)
                
#                 # num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, intrinsics)
                
#                 # solution = []
#                 # for i in range(num):
#                 #     if Ts[i][0] > 0 and Ts[i][1] > 0 and Ts[i][2] > 0:
#                 #         solution.append(Rs[i])
#                 #         solution.append(Ts[i])
                
#                 # Then, can apply
                
                
#                 # # Extract the x and y and ignore z
#                 # # TODO: Throw in sclaing factor here and see how impact resulting H
#                 #alignedPoints = objectPoints[:, :, :-1]
                
#                 # # Now, find homography H. Can play with different methods
#                 #H, mask = cv2.findHomography(chorners, alignedPoints)
                
#                 # H = cv2.getPerspectiveTransform(chorners[:4, :, :], alignedPoints[:4, :, :])
                
#                 #print(H)
#                 #print(np.linalg.det(H))
                
                
                
#                 # 1, numPoints, 2
#                 # inputPoints = np.expand_dims(rotatedPoints[:-1, :].T, 0)
#                 # outputPoints = np.squeeze(cv2.perspectiveTransform(inputPoints, H))
                
#                 # # Apply the homography to rotate the image
#                 # highlightImage = np.zeros(image.shape)
                
#                 # for row in range(outputPoints.shape[0]):
#                 #     y = int(outputPoints[row, 1])
#                 #     x = int(outputPoints[row, 0])
#                 #     highlightImage[y - 20: y + 20, x - 20: x + 20] = 1
                
#                 # cv2.imshow('highlightImage', cv2.resize(highlightImage, (int(0.15 * highlightImage.shape[1]), int(0.15 * highlightImage.shape[0]))))
#                 # cv2.waitKey(0)
    
    
    
    
