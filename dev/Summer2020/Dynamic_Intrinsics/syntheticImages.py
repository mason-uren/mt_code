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
import scipy.optimize as opt
import itertools

def scaleDown(image, fraction):
    return cv2.resize(image, (int(fraction * image.shape[1]), int(fraction * image.shape[0])))


# objPoints assumed to be NX3
def projectPoints(K, dist, R, T, objPoints):
    xCam = R @ objPoints.T + np.tile(T, (len(objPoints), 1)).T
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

# Given undistorted image points (pixel units), return distorted pixel positions
def distortPoints(K, dist, imgPoints):
    dstPoints = np.copy(imgPoints)
    
    # Normalize
    dstPoints[:,0] -= K[0,2]
    dstPoints[:,1] -= K[1,2]
    dstPoints[:,0] *= 1/K[0,0]
    dstPoints[:,1] *= 1/K[1,1]
        
    # Distort
    rSquared = np.square(np.linalg.norm(dstPoints, axis=1))
    radialX = dstPoints[:,0] * (1 + dist[0,0] * rSquared + dist[0,1] * rSquared**2 + dist[0,4] * rSquared**3)
    tanX = 2 * dist[0,2] * dstPoints[:,0] * dstPoints[:,1] + dist[0,3] * (rSquared + 2 * np.square(dstPoints[:,0]))
    radialY = dstPoints[:,1] * (1 + dist[0,0] * rSquared + dist[0,1] * rSquared**2 + dist[0,4] * rSquared**3)
    tanY = 2 * dist[0,3] * dstPoints[:,0] * dstPoints[:,1] + dist[0,2] * (rSquared + 2 * np.square(dstPoints[:,1]))
    
    dstPoints[:,0] = radialX + tanX
    dstPoints[:,1] = radialY + tanY
    
    # Un-normalize
    dstPoints[:,0] = K[0,0] * dstPoints[:,0] + K[0,2]
    dstPoints[:,1] = K[1,1] * dstPoints[:,1] + K[1,2]

    return dstPoints

# # Input: K, dist, R, T, template image of board occupying the full image
# # Output: mapx, mapy. For every point in input image mapx indicates the new x-coordinate in output image, similarly for mapy. 
# def mapInput(K, dist, R, T, points, boardPoints, boardImg):
#     print('started mapping')
#     #points = np.array(list(itertools.product(list(range(boardImg.shape[0])), list(range(boardImg.shape[1])), [0])))
#     #points = points.astype(np.float64)
    
#     # 3 X N
#     xCam = R @ boardPoints.T + np.tile(T, (len(boardPoints), 1)).T
#     xCam[0, :] /= xCam[2,:]
#     xCam[1, :] /= xCam[2,:]
#     xCam[2, :] /= xCam[2,:]

#     rSquared = np.square(np.linalg.norm(xCam[:-1,:], axis=0))
#     radialX = xCam[0,:] * (1 + dist[0,0] * rSquared + dist[0,1] * rSquared**2 + dist[0,4] * rSquared**3)
#     tanX = 2 * dist[0,2] * xCam[0,:] * xCam[1,:] + dist[0,3] * (rSquared + 2 * np.square(xCam[0,:]))
#     radialY = xCam[1,:] * (1 + dist[0,0] * rSquared + dist[0,1] * rSquared**2 + dist[0,4] * rSquared**3)
#     tanY = 2 * dist[0,3] * xCam[0,:] * xCam[1,:] + dist[0,2] * (rSquared + 2 * np.square(xCam[1,:]))
    
#     xCam[0,:] = radialX + tanX
#     xCam[1,:] = radialY + tanY
    
#     xCam[0,:] = K[0,0] * xCam[0,:] + K[0,2]
#     xCam[1,:] = K[1,1] * xCam[1,:] + K[1,2]
    
#     outputPoints = xCam[:-1,:].T
    
#     #outputPoints = cv2.projectPoints(points, R, T, K, dist)
#     print('finished projecting')
    
#     mapu = np.zeros(boardImg.shape)
#     mapv = np.zeros(boardImg.shape)
    
#     for i, point in enumerate(points):
#         mapu[point[1], point[0]] = outputPoints[i,0]
#         mapv[point[1], point[0]] = outputPoints[i,1]
   
#     #mapu = np.reshape(outputPoints[:,0], (boardImg.shape[0], boardImg.shape[1]))
#     #mapv = np.reshape(outputPoints[:,1], (boardImg.shape[0], boardImg.shape[1]))

#     return outputPoints, mapu, mapv

def mapInput(K, dist, R, T, points, Kprime, imageShape):
    homPoints = np.ones((len(points),3))
    homPoints[:,0] = points[:,0]
    homPoints[:,1] = points[:,1]
    boardPoints = (Kprime @ homPoints.T).T
    boardPoints[:,2] = 0
    
    imgPoints = projectPoints(K, dist, R, T, boardPoints)
        
    print('finished projecting')
    
    mapu = imgPoints[:,0].reshape(imageShape)
    mapv = imgPoints[:,1].reshape(imageShape)
    
    return imgPoints, mapu, mapv

def mapBoardPixelToBoardWorld(points, Kprime):
    homPoints = np.ones((len(points),3))
    homPoints[:,0] = points[:,0]
    homPoints[:,1] = points[:,1]
    boardPoints = (Kprime @ homPoints.T).T
    boardPoints[:,2] = 0
    
    return boardPoints
    
if __name__ == '__main__':        
    #########################
    # Camera Info           #
    #########################
    f = 200 * 1e-3 / 1.0372608996079156

    cameraName = 'ximea'
            
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    imageShape = (6004, 7920)
    
    ##########################
    # Charuco Board Consts   #
    ##########################
    
    squareLength = boards['TV_3']['squareLength']
    markerLength = boards['TV_3']['markerLength']
    charucoX = boards['TV_3']['charucoX']
    charucoY = boards['TV_3']['charucoY']
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    
    board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

    # What fronto-parrallel, undistorted board would look like in image at given scale without worrying about cutoff
    #scale = 25
    #width = int(imageShape[1] / sensor_size[1] * squareLength * charucoX / scale)
    #height = int(imageShape[0] / sensor_size[0] * squareLength * charucoY / scale)
    width = squareLength * charucoX
    height = squareLength * charucoY
    resolution = 50 # How many points per one dimension of square
    #xChunks = np.linspace(0, width, resolution * charucoX, False)
    #yChunks = np.linspace(0, height, resolution * charucoY, False)[::-1] # Reverse because origin shift and orientation flip
    boardImg = board.draw((resolution * charucoX, resolution * charucoY))
    
    # Pixels' real-world points in board frame are associated with the pixel bottom left corner
    #boardPoints = np.array(list(itertools.product(yChunks, xChunks)))
    #boardPoints = boardPoints.reshape((len(yChunks), len(xChunks), 2))
                                      
    path = 'C:/Users/aofeldman/Desktop/testCollection8-31'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 6
    
    trainingRounds = set(range(numRounds+1))
    testRounds = []
        
    # First, load the fixed models using loadFixedModels
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
        
    ind = 2
    AFround = roundList[0][ind]
    K = intrinsicsList[0][ind]
    
    dist = distortionList[0][ind]
        
    foldername = path + '/' + subfolderPrefixes + str(AFround)

    # Load in all images for the given focus setting
    filenames = glob.glob(foldername + '/' + '*.tif')
    
    imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
    filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
    imageNumbers = np.sort(imageNumbers)
    
    # Read in and mildly blur images
    image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
    
    imageIndex = -1

    testImage = image_list[imageIndex]

    rvecs = np.load(path + '/' + subfolderPrefixes + str(AFround) + '/' + 'ximea_rvecs.npy')
    tvecs = np.load(path + '/' + subfolderPrefixes + str(AFround) + '/' + 'ximea_tvecs.npy')
    
    R, _ = cv2.Rodrigues(rvecs[imageIndex])
    T = np.squeeze(tvecs[imageIndex])
    
    Kprime = np.array([[squareLength / resolution, 0, 0], 
                       [0, -squareLength/resolution, squareLength/resolution * (len(boardImg)-1)],
                       [0, 0, 1]])
    
    # Nx2 array where each row corresponds to given v, u. Inner iteration over columns and outer iteration over row
    #points = np.array(list(itertools.product(list(range(boardImg.shape[0])), list(range(boardImg.shape[1])))))
    # # Swap so places u,v
    # temp = points[:,0]
    # points[:,0] = points[:,1]
    # points[:,1] = temp
    points = np.array([[u,v] for v in range(boardImg.shape[0]) for u in range(boardImg.shape[1])])
    
    homPoints = np.ones((len(points),3))
    homPoints[:,0] = points[:,0]
    homPoints[:,1] = points[:,1]
    boardPoints = (Kprime @ homPoints.T).T
    boardPoints[:,2] = 0
    
    # expandedBoard = np.zeros(imageShape).astype(np.float32)
    # expandedBoard[:boardImg.shape[0], :boardImg.shape[1]] = boardImg
    
    # expandedPoints = np.array([[i,j] for j in range(imageShape[0]) for i in range(imageShape[1])]).astype(np.float32)
    
    # # TODO: Kprime must be shifted since now using the expandedBoard, create a function to do this
    # # imgPoints, mapu, mapv = mapInput(K, np.zeros((1,5)), R, T, expandedPoints, Kprime, imageShape)
    
    # # directImg = cv2.remap(expandedBoard, mapu, mapv, cv2.INTER_LINEAR, None, cv2.BORDER_CONSTANT, 0)
    
    # # cv2.imshow('directImg', scaleDown(directImg, 0.15))
    # # cv2.waitKey(0)
    
    H = K @ np.hstack([R[:, :-1], np.expand_dims(T, axis=1)]) @ Kprime
    H *= 1 / H[-1, -1]
    
    imagePoints = projectPoints(K, dist, R, T, boardPoints)
    
    imagePoints = np.squeeze(imagePoints)
    
    # Apply the homography to the points without distorting
    mappedPoints = (H @ homPoints.T).T
    mappedPoints[:,0] = np.divide(mappedPoints[:,0], mappedPoints[:,2])
    mappedPoints[:,1] = np.divide(mappedPoints[:,1], mappedPoints[:,2])
    mappedPoints[:,2] = 1
    
    for i, arr in enumerate([mappedPoints, distortPoints(K, dist, mappedPoints)]):
        image = np.zeros(imageShape)    
        for point in mappedPoints:
            v = int(round(point[1]))
            u = int(round(point[0]))
            
            if v >= 0 and v < imageShape[0] and u >= 0 and u < imageShape[1]:
                image[v-20:v+20, u-20:u+20] = 255
        cv2.imshow('distorted' * i + 'points', scaleDown(image, 0.15))
    
    dst = cv2.warpPerspective(boardImg, H, (imageShape[1], imageShape[0]), None, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, 200)
    
    cv2.imshow('undistorted', scaleDown(dst, 0.15))
    #cv2.waitKey(0)
    
    # Now, construct the distorted image
    dstPoints = np.array([[i,j] for j in range(imageShape[0]) for i in range(imageShape[1])]).astype(np.float32)
    
    dstPoints = distortPoints(K, dist, dstPoints)
    
    print('Finished outlining points')
    
    mapu = dstPoints[:,0].reshape(imageShape)
    mapv = dstPoints[:,1].reshape(imageShape)
    
    print('Starting remap')
    distortedImg = cv2.remap(dst, mapu, mapv, cv2.INTER_LINEAR, None, cv2.BORDER_CONSTANT, 200)
    
    cv2.imshow('distorted', scaleDown(distortedImg, 0.15))
    #cv2.waitKey(0)
    
    # Try undistorting to see if get back to original
    undistorted = cv2.undistort(distortedImg, K, dist)
    
    cv2.imshow('re-undistort', scaleDown(undistorted, 0.15))    
    
    cv2.imshow('actual image', scaleDown(testImage, 0.15))
    
    remapped = cv2.undistort(testImage, K, dist)
    cv2.imshow('undistorted actual', scaleDown(remapped, 0.15))

    cv2.waitKey(0)    
    
    
    
    
    
    
    
    # for i, point in enumerate(imagePoints):
    #     v = int(round(point[1]))
    #     u = int(round(point[0]))
        
    #     if v >= 0 and v < imageShape[0] and u >= 0 and u < imageShape[1]:
    #         image[v, u] = \
    #             boardImg[points[i,1], points[i,0]]
    
    
    # outputPoints, mapu, mapv = mapInput(K, np.zeros((1,5)), R, T, points, boardPoints, boardImg)
    
    # image = np.zeros(imageShape)
    
    # imagePoints, _ = cv2.projectPoints(board.chessboardCorners, rvecs[-1], tvecs[-1], K, dist)
    
    # imagePoints = np.squeeze(imagePoints)
    
    # for point in imagePoints:
    #     if point[1] >= 0 and point[1] < imageShape[0] and point[0] >= 0 and point[0] < imageShape[1]:
    #         image[int(round(point[1]))-20:int(round(point[1]))+20, int(round(point[0]))-20:int(round(point[0]))+20] = 255
    
    # for i, outputPoint in enumerate(imagePoints):
    #     outputPoint = np.squeeze(outputPoint)
    #     outU = int(round(outputPoint[0]))
    #     outV = int(round(outputPoint[1]))
    #     if outU >= 0 and outU < boardImg.shape[1] and outV >= 0 and outV < boardImg.shape[0]:
    #         image[outV, outU] = boardImg[points[i,1], points[i,0]]
        
    #output = cv2.remap(boardImg.astype(np.float32), mapu.astype(np.float32), mapv.astype(np.float32), cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    #H = K @ np.hstack([R[:, :-1], np.expand_dims(T, axis=1)]) @ Kprime

    #H = np.hstack([R[:, :-1], np.expand_dims(T, axis=1)]) @ Kprime

    #output = cv2.warpPerspective(boardImg, H, imageShape)
    
    #mapx, mapy = mapInput(K, dist, R, T, boardImg)
    
    #output = cv2.remap(boardImg, mapx, mapy, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)