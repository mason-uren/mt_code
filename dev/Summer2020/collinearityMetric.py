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
from dynamicIntrinsicsHelpers import *
import scipy.optimize as opt
import itertools

# Assumes already undistorted
def assessDistortion(chorners, chids, graph=False, image=None):
    lines, linesInd = findLines(board, chorners, chids, 5)
    
    xLines = lines[1]
    yLines = lines[0]
    
    # print('Number of lines: ', len(xLines))
    # Should plot the points and the corresponding lines
    
    totalError = 0
    errVecs = []    
    
    A = np.zeros((len(xLines) + len(yLines), 2))
    b = np.zeros((len(xLines) + len(yLines)))
    counter = 0
    
    for i, dirLines in enumerate([xLines, yLines]):
        for row, line in enumerate(dirLines):
            nx, ny, d0, error, indError = computeLine(line)
            A[counter, :] = np.array([nx, ny])
            b[counter] = d0
            counter +=1
            totalError += error
            errVecs += indError
            
    #vx = np.linalg.pinv(A) @ b
    
    if graph and image is not None:
        
        color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        for j, point in enumerate(np.squeeze(chorners)):
            color_image[int(point[1])-20:int(point[1])+20, int(point[0])-20:int(point[0])+20] = (0, 0, 255)
            #print('Point: ', point)
        
        # Plot corresponding lines
        for row in range(len(A)):
            nx, ny = A[row, :]
            d0 = b[row]
            
            #print('nx', nx)
            #print('ny', ny)
            #print('d0', d0)
            
            point1 = projPointLine((0, 0), (nx, ny, d0))
            point2 = projPointLine((7920, 6004), (nx, ny, d0))
            
            cv2.line(color_image, (int(point1[0]), int(point1[1])), (int(point2[0]), int(point2[1])), (0, 255, 0), thickness=8)
        
        # Plot vanishing point
        # color_image[int(vx[1])-20:int(vx[1])+20, int(vx[0])-20:int(vx[0])+20] = (255, 0, 0)
        
        cv2.imshow('found lines', scaleDown(color_image, 0.15))
        cv2.waitKey(0)
        
    return totalError, errVecs

# Given the board and detected chessboard corners and ids, create list of 
# vertical and horizontal lines identified. minPoints specifies the minimum
# number of chessboard corners for a line to be used.
def findLines(board, chorners, chids, minPoints):
    # Should be N X 3 (x,y,z)
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    # Should be N X 2
    modChorners = np.squeeze(chorners)
    
    numXlines, numYlines = board.getChessboardSize()
    squareLength = board.getSquareLength()
    xLines = []
    yLines = []
    xLinesInd = []
    yLinesInd = []
    
    for i, xInd in enumerate([squareLength * i for i in range(numXlines)]):
        mask = (objectPoints[:, 0] == xInd)
        if np.sum(mask):
            line = modChorners[np.where(mask)]
            if len(line) > minPoints:
                xLines.append(line)        
                xLinesInd.append(np.where(mask))
                
    for i, yInd in enumerate([squareLength * i for i in range(numYlines)]):
        mask = (objectPoints[:, 1] == yInd)
        if np.sum(mask):
            line = modChorners[np.where(mask)]
            if len(line) > minPoints:
                yLines.append(line)
                yLinesInd.append(np.where(mask))
    
    # TODO: These are mislabled
    return [xLines, yLines], [xLinesInd, yLinesInd]

# Given the list of undistorted points, computes the corresponding
# associated line parameterized by nx, ny, d0: [nx, ny].T [x, y] - d0 = 0
def computeLine(undistorted):
    Nm = len(undistorted)
    Ex = 1 / Nm * np.sum(undistorted[:,0])
    Ey = 1 / Nm * np.sum(undistorted[:,1])
    Exx = 1 / Nm * np.sum(np.square(undistorted[:,0]))
    Eyy = 1 / Nm * np.sum(np.square(undistorted[:,1]))
    Exy = 1 / Nm * np.sum(undistorted[:,0] * undistorted[:,1])
    
    # Horizontal-like line y = ax + b
    if (Exx - Ex**2) >= (Eyy - Ey**2):
        a = (Exy - Ex * Ey) / (Exx - Ex**2)
        b = (Exx * Ey - Ex * Exy) / (Exx - Ex**2)
        nx = -a / np.sqrt(a**2 + 1)
        ny = 1 / np.sqrt(a**2 + 1)
        d0 = b / np.sqrt(a**2 + 1)
    
    # Vertical-like line x = cy + d
    else:
        c = (Exy - Ex * Ey) / (Eyy - Ey**2)
        d = (Eyy * Ex - Ey * Exy) / (Eyy - Ey**2)
        nx = 1 / np.sqrt(c**2 + 1)
        ny = -c / np.sqrt(c**2 + 1)
        d0 = d / np.sqrt(c**2 + 1)
        
    # Given the line, compute the SSE for points and the line
    error = 0
    errVecs = []
    for i in range(len(undistorted)):
        proj = projPointLine(undistorted[i], (nx, ny, d0))
        errVecs += (proj - undistorted[i]).tolist()
        error += np.linalg.norm(proj - undistorted[i])**2
    
    return nx, ny, d0, error, errVecs

def generateSamples(numImages, subsetSize, numSamples, cameraName='ximea'):
    assert (subsetSize <= numImages)
    
    samplesDrawn = 0
    subsets = []
    
    # Assume the probability of an exact repeat is small
    # Randomly draw samples of given size from the image_list
    while samplesDrawn < numSamples:
        print('Drawing sample ' + str(samplesDrawn) + ' of ' + str(numSamples))
        chosenInd = np.random.choice(numImages, subsetSize, False) # sample without replacement
        subsets.append(chosenInd)
        samplesDrawn += 1
        
    return subsets

def bootstrap(image_list, subsetSize, numSamples, perturbKlist, perturbDistList):
    chornersList = []
    chidsList = []
    
    for j, image in enumerate(image_list):
        print('On image: ' + str(j))
        
        parameters = cv2.aruco.DetectorParameters_create()
    
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            chornersList.append(chorners)
            chidsList.append(chids)
    
    subsets = generateSamples(len(image_list), subsetSize, numSamples)
    
    print('subsets', subsets)
    
    # Each row is a different subset
    # Each column is a different camera matrix
    meanErrList = np.zeros((len(subsets), len(perturbKlist)))
    
    for i, subset in enumerate(subsets):
        for j, (cameraMatrix, dist) in enumerate(zip(perturbKlist, perturbDistList)):
            meanError = 0
            print('Camera Matrix: ', cameraMatrix)
            subChorners = np.array(chornersList)[subset]
            subChids = np.array(chidsList)[subset]
            print('shape subChorners:', np.shape(subChorners))
            for k, (chorners, chids) in enumerate(zip(subChorners, subChids)):
                totalError, errVecs = computeCollinearity(cameraMatrix, dist, chorners, chids, image_list[0].shape, True)
                
                
                undistorted = cv2.undistortPoints(np.squeeze(chorners), cameraMatrix, dist, np.eye(3), cameraMatrix)
                
                # color_image = cv2.cvtColor(image_list[subset[k]], cv2.COLOR_GRAY2BGR)
                
                # for j, point in enumerate(np.squeeze(chorners)):
                #     color_image[int(point[1])-20:int(point[1])+20, int(point[0])-20:int(point[0])+20] = (0, 0, 255)
                #     #print('Point: ', point)        
                # cv2.imshow('distorted', scaleDown(color_image, 0.15))
                
                # remapped = cv2.undistort(image_list[subset[k]], cameraMatrix, dist)
                # print('original', assessDistortion(undistorted, chids, True, remapped)[0])
                print('error', totalError)
                meanError += totalError
                
            meanError /= len(subChorners)
            meanErrList[i, j] = meanError
    
    return meanErrList

def computeCollinearity(cameraMatrix, dist, chorners, chids, imageShape, graph=False):
    undistorted = cv2.undistortPoints(chorners, cameraMatrix, dist, np.eye(3), cameraMatrix)
    undistorted = np.squeeze(undistorted)
    # pdb.set_trace()
    if graph:
        plt.figure()
        plt.title('Undistorted')
        plt.scatter(undistorted[:,0], undistorted[:,1], label='Undistorted')
        plt.scatter(np.squeeze(chorners)[:,0], np.squeeze(chorners)[:,1], label='Original')
        plt.legend()
    return assessDistortion(undistorted, chids, graph, np.zeros(imageShape).astype(np.float32))        

if __name__ == '__main__':
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
    f = 200 * 1e-3 / 1.0372608996079156
    
    cameraName = 'ximea'
            
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    
    imageShape = (6004, 7920)

        
    #readPath = 'C:/Users/aofeldman/Desktop/testCollection8-31/AFround3'
        
    # filenames = glob.glob(readPath + '/*.jpg') + \
    #             glob.glob(readPath + '/*.tif')
        
    # Load intrinsics        
    path = 'C:/Users/aofeldman/Desktop/testCollection8-31'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 19
    trainingRounds = set(range(numRounds+1))
    testRounds = []
    
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
    
    meanErrList = []
    for i, AFround in enumerate(roundList[0]):                
        prefix = path + '/' + subfolderPrefixes + str(AFround) + '/'
            
        foldername = path + '/' + subfolderPrefixes + str(AFround)
    
        # Load in all images for the given focus setting
        filenames = glob.glob(foldername + '/' + '*.tif')
        
        imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
        filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
        imageNumbers = np.sort(imageNumbers)
        
        # Read in and mildly blur images
        image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
        K = intrinsicsList[0][i]
        
        meanError = bootstrap(image_list, len(image_list), 1, [K], [distortionList[0][i]]).item()
        meanErrList.append(meanError)
            
    fList = [intrinsicsList[0][i][0,0] for i in range(len(intrinsicsList[0]))]
    u0List = [intrinsicsList[0][i][0,2] for i in range(len(intrinsicsList[0]))]
    v0List = [intrinsicsList[0][i][1,2] for i in range(len(intrinsicsList[0]))]
    k1List = [distortionList[0][i][0,0] for i in range(len(distortionList[0]))]

    qList = np.array(fList) * sensor_size[0] / imageShape[0]
    avgZ = np.array([4.73908615, 3.34321726, 3.59055629, 3.88516498, 4.19571537, 4.49398583])
    
    #u0List = [3930 + 5*i for i in range(10)]
    #v0List = [3030 + 10*i for i in range(10)]

    AFround = 3
    
    index = np.where(np.array(roundList[0]) == AFround)
                
    prefix = path + '/' + subfolderPrefixes + str(AFround) + '/'
    
    # Load the appropriate intrinsics, distCoeffs
    K = np.load(prefix + cameraName + '_intrinsics.npy')
    
    # Modify the q and see if resulting best u0 changes
    #K[0,0] = fList[1]
    #K[1,1] = fList[1]
    
    dist = np.load(prefix + cameraName + '_distCoeffs.npy')
    
    meanErrList = []
        
    # Comment this out to see what happens when use the same K, dist across all focus positions
    #K = intrinsicsList[0][i]
    #dist = distortionList[0][i]
    
    foldername = path + '/' + subfolderPrefixes + str(AFround)

    # Load in all images for the given focus setting
    filenames = glob.glob(foldername + '/' + '*.tif')
    
    imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
    filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
    imageNumbers = np.sort(imageNumbers)
    
    # Read in and mildly blur images
    image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
        
    print('Number of images: ' + str(len(image_list)))
    
    plotLines = False
            
    perturbKlist = []
    perturbDistList = []
    
    varyF = False
    varyU = False
    varyV = False
    wrongPoint = True
    varyK = False
    
    if varyF:
        for param in fList:
            cameraMatrix = np.copy(K)
            cameraMatrix[0,0] = param
            cameraMatrix[1,1] = param
            perturbKlist.append(cameraMatrix)
            perturbDistList.append(dist) 
    elif varyU:
        for param in u0List:
            cameraMatrix = np.copy(K)
            cameraMatrix[0,2] = param
            perturbKlist.append(cameraMatrix)
            perturbDistList.append(dist)
    elif varyV:
        for param in v0List:
            cameraMatrix = np.copy(K)
            cameraMatrix[1,2] = param
            perturbKlist.append(cameraMatrix)
            perturbDistList.append(dist)
    elif wrongPoint:
        for cameraMatrix in intrinsicsList[0]:
            perturbKlist.append(cameraMatrix)
            perturbDistList.append(dist)
    elif varyK:
        for param in k1List:
            perturbKlist.append(K)
            revDist = np.copy(dist)
            revDist[0,0] = param
            perturbDistList.append(revDist)
        
    # Can always do len(image_list), 1 to get non-bootstrap result
    meanErrArr = bootstrap(image_list, len(image_list), 1, perturbKlist, perturbDistList)
        
    print('meanErrArr shape:', np.shape(meanErrArr))
    
    # Average across subsets
    if varyF:
        plt.figure()
        plt.scatter(fList, np.mean(meanErrArr, axis=0))
        plt.axvline(x = K[0,0], linestyle='dashed', label='Point Estimated')
        plt.xlabel('f')
        plt.ylabel('MSE across images')
        plt.title('Colinearity When Using Different f Values')
        plt.legend()
    elif varyU:
        plt.figure()
        plt.scatter(u0List, np.mean(meanErrArr, axis=0))
        plt.axvline(x = K[0,2], linestyle='dashed', label='Point Estimated')
        plt.xlabel('u0')
        plt.ylabel('MSE across images')
        plt.title('Colinearity When Using Different u0 Values')
        plt.legend()
    elif varyV:
        plt.figure()
        plt.scatter(v0List, np.mean(meanErrArr, axis=0))
        plt.axvline(x = K[1,2], linestyle='dashed', label='Point Estimated')
        plt.xlabel('v0')
        plt.ylabel('MSE across images')
        plt.title('Colinearity When Using Different v0 Values')
        plt.legend()
    elif wrongPoint:
        plt.figure()
        plt.scatter(focusList[0], np.mean(meanErrArr, axis=0))
        plt.axvline(x = np.array(focusList[0])[np.where(np.array(roundList[0]) == AFround)], linestyle='dashed', label='True Focus Position')
        plt.xlabel('Focus Position')
        plt.ylabel('MSE across images')
        plt.title('Impact of Wrong Point Estimate on Collinearity')
        plt.legend()
    elif varyK:
        plt.figure()
        plt.scatter(k1List, np.mean(meanErrArr[:,:-1], axis=0))
        plt.axvline(x = dist[0,0], linestyle='dashed', label='Point Estimated')
        plt.axvline(x = extra, linestyle='dashed', label='Using theory k, point q', color='g')
        plt.scatter(k1List[-1], np.mean(meanErrArr[:,-1]), label='Using theory k and q')
        plt.xlabel('k1')
        plt.ylabel('MSE across images')
        plt.title('Colinearity When Using Different k1 Values')
        plt.legend()
        
    
    # for i, cameraMatrix in enumerate(perturbKlist):
    #     meanError = 0
    #     print('Camera Matrix: ', cameraMatrix)
    #     for j, image in enumerate(image_list):
    #         print('On image: ' + str(j))
            
    #         parameters = cv2.aruco.DetectorParameters_create()
        
    #         markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
            
    #         # if we dont find enough points skip
    #         if (ids is not None and len(ids) > 8):
    #             ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                     
    #             remapped = cv2.undistort(image, cameraMatrix, dist)
                
    #             cv2.imshow('distorted', scaleDown(image, 0.15))
    #             cv2.imshow('default undistort', scaleDown(remapped, 0.15))
                
    #             undistorted = cv2.undistortPoints(np.squeeze(chorners), cameraMatrix, dist, np.eye(3), cameraMatrix)
    #             #undistorted = chorners
    #             totalError = assessDistortion(undistorted, chids, plotLines, remapped)            
    #             print('Total error for image ' + str(j), totalError)
    #             meanError += totalError
        
    #     meanError /= len(image_list)
    #     meanErrList.append(meanError)
    
    # # Plot the mean error obtained on the dataset as a function of the focus position used
    # # Plot dashed vertical line for the appropriate point estimate
    # if varyF:
    #     plt.figure()
    #     plt.scatter(fList, meanErrList)
    #     plt.axvline(x = K[0,0], linestyle='dashed', label='Point Estimated')
    #     plt.xlabel('f')
    #     plt.ylabel('MSE across images')
    #     plt.title('Colinearity When Using Different f Values')
    #     plt.legend()
    # elif varyU:
    #     plt.figure()
    #     plt.scatter(u0List, meanErrList)
    #     plt.axvline(x = K[0,2], linestyle='dashed', label='Point Estimated')
    #     plt.xlabel('u0')
    #     plt.ylabel('MSE across images')
    #     plt.title('Colinearity When Using Different u0 Values')
    #     plt.legend()
    # elif varyV:
    #     plt.figure()
    #     plt.scatter(v0List, meanErrList)
    #     plt.axvline(x = K[1,2], linestyle='dashed', label='Point Estimated')
    #     plt.xlabel('v0')
    #     plt.ylabel('MSE across images')
    #     plt.title('Colinearity When Using Different v0 Values')
    #     plt.legend()