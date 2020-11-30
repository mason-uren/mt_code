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
import scipy.optimize
import itertools
from dynamicIntrinsicsHelpers import *
import cv2

# Given a single image compute the slope M between p and q
def computeM(chorners, chids, pixel2world, intrinsics, dist, qList, graph = False):
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
    ITERATE = True
        
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
    
    qx, p, poly, Mstar = computeM(chorners, chids, pixel2world, defaultIntrinsics, dist, qList)
    
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
    
    if not useIntercept:    
        qHat = f * (1 / M + 1)
    
    else:
        # Assume p = Mq + a0 and plug into 1/p+1/q = 1/f
        num = -a0 / f + M + 1
        disc = np.sqrt((a0/f - M - 1)**2 + 4 * M *a0 / f)
        den = 2 / f * M
        
        root1 = (num + disc) / den
        root2 = (num - disc) / den

        #For root to be valid must be at least as large as f
        qHat = None
        distance = math.inf
        theoreticalQ = f / M + f
        for root in [root1, root2]:
            temp = np.abs(root - theoreticalQ)
            if root > f and temp < distance:
                dist = temp
                qHat = root 
        
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

if __name__ == '__main__':
    
    plt.close('all')
    
    TRANSLATING = True
    ROTATING = False

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
    # Need to separate the imperx and ximea
    cameraName = 'ximea'
    
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    
    # TODO: Change this back or refine it as needed
    f = 200 * 1e-3 / 1.0372608996079156
    
    imageShape = (6004, 7920)
    
    pixel2world = sensor_size[0] / imageShape[0]
    
    qList = [(f / M + f) / pixel2world for M in np.linspace(21, 23, 10)]
    
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
    
    intrinsicsList, distortionList, focusList, roundsList = loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
    
    # Obtain the median intrinsics and distortion parameters
    temp = np.argsort(focusList[0])
    medianInd = temp[len(temp) // 2]
    medIntrinsics = intrinsicsList[0][medianInd]
    medDist = distortionList[0][medianInd]
    medFocus = focusList[0][medianInd]
    
    if TRANSLATING:
        readPath = 'C:/Users/aofeldman/Desktop/Experiments8-25/translating'
    
    elif ROTATING:
        readPath = 'C:/Users/aofeldman/Desktop/Experiments8-25/rotating'
        
    filenames = glob.glob(readPath + '/*.jpg') + \
                glob.glob(readPath + '/*.tif')
    
    # Reorder based on initial numbering
    image_list = []
    for i in range(1, len(filenames)+1):
        image_list.append(cv2.imread(readPath + '/ximea'+str(i) + '.tif', 0))
    
    # Slightly blur all images
    image_list = [cv2.blur(img, (3,3)) for img in image_list]
    
    print('Number of images: ' + str(len(image_list)))
    
    graph = True
    
    rotations = [[], [], []]
    translations = [[], [], []]
    fEffList = []
    polyList = []
    MstarList = []
    MestList = []
    
    # Uncomment to make median estimate closely align with new approach for relative p-estimates
    medIntrinsics[0,0] = f / pixel2world
    medIntrinsics[1,1] = f / pixel2world
    
    
    for i, image in enumerate(image_list):
        ret, chorners, chids, markerCorners, markerIds = findChorners(image, board, aruco_dict, 8)
        qx, p, poly, Mstar = computeM(chorners, chids, pixel2world, medIntrinsics, medDist, qList, graph)
        polyList.append(poly)
        MstarList.append(Mstar)
        
        
        rvec, tvec, fEff, Mlist = estimatePoseFancy(qList, pixel2world, f, medIntrinsics, medDist, chorners, chids, markerCorners, markerIds, board)
        MestList.append(Mlist[-1])
        fEffList.append(fEff)
        
        R, _ = cv2.Rodrigues(rvec)
        rotations[0].append(np.rad2deg(rotationMatrixToEulerAngles(R)))
        translations[0].append(np.squeeze(tvec))
        
        print('\nNew Approach')
        print('Rotation: ' + str(rotations[0][-1]))
        print('Translation: ' + str(translations[0][-1]))
        
        retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, medIntrinsics, medDist, None, None, useExtrinsicGuess=False)
        
        R, _ = cv2.Rodrigues(rvec)
        rotations[1].append(np.rad2deg(rotationMatrixToEulerAngles(R)))
        translations[1].append(np.squeeze(tvec))
        
        print('\nMedian')
        print('Rotation: ' + str(rotations[1][-1]))
        print('Translation: ' + str(translations[1][-1]))
        
        rvec, tvec, qHatList = altThinLensPose(pixel2world, f, medIntrinsics, medDist, chorners, chids, board)
        
        R = cv2.Rodrigues(rvec)[0]
        rotations[2].append(np.rad2deg(rotationMatrixToEulerAngles(R)))
        translations[2].append(np.squeeze(tvec))
    
        print('\nAlt Thin Lens')
        print('Rotation: ' + str(rotations[2][-1]))
        print('Translation: ' + str(translations[2][-1]))
        
    
    if ROTATING:
        # Left column is M, right column is a0, each row is a different image
        polyList = np.array(polyList)
        imageNumbers = range(1,len(image_list)+1)
        # Plot the z translation across the images
        zChange = np.squeeze(translations[0])[:,2]
        
        plt.figure()
        plt.scatter(imageNumbers, zChange)
        plt.xlabel('Image Number')
        plt.ylabel('Z (optical axis) translation [m] Board wrt Camera')
        
        plt.figure()
        plt.scatter(imageNumbers, fEffList)
        plt.xlabel('Image Number')
        plt.ylabel('Computed Effective Focal Length')
        
        plt.figure()
        plt.scatter(imageNumbers, polyList[:,0], label='With Intercept')
        plt.scatter(imageNumbers, MstarList, label='Without Intercept')
        plt.scatter(imageNumbers, MestList, label='After Iteration')
        plt.xlabel('Image Number')
        plt.ylabel('M')
        plt.legend()
        
        plt.figure()
        plt.scatter(imageNumbers, polyList[:,1])
        plt.xlabel('Image Number')
        plt.ylabel('a0')
        
        pitchs = np.squeeze(rotations[1])[:,1]
        
        plt.figure()
        plt.scatter(pitchs, fEffList)
        plt.xlabel('Pitch [deg]')
        plt.ylabel('Computed Effective Focal Length')
        
        plt.figure()
        plt.scatter(pitchs, polyList[:,0], label='With Intercept')
        plt.scatter(pitchs, MstarList, label='Without Intercept')
        plt.scatter(pitchs, MestList, label='After Iteration')
        plt.xlabel('Pitch [deg]')
        plt.ylabel('M')
        plt.legend()
        
        plt.figure()
        plt.scatter(pitchs, polyList[:,1])
        plt.xlabel('Pitch [deg]')
        plt.ylabel('a0')
    
    if TRANSLATING:
        # Left column is M, right column is a0, each row is a different image
        polyList = np.array(polyList)
        imageNumbers = range(1,len(image_list)+1)
        # Plot the z translation across the images
        zChange = np.squeeze(translations[0])[:,2]
        pitchs = np.squeeze(rotations[0])[:,2]
        xChange = np.squeeze(translations[0])[:,0]
        
        plt.figure()
        plt.scatter(imageNumbers, zChange)
        plt.xlabel('Image Number')
        plt.ylabel('Z (optical axis) translation [m] Board wrt Camera')
        
        plt.figure()
        plt.scatter(imageNumbers, pitchs)
        plt.xlabel('Image Number')
        plt.ylabel('Pitch Board wrt Camera')
        
        plt.figure()
        plt.scatter(imageNumbers, fEffList)
        plt.xlabel('Image Number')
        plt.ylabel('Computed Effective Focal Length')
        
        plt.figure()
        plt.scatter(imageNumbers, polyList[:,0], label='With Intercept')
        plt.scatter(imageNumbers, MstarList, label='Without Intercept')
        plt.scatter(imageNumbers, MestList, label='After Iteration')
        plt.xlabel('Image Number')
        plt.ylabel('M')
        plt.legend()
        
        plt.figure()
        plt.scatter(imageNumbers, polyList[:,1])
        plt.xlabel('Image Number')
        plt.ylabel('a0')
        
        plt.figure()
        plt.scatter(xChange, fEffList)
        plt.xlabel('X [m]')
        plt.ylabel('Computed Effective Focal Length')
        
        plt.figure()
        plt.scatter(xChange, polyList[:,0], label='With Intercept')
        plt.scatter(xChange, MstarList, label='Without Intercept')
        plt.scatter(xChange, MestList, label='After Iteration')
        plt.xlabel('X [m]')
        plt.ylabel('M')
        plt.legend()

        plt.figure()
        plt.scatter(xChange, polyList[:,1])
        plt.xlabel('X [m]')
        plt.ylabel('a0')
        
    # Rotation estimation fluctuates with thin lens approach
    thinLensPitch = np.squeeze(rotations[2])[:,1]
    plt.figure()
    plt.title('Estimated Pitch: Median and Thin Lens')
    plt.scatter(imageNumbers, thinLensPitch, label='Thin Lens')
    plt.scatter(imageNumbers, pitchs, label='Median with f = q')
    plt.xlabel('Image Number')
    plt.ylabel('Estimated Pitch [deg]')
    plt.legend()
    
    # Thin Lens Based - Fixed (with q = f)
    # From theory would expect that M(f/M+f) - fM = f consistently
    # However, see strong correlation with pitch!
    deltaTrans = np.squeeze(translations[0]) - np.squeeze(translations[1])
    plt.figure()
    plt.scatter(pitchs, deltaTrans[:,2], label='Estimate Difference')
    plt.xlabel('Pitch [deg]')
    plt.ylabel('Difference between Median and Thin Lens Z [m]')
    plt.plot(pitchs, [f] * len(pitchs), label='Input f')
    plt.scatter(pitchs, np.squeeze(translations[2])[:,2] - np.squeeze(translations[1])[:,2], label='Alt Thin Lens')
    plt.legend()
    