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
from fitVaryingModel import loadFixedModels
import scipy.optimize as opt
import itertools
import scipy.linalg

def scaleDown(image, fraction):
    return cv2.resize(image, (int(fraction * image.shape[1]), int(fraction * image.shape[0])))


def findF(board, chorners, chids):
    # Should be N X 3 (x,y,z)
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    # Extract (x,y) components
    boardPoints = objectPoints[:, :-1]
    
    # Should be N X 2
    chorners = np.squeeze(chorners)
    
    
    F, mask = cv2.findFundamentalMat(boardPoints, chorners)
    
    e = scipy.linalg.null_space(F.T)
    
    assert (np.shape(e) == (3,1))
    
    # Put in u, v, 1 form
    e /= e[2,0]
    
    return F, e

def findCenter(Flist):
    M = np.vstack([Flist[i].T for i in range(len(Flist))])
    # Create a rank 2 approximation for M if needed
    if np.linalg.matrix_rank(M) > 2:
        U, sList, VT = np.linalg.svd(M)
        sList[-1] = 0
        temp = np.zeros((len(U), len(sList)))
        temp[:,0] = U[:,0] * sList[0]
        temp[:,1] = U[:,1] * sList[1]
        temp[:,2] = U[:,1] * sList[2]
        approxM = temp @ VT
        #approxM = (U * sList[..., None, :]) @ VT
    else:
        approxM = M
    # Find basis for nullspace
    e = scipy.linalg.null_space(approxM)

    assert (np.shape(e) == (3,1))
    
    e /= e[2,0]
    
    return e

# TODO: Implement this
# Given the epipole, find a refined F after shifting image points by
# e.x and e.y. Will yield that row 3 of F be 0
def refineF(board, chorners, chids, e):
    # Should be N X 3 (x,y,z)
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    # Extract (x,y) components
    boardPoints = objectPoints[:, :-1]
    
    # Should be N X 2
    chorners = np.squeeze(chorners)
        
    # Shift the points by the epipole
    boardPoints[:,0] -= e[0,0]
    boardPoints[:,1] -= e[1,0]
    chorners[:,0] -= e[0,0]
    chorners[:,1] -= e[1,0]
        
    # Normalize for numerical stability
    normBoardPoints = np.copy(boardPoints)
    normChorners = np.copy(chorners)
    
    # First, center
    #boardMu = np.mean(normBoardPoints, axis=0)
    #chornerMu = np.mean(normChorners, axis=0)
    
    #normBoardPoints[:,0] -= boardMu[0]
    #normBoardPoints[:,1] -= boardMu[1]
    #normChorners[:,0] -= chornerMu[0]
    #normChorners[:,1] -= chornerMu[1]
    
    # Now, scale so average distance from origin is 1
    boardScale = 1 / np.mean(np.linalg.norm(normBoardPoints, axis=1))
    chornerScale = 1 / np.mean(np.linalg.norm(normChorners, axis=1))
    
    normBoardPoints *= boardScale
    normChorners *= chornerScale
    
    
    # TODO: Cannot shift the points, but may be able to still scale
    
    Tboard = np.zeros((3,3))
    Tboard[0,0] = boardScale
    Tboard[1,1] = boardScale
    Tboard[2,2] = 1
    #Tboard[0,2] = -boardMu[0]
    #Tboard[1,2] = -boardMu[1]
    
    Tchorners = np.zeros((3,3))
    Tchorners[0,0] = chornerScale
    Tchorners[1,1] = chornerScale
    Tchorners[2,2] = 1
    #Tchorners[0,2] = -chornerMu[0]
    #Tchorners[1,2] = -chornerMu[1]

    # Each row is a point correspondence, each column is a constraint on f
    # f = [F11, F12, F13, F21, F22, F23] (and 3rd row should be 0)
    # A = np.zeros((len(chorners, 6))
    # A[:,0] = boardPoints[:,0] * chorners[:,0]
    # A[:,1] = boardPoints[:,0] * chorners[:,1]
    # A[:,2] = boardPoints[:,0]
    # A[:,3] = boardPoints[:,1] * chorners[:,0]
    # A[:,4] = boardPoints[:,1] * chorners[:,1]
    # A[:,5] = boardPoints[:,1]
    
    A = np.zeros((len(normChorners), 6))
    A[:,0] = normBoardPoints[:,0] * normChorners[:,0]
    A[:,1] = normBoardPoints[:,0] * normChorners[:,1]
    A[:,2] = normBoardPoints[:,0]
    A[:,3] = normBoardPoints[:,1] * normChorners[:,0]
    A[:,4] = normBoardPoints[:,1] * normChorners[:,1]
    A[:,5] = normBoardPoints[:,1]
    
    Fnorm, _ = cv2.findFundamentalMat(normChorners, normBoardPoints)
    
    print(np.shape(Fnorm))
    # f = constrainedMin(A)
    # f = np.reshape(f, (2,3))
    
    # Fnorm = np.zeros((3,3))
    # Fnorm[0] = f[0]
    # Fnorm[1] = f[1]
    # Fnorm[2,:] = 0
    
    Forig = Tchorners.T @ Fnorm @ Tboard
    
    print('Forig:' , Forig)
    
    Forig[2,:] = 0
    
    return Forig

# After computing the refined F (with third row 0), can find v to estimate
# the full homography
def computeV(board, chorners, chids, e, F):
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    
    chorners = np.squeeze(chorners)
    
    # Shift the points by the epipole
    objectPoints[:,0] -= e[0,0]
    objectPoints[:,1] -= e[1,0]
    chorners[:,0] -= e[0,0]
    chorners[:,1] -= e[1,0]
    
    print('refinedF: ', F)
    
    Hhat = np.copy(F)[:-1, :] # Only take first two rows
    Hhat[0,:] *= -1
    
    rd = np.linalg.norm(chorners, axis=1)
    # N X 2 of Hhat @ xC
    xHatU = (Hhat @ objectPoints.T).T
    
    # Want chorners and xHatU to have same sign so if -1, make go in -1 radial
    # direction
    sgn = np.sign(np.sum(chorners  * xHatU, axis=1))
    rHatU = sgn * np.linalg.norm(xHatU, axis=1)
    
    ordering = np.argsort(rd)
    
    rd = rd[ordering]
    xHatU = xHatU[ordering]
    rHatU = rHatU[ordering]
    objectPoints = objectPoints[ordering]

    A = (np.tile(rHatU[1:], (3,1)).T * objectPoints[:-1,:] - np.tile(rHatU[:-1], (3,1)).T * objectPoints[1:,:])
    print('A Shape: ', np.shape(A))
    print('rank A: ', np.linalg.matrix_rank(A))
    print('A\n', A)
    v = constrainedMin(A)
    
    print('v', v)
    print('shape', np.shape(v))
    distortionFactor = objectPoints @ v
    undistorted = np.hstack([xHatU, distortionFactor])
    undistorted[:,0] = np.divide(undistorted[:,0], undistorted[:,2])
    undistorted[:,1] = np.divide(undistorted[:,1], undistorted[:,2])
    
    undistorted = undistorted[:, :-1]
    
    ru = np.linalg.norm(undistorted, axis=1)
    
    plt.figure()
    plt.title('Undistorted vs. Distorted Radii')
    plt.scatter(rd, ru)
    plt.ylabel('ru')
    plt.xlabel('rd')
    
    # Shift back to normal coordinates
    undistorted[:,0] += e[0,0]
    undistorted[:,1] += e[1,0]
    
    return v, undistorted

# Given A, find f such ||Af||**2 is minimized st. ||f|| = 1
def constrainedMin(A):
    U, sList, VT = np.linalg.svd(A)
    minVecInd = np.argmin(sList)
    minVec = VT[minVecInd, :]
    print('predictedResult', sList[minVecInd]**2 * minVec)
    print('actualResult', A.T @ A @ minVec)
    assert (np.linalg.norm(A.T @ A @ minVec - sList[minVecInd]**2 * minVec) < 1e-8)
    return minVec / np.linalg.norm(minVec)


def bootstrap(chornersList, chidsList, numSamples, subsetSize):
    assert (subsetSize <= len(chornersList))
    assert (len(chornersList) == len(chidsList))
    
    Flist = []
    elist = []
    for chorners, chids in zip(chornersList, chidsList):
        F, e = findF(board, chorners, chids)
        Flist.append(F)
        elist.append(e)
        
    samplesDrawn = 0
    subsets = []
    
    # Randomly draw samples of given size from the image_list
    while samplesDrawn < numSamples:
        print('Drawing sample ' + str(samplesDrawn) + ' of ' + str(numSamples))
        chosenInd = np.random.choice(len(chornersList), subsetSize, False) # sample without replacement
        while any(np.array_equal(chosenInd, x) for x in subsets):
            chosenInd = np.random.choice(len(chornersList), subsetSize, False) # sample without replacement
        #subset = [(chornersList[i], chidsList[i]) for i in chosenInd]
        #subsets.append(subset)
        subsets.append(chosenInd)
        samplesDrawn += 1

    centerList = []

    for sample, subset in enumerate(subsets):
        
        Fsubset = [Flist[i] for i in subset]
        
        multiE = findCenter(Fsubset)
        centerList.append(np.squeeze(multiE[:-1]))
    
    centerList = np.array(centerList)
    
    plt.figure()
    plt.title('Computed Distortion Center')
    # Fake middle dimension
    plt.scatter(chorners[:,0,0], chorners[:,0,1], color='r', label='Distorted Corners')
    plt.scatter(centerList[:,0], centerList[:,1], color='g', label='Distortion Centers')
    plt.legend()
    
    return centerList

if __name__ == '__main__':
    ##########################
    # Charuco Board Consts   #
    ##########################
    
    squareLength = boards['CorrectedTV_3']['squareLength']
    markerLength = boards['CorrectedTV_3']['markerLength']
    # In the images, seems like board is actually rotated so that have 12 across 
    # by 8 down 
    charucoX = boards['CorrectedTV_3']['charucoX']
    charucoY = boards['CorrectedTV_3']['charucoY']
    
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
    f = 200 * 1e-3 # / 1.0372608996079156
    
    cameraName = 'ximea'
            
    sensor_size = np.array([27.6, 36.4]) * 1e-3
        
    # Load intrinsics        
    path = 'C:/Users/aofeldman/Desktop/testCollection9-11'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 12
    trainingRounds = set(range(1, numRounds+1))
    testRounds = []
    
    overallCenterList = []
    
    epipoles = []
    for rnd in trainingRounds:
        print('On round ' + str(rnd))
        readPath = path + '/' + subfolderPrefixes + str(rnd)
            
        filenames = glob.glob(readPath + '/*.jpg') + \
                    glob.glob(readPath + '/*.tif')
            
        
        imageNumbers = [int(filenames[i].replace(readPath +'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
        filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
        imageNumbers = np.sort(imageNumbers)
        
        print('imageNumbers\n', imageNumbers)
        
        # Reorder based on initial numbering
        image_list = []
        for i in imageNumbers:
            image_list.append(cv2.imread(readPath + '/ximea'+str(i) + '.tif', 0))
        
        # Slightly blur all images
        image_list = [cv2.blur(img, (3,3)) for img in image_list]
        Flist = []
        
        print('Number of images: ' + str(len(image_list)))
        
        chornersList = []
        chidsList = []
        for i, image in enumerate(image_list):
            print('On image: ' + str(i))
            
            parameters = cv2.aruco.DetectorParameters_create()
        
            markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
            
            # if we dont find enough points skip
            if (ids is not None and len(ids) > 8):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                
                chornersList.append(chorners)
                chidsList.append(chids)
                #F, e = findF(board, chorners, chids)
                #print('Computed F was: ', F)
                #print('Computed e was: ', e)
                #Flist.append(F)
        
                #refinedF = refineF(board, chorners, chids, e)
                #v, undistorted = computeV(board, chorners, chids, e, refinedF)
                        
                # plt.figure()
                # plt.title('Undistorting Image')
                # plt.scatter(chorners[:,0], chorners[:,1], color='r', label='Distorted')
                # plt.scatter(undistorted[:,0], undistorted[:,1], color='g', label='Undistorted')
                # plt.legend()
        
        centerList = bootstrap(chornersList, chidsList, 100, 15)
        
        overallCenterList.append(centerList)
        
        #multiE = findCenter(Flist)
        #epipoles.append(multiE)

# focusList = [588, 295, 368, 441, 493, 542]
#colors = ['m', 'y', 'r', 'g', 'c', 'b']

focusList = [473, 428, 526, 563, 435, 521, 572, 292, 323, 372, 606, 420]

lowest, highest = np.argmin(focusList), np.argmax(focusList)
scale = focusList[lowest], focusList[highest]
colors = [(focus - scale[0]) / (scale[1] - scale[0]) for focus in focusList]
colors = np.clip(colors, 0, 1)
# Actually rescale it so only goes up to 0.75 and 0 is red so invert
# Set l and s by default
# Each row should be color (r,g,b) for row'th point
colors = [colorsys.hls_to_rgb(0.75 * (1 - color), 0.5, 1) for color in colors]
   
plt.rcParams["scatter.marker"] = 'x'

plt.figure()
plt.title('Distortion Centers Overlaid Across Focus Positions')
for i, centerList in enumerate(overallCenterList):
    plt.scatter(centerList[:,0], centerList[:,1], color=colors[i], label='Focus = ' + str(focusList[i]))
plt.xlabel('cx')
plt.ylabel('cy')

order = np.argsort(focusList)
legend_elements = [Patch(facecolor=colors[i], label=focusList[i]) for i in order]
plt.legend(handles=legend_elements)

 
        