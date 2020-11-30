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
from collinearityMetric import computeCollinearity
import cv2

##########################
# Helper Functions       #
##########################

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

def predictPoseFromModel(pixel2world, params, defaultM, focus, chorners, chids, board):
    
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    chorners = np.squeeze(chorners)
    
    rnd = 0
    maxRounds = 5
    
    cameraMatrix, dist = constructIntrinsics(params, defaultM, focus)
   
    cameraMatrixList = [cameraMatrix]
    distList = [dist]
    Mlist = []
        
    
    while rnd < maxRounds:
        retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, cameraMatrix, dist)
        
        q = pixel2world * cameraMatrixList[-1][0,0]
        Mlist.append(tvec[2,0] / q)
        
        cameraMatrix, dist = constructIntrinsics(params, Mlist[-1], focus)
        
        cameraMatrixList.append(cameraMatrix)
        distList.append(dist)
        
        rnd += 1
    
    return np.squeeze(rvec), np.squeeze(tvec), cameraMatrixList, distList, Mlist


# Can either assume constant u0, v0 and ignore focus position, make everything
# dependent on M or assume that u0, v0 vary linearly with focus or M
# (when don't pass in a focus)
def constructIntrinsics(params, M, focus=None):
    global pixel2world

    if len(params) == 7:
        f, u0, v0, ku0, kv0, ka0, ka1 = params
    elif len(params) == 5:
        f, u0, v0, ka0, ka1 = params
        ku0 = 0
        kv0 = 0
    else:
        raise IndexError('First array should contain either 5 or 7 parameters')
    
    cameraMatrix = np.zeros((3,3))
    F = (f / M + f) / pixel2world
    cameraMatrix[0,0] = F
    cameraMatrix[1,1] = F
    
    if focus is not None:
        cameraMatrix[0,2] = u0 + ku0 * focus
        cameraMatrix[1,2] = v0 + kv0 * focus
    else:
        cameraMatrix[0,2] = u0 + ku0 * M
        cameraMatrix[1,2] = v0 + kv0 * M
        
    cameraMatrix[2,2] = 1
    dist = np.zeros((1,5))
    dist[0,0] = ka0 * M + ka1
    
    return cameraMatrix, dist

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

# Expect croppedX = [rx1, ry1, rz1, tx1, ty1, tz1, rx2, ...]
def decomposePose(croppedX):
    numPoses = len(croppedX) // 6
    # Each row should be a different pose, each column a different component
    x = np.reshape(croppedX, (6, numPoses), order='F').T
    rvecs = x[:,:3]
    tvecs = x[:,3:]
    
    return rvecs, tvecs

def createPose(rvecs, tvecs):
    flattenedPoses = []
    for i in range(len(rvecs)):
        flattenedPoses += np.hstack([rvecs[i], tvecs[i]]).tolist()
    return flattenedPoses

def sparsityMap(initParams, chornersList, graph=False):
    yOffset = 0
    xOffset = len(initParams)
    
    # Number of function components is 2 * the number of points (x,y)
    m = sum([2 * len(chorners) for chorners in chornersList])
    n = xOffset + 6 * len(chornersList)
    A = np.zeros((m,n))
    A[:, :xOffset] = 1
    
    for i, chorners in enumerate(chornersList):
        A[yOffset:(2 * len(chorners) + yOffset), xOffset:xOffset + 6] = 1
        xOffset += 6
        yOffset += 2 * len(chorners)
    
    if graph:
        fig = plt.figure()
        ax = plt.axes()
        coordinates = np.transpose(np.where(A != 0))
        ax.invert_yaxis()
        plt.scatter(coordinates[:, 1], coordinates[:, 0])
        plt.title('Jacobian Sparsity Map')
                
    return A
    
# Let's just try re-projection error
def objFunc(x, Mlist, focusList, chornersList, chidsList, graph=False):
    global board
    
    if len(x) % 6 == 5:
        f, u0, v0, ka0, ka1 = x[:5]
        rvecs, tvecs = decomposePose(x[5:])
        ku0 = 0
        kv0 = 0
    elif len(x) % 6 == 1:
        f, u0, v0, ku0, kv0, ka0, ka1 = x[:7]
        rvecs, tvecs = decomposePose(x[7:])

    Rlist = []
    for rvec in rvecs:
        R, _ = cv2.Rodrigues(rvec)
        Rlist.append(R)
    
    Tlist = tvecs.tolist()    
    
    cameraMatrixList = []
    distortionList = []
    
    errorList = []
    # Construct the intrinsics and distortion matrices for each image
    # Then, compute the collinearity error for it
    for i in range(len(Mlist)):
        M = Mlist[i]
        focus = focusList[i]
        param = [f, u0, v0, ku0, kv0, ka0, ka1]
        cameraMatrix, dist = constructIntrinsics(param, M, focus)
        #print('CameraMatrix: ')
        #print(cameraMatrix)
        #print('dist: ')
        #print(dist)
        cameraMatrixList.append(cameraMatrix)
        distortionList.append(dist)
        # print('On Image ' + str(i))
        
        chorners = np.squeeze(chornersList[i])
        chids = np.squeeze(chidsList[i])
        objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
        
        predictedPoints = projectPoints(cameraMatrix, dist, Rlist[i], Tlist[i], objectPoints)
        
        if graph:
            plt.figure()
            plt.scatter(predictedPoints[:,0], predictedPoints[:,1], color='g')
            plt.scatter(chorners[:,0], chorners[:,1], color='r')
            pdb.set_trace()
            
        errorList += (predictedPoints - chorners).ravel().tolist()

    return np.squeeze(errorList)

def fitIntrinsics(initParams, correspondingM, focusList, chornersList, chidsList, A=None): 
    global pixel2world, board
    costList = []
    resList = []

    # Copy the list so don't alter input
    Mlist = correspondingM.copy()
    params = np.copy(initParams).tolist()
    
    rnd = 0
    maxRnd = 5
    
    # Should ultimately change this so is based on change in cost
    while rnd < maxRnd:       
        print('\nRound: ' + str(rnd))
        print('Initial Mlist: ')
        print(Mlist)
        
        rvecs = []
        tvecs = []
        # Update Mlist
        for i in range(len(chornersList)):
            chorners = np.squeeze(chornersList[i])
            chids = np.squeeze(chidsList[i])
            
            defaultM = Mlist[i]
            focus = focusList[i]
            
            rvec, tvec, cameraMatrixList, _, _ = \
                predictPoseFromModel(pixel2world, params, defaultM, focus, chorners, chids, board)
            q = pixel2world * cameraMatrixList[-1][0,0]

            rvecs.append(np.squeeze(rvec))
            tvecs.append(np.squeeze(tvec))
            
            Mlist[i] = tvec[2] / q
        
        print('Prediction updated Mlist: ')
        print(Mlist)
        
        flattenedPoses = createPose(rvecs, tvecs)
                
        # Update intrinsics
        x0 = np.array(params + flattenedPoses)
          
        print('Intrinsics: ')
        print(x0[:len(params)])
    
        # Using the revised M list, perform levenberg-marquardt to refine the intrinsics parameters
        if A is not None:
            lower = [-np.inf] * len(x0)
            upper = [np.inf] * len(x0)
            lower[0] = 0.185
            upper[0] = 0.195
            result = scipy.optimize.least_squares(objFunc, x0, method='trf', jac_sparsity = A, x_scale='jac', ftol=1e-4, bounds=(lower, upper), args=([Mlist, focusList, chornersList, chidsList]))
        else:            
            result = scipy.optimize.least_squares(objFunc, x0, method='lm', x_scale = 'jac', ftol=1e-4,  args=([Mlist, focusList, chornersList, chidsList]))

        params = result.x[:len(params)].tolist()
        print('Updated Intrinsics: \n', params)
        
        print('Cost: ', result.cost)
        print('Per-point-component average abs difference')
        print(np.mean(np.abs(result.fun)))
        costList.append(result.cost)
        resList.append(result.fun)
        
        print('Status: ', result.status)
        
        # Now, recompute M at this step since have updated intrinsics and extrinsics
        modRvecs, modTvecs = decomposePose(result.x[len(params):])
        modRvecs = np.squeeze(modRvecs)
        modTvecs = np.squeeze(modTvecs)
        
        modMlist = Mlist.copy()
        
        for i, tvec in enumerate(modTvecs):
            cameraMatrix, dist = constructIntrinsics(params, Mlist[i], focusList[i])
            q = pixel2world * cameraMatrix[0,0]
            
            modMlist[i] = tvec[2] / q
        
        print('BA Mlist: ')
        print(modMlist)
        
        Mlist = modMlist
        
        rnd += 1
    
    # Goal: Do modRvecs, modTvecs agree wih 
    # pdb.set_trace()
    
    return params, rvecs, tvecs, Mlist, costList, resList


def fitIntrinsicsThinLensOnce(initParams, correspondingM, focusList, chornersList, chidsList, A=None): 
    global pixel2world, board
    costList = []
    resList = []

    # Copy the list so don't alter input
    Mlist = correspondingM.copy()
    params = np.copy(initParams).tolist()
    
    rnd = 0
    maxRnd = 5
    
    # Get initial guess for extrinsics using prediction with the default/input Mlist
    # Also update Mlist once done
    
    print('Default Mlist: ')
    print(Mlist)
    
    rvecs = []
    tvecs = []
        
    # Update Mlist
    for i in range(len(chornersList)):
        chorners = np.squeeze(chornersList[i])
        chids = np.squeeze(chidsList[i])
        
        defaultM = Mlist[i]
        focus = focusList[i]
        
        rvec, tvec, cameraMatrixList, _, _ = \
            predictPoseFromModel(pixel2world, params, defaultM, focus, chorners, chids, board)
        q = pixel2world * cameraMatrixList[-1][0,0]

        rvecs.append(rvec)
        tvecs.append(tvec)
        
        Mlist[i] = tvec[2] / q

    flattenedPoses = createPose(rvecs, tvecs)
            
    # Update intrinsics
    x0 = np.array(params + flattenedPoses)
     
    # TODO: Should ultimately change this criteria to something better
    while rnd < maxRnd:
        print('Round: ' + str(rnd))

        print('Mlist: ')
        print(Mlist)
                
        print('Intrinsics: ')
        print(x0[:len(params)])
        
        # Using the revised M list, perform levenberg-marquardt to refine the intrinsics parameters
        if A is not None:
            result = scipy.optimize.least_squares(objFunc, x0, method='trf', jac_sparsity = A, x_scale='jac', ftol=1e-4,  args=([Mlist, focusList, chornersList, chidsList]))
        else:            
            result = scipy.optimize.least_squares(objFunc, x0, method='lm', x_scale = 'jac', ftol=1e-4,  args=([Mlist, focusList, chornersList, chidsList]))

        params = result.x[:len(params)]
        
        print('Updated Intrinsics: \n', params)
        
        print('Cost: ', result.cost)
        print('Per-point-component average abs difference')
        print(np.mean(np.abs(result.fun)))
        costList.append(result.cost)
        resList.append(result.fun)
        
        print('Status: ', result.status)
        
        # Now, recompute M at this step since have updated intrinsics and extrinsics
        rvecs, tvecs = decomposePose(result.x[len(params):])
        rvecs = np.squeeze(rvecs)
        tvecs = np.squeeze(tvecs)
        
        for i, tvec in enumerate(tvecs):
            cameraMatrix, dist = constructIntrinsics(params, Mlist[i], focusList[i])
            q = pixel2world * cameraMatrix[0,0]
            
            Mlist[i] = tvec[2] / q
        
        # This x will serve as initial guess for next round but now have changed Mlist
        x0 = result.x
        
        rnd += 1
    
    return params, rvecs, tvecs, Mlist, costList, resList

def reprojError(board, chorners, chids, rvec, tvec, cameraMatrix, dist):
    R = cv2.Rodrigues(rvec)[0]
    
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))

    predictedPoints = projectPoints(cameraMatrix, dist, R, tvec, objectPoints)
    errorVecs = predictedPoints - chorners
    error = np.mean(np.linalg.norm(errorVecs, axis=1))

    return errorVecs, error, predictedPoints 

def getInitialGuess(trainingFocus, trainingIntrinsics, trainingDistortion, degList):
    # Initial guess based on polynomial fit
    
    F = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
    u0 = [camMatrix[0,2] for camMatrix in trainingIntrinsics]
    v0 = [camMatrix[1,2] for camMatrix in trainingIntrinsics]
    k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
    k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
    p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
    p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
    k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]
    
    parameterSet = [F, u0, v0, k1, k2, p1, p2, k3]
        
    initPolyModel = []
    for i, parameter in enumerate(parameterSet):
        # Don't add anything for degree of -1
        if degList[i] >= 0:
            initPolyModel.append(np.polyfit(trainingFocus, parameter, degList[i]))
    
    return initPolyModel

def predictPoseFromIntrinsics(cameraMatrix, dist, chorners, chids, board):
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    chorners = np.squeeze(chorners)
        
    retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, cameraMatrix, dist)
    
    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)
    
    return rvec, tvec


# cameraMatrixLists should be a list of lists where each inner list is one
# model's predicted cameraMatrices across the focus positions
def plotParametersFromList(cameraMatrixLists, distLists, correspondingFocus, legend=None):
    
    # Number of models, number of focus positions, number of parameters
    parameterLists = np.zeros((len(cameraMatrixLists), len(correspondingFocus), 8))
    parameterNames = ['F', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
    
    for i in range(len(cameraMatrixLists)):
        cameraMatrixList = cameraMatrixLists[i]
        distList = distLists[i]
        
        for j in range(len(correspondingFocus)):
            cameraMatrix = cameraMatrixList[j]
            dist = distList[j]
            
            parameterLists[i,j,0] = cameraMatrix[0,0]
            parameterLists[i,j,1] = cameraMatrix[0,2]
            parameterLists[i,j,2] = cameraMatrix[1,2]
            parameterLists[i,j,3] = dist[0,0]
            parameterLists[i,j,4] = dist[0,1]
            parameterLists[i,j,5] = dist[0,2]
            parameterLists[i,j,6] = dist[0,3]
            parameterLists[i,j,7] = dist[0,4]
    
    for k in range(8):
        plt.figure()
        plt.title(parameterNames[k] + ' across Focus Positions')
        for i in range(len(cameraMatrixLists)):
            if legend is not None:
                plt.scatter(correspondingFocus, parameterLists[i, :, k], label=legend[i])
            else:
                plt.scatter(correspondingFocus, parameterLists[i, :, k], label=legend[i])
        plt.xlabel('Focus Position')
        plt.ylabel(parameterNames[k])
        plt.legend()
    
if __name__ == '__main__':

    plt.close('all')
    plt.rcParams["scatter.marker"] = 'o'
    
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
    # Load Images from Path  #
    ##########################
        
    # Need to separate the imperx and ximea
    cameraName = 'ximea' # or imperx
    
    # Vertical, Horizontal
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    
    #f = 200 * 1e-3 # / 1.0372608996079156
    #f = 0.1888
    
    # Vertical, Horizontal
    imageShape = (6004, 7920)

    pixel2world = sensor_size[0] / imageShape[0] 
    
    # Get polynomial fit for reference
    
    path = 'C:/Users/aofeldman/Desktop/testCollection9-11'   
    
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 12
    trainingRounds = set(range(numRounds+1))
    testRounds = []    
    
    # First, load the fixed models using loadFixedModels
    intrinsicsList, distortionList, focusList, roundList = \
        loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
    
    trainingIntrinsics = intrinsicsList[0]
    trainingDistortion = distortionList[0]
    trainingFocus = focusList[0]
    
    # F, u0, v0, k1, k2, p1, p2, k3
    degList = [1, 0, 0, 1, 1, 1, 1, 1]
    
    initPolyModel = getInitialGuess(trainingFocus, trainingIntrinsics, trainingDistortion, degList)
    


    #path = 'C:/Users/aofeldman/Desktop/testCollectionRefined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection7-21Refined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection8-10Refined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection8-31'
    #path = 'C:/Users/aofeldman/Desktop/testCollectionCombined'
    #path = 'C:/Users/aofeldman/Desktop/testCollection9-11'
    path = 'C:/Users/aofeldman/Desktop/knownZexpanded/rightForwardNoOutlierRemapped'    
    
    POSTIT = True
    
    # EVALUATE = False
    
    # subfolderPrefixes = 'AFround'
    subfolderPrefixes = ''
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 18
    trainingRounds = set(range(numRounds+1))    
    
    focusList = []
    # roundList = []
    for i in trainingRounds:
        foldername = path #  + '/' + subfolderPrefixes
        
        # Skip over AF rounds where did not collect any images
        try:
            focus = searchForFocus(foldername + '/' + relFocusPrefix + str(i) + '.txt')
            focusList.append(focus)
            # roundList.append(i)
            
        except FileNotFoundError:
            print('Could not find focus file, skipping round ' + str(i))

    chornersList = []
    chidsList = []
    correspondingFocus = []
    correspondingM = []
    
    #for i, AFround in enumerate(roundList):
        # print('\nLoading in round ' + str(i))
        
    foldername = path  # + '/' + subfolderPrefixes + str(AFround)

    # Load in all images for the given focus setting
    filenames = glob.glob(foldername + '/' + '*.tif')
    
    imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
    filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
    imageNumbers = np.sort(imageNumbers)
    
    # Read in and mildly blur images
    image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
        
    print('Number of images: ' + str(len(image_list)))
    
    for j, image in enumerate(image_list):
        print('On image: ' + str(j))
        
        parameters = cv2.aruco.DetectorParameters_create()
    
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                 
            chornersList.append(chorners)
            chidsList.append(chids)
            correspondingFocus.append(focusList[j])
    
    initParams = [0.1888, imageShape[1] // 2, imageShape[0] // 2, 0, 0]
    # Create the sparsity map
    # A = sparsityMap(initParams, chornersList, True)
    
    # For an initial guess at M, a linear regression against 
    # focus position can be used
    correspondingM = 0.0261 * np.array(correspondingFocus) + 8.4172

    # Make an initial guess, assume constant center
    revParams, rvecs, tvecs, Mlist, costList, resList = \
    fitIntrinsics(initParams, correspondingM, correspondingFocus, chornersList, chidsList) #, A)
    
    rvecs = np.squeeze(rvecs)
    tvecs = np.squeeze(tvecs)
    
    # This approach now gives the same results, and is simpler
    # print('\n\nStarting second approach')
    
    # modRevParams, modRvecs, modTvecs, modMlist, modCostList, modResList = \
    # fitIntrinsicsThinLensOnce(initParams, correspondingM, focusList, chornersList, chidsList)
    
    # modRvecs = np.squeeze(modRvecs)
    # modTvecs = np.squeeze(modTvecs)
    
    #pdb.set_trace()
    
    # meanErrList = []
    # collinearityErrList = []
    # predMlist = []
    # predRvecs = []
    # predTvecs = []
    
    # if True:
    #     pointsPerImage = [len(chorners) for chorners in chornersList]
        
    #     offset = 0
    #     residuals = resList[-1]
        
        
    #     for i, shift in enumerate(pointsPerImage):
    #         # Get re-projection error
    #         chorners = np.squeeze(chornersList[i])
    #         chids = np.squeeze(chidsList[i])
    #         move = 2 * shift
    #         errVecs = np.reshape(residuals[offset:offset + move], (-1, 2))
    #         errors = np.linalg.norm(errVecs, axis=1)
    #         print('Mean Re-projection Error for Image ' + str(i+1) + ' = ' + str(np.mean(errors)))
    #         offset += move
    #         M = Mlist[i]
    #         focus = correspondingFocus[i]
    #         cameraMatrix, dist = constructIntrinsics(revParams, M, focus)
            
    #         rvec = rvecs[i]
    #         tvec = tvecs[i]
    #         R = cv2.Rodrigues(rvec)[0]
            
    #         objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    
    #         predictedPoints = projectPoints(cameraMatrix, dist, R, tvec, objectPoints)
    #         error = np.mean(np.linalg.norm(predictedPoints - chorners, axis=1))
    #         print('Direct Computation of Reproj Error', error)
            
    #         if True:
    #             plt.figure()
    #             plt.scatter(predictedPoints[:,0], predictedPoints[:,1], color='g')
    #             plt.scatter(chorners[:,0], chorners[:,1], color='r')
    #             plt.title('Image ' + str(i+1))
            
    #         collinearity = computeCollinearity(cameraMatrix, dist, chorners, chids, imageShape, False)[0]

    #         print('Collinearity: ' + str(collinearity))
    
    #         meanErrList.append(np.mean(errors))
    #         collinearityErrList.append(collinearity)
            
    #         print('Starting online prediction mode')
    #         defaultM = 0.0261 * focus + 8.4172
    #         rvec, tvec, cameraMatrixList, distList, refiningM = \
    #             predictPoseFromModel(pixel2world, revParams, defaultM, focus, chorners, chids, board)
    #         cameraMatrix = cameraMatrixList[-1]
    #         dist = distList[-1]
    #         predMlist.append(refiningM[-1])
    #         predRvecs.append(rvec)
    #         predTvecs.append(tvec)
            
    # predRvecs = np.squeeze(predRvecs)
    # predTvecs = np.squeeze(predTvecs) 

    # First index = round, second index = initial, refined, or polyfit
    # Stores the reprojection error for each image
    meanErrList = np.zeros((len(chornersList), 3))

    initCameraMatrixList = []
    initDistList = []
    
    revCameraMatrixList = []
    revDistList = []
    
    polyCameraMatrixList = []
    polyDistList = []

    for i in range(len(chornersList)):
        chorners = np.squeeze(chornersList[i])
        chids = np.squeeze(chidsList[i])
        focus = correspondingFocus[i]
        defaultM = 0.0261 * focus + 8.4172
        
        rvec, tvec, cameraMatrixList, distList, refiningM = \
            predictPoseFromModel(pixel2world, initParams, defaultM, focus, chorners, chids, board)
            
        initCameraMatrixList.append(cameraMatrixList[-1])
        initDistList.append(distList[-1])
        
        meanErrList[i, 0] = reprojError(board, chorners, chids, rvec, tvec, cameraMatrixList[-1], distList[-1])[1]
        
        rvec, tvec, cameraMatrixList, distList, refiningM = \
            predictPoseFromModel(pixel2world, revParams, defaultM, focus, chorners, chids, board)
        
        revCameraMatrixList.append(cameraMatrixList[-1])
        revDistList.append(distList[-1])
        
        _, error, predictedPoints = reprojError(board, chorners, chids, rvec, tvec, cameraMatrixList[-1], distList[-1])
            
        if True:
            plt.figure()
            plt.scatter(predictedPoints[:,0], predictedPoints[:,1], color='g')
            plt.scatter(chorners[:,0], chorners[:,1], color='r')
            plt.title('Image ' + str(i+1))
        
        meanErrList[i, 1] = error
        
        polyK, polyDist = evaluatedModel(initPolyModel, focus)
        
        polyCameraMatrixList.append(polyK)
        polyDistList.append(polyDist)
        
        rvec, tvec = predictPoseFromIntrinsics(polyK, polyDist, chorners, chids, board)
        
        meanErrList[i, 2] = reprojError(board, chorners, chids, rvec, tvec, polyK, polyDist)[1]
    
    legend = ['Initial Thin Lens Fit', 'Refined Thin Lens Fit', 'Polynomial Fit']
    plt.figure()
    plt.title('Re-projection Error across Focus Positions')
    for i in range(meanErrList.shape[1]):
        plt.scatter(correspondingFocus, meanErrList[:,i], label=legend[i])
    plt.xlabel('Focus Position')
    plt.ylabel('Re-projection Error')
    plt.legend()
    
    plotParametersFromList([initCameraMatrixList, revCameraMatrixList, polyCameraMatrixList], 
                           [initDistList, revDistList, polyDistList], correspondingFocus, legend)
    

    if POSTIT:
        # Load in the images from appropriate directory
        foldername = 'C:/Users/aofeldman/Desktop/knownZexpanded/frontFacingNoOutlier'    
        postItRounds = 18
        
        # Load in all images for the given focus setting
        filenames = glob.glob(foldername + '/' + '*.tif')
        
        imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
        filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
        imageNumbers = np.sort(imageNumbers)
        
        expectedDist = (imageNumbers - 1) * 3 * 2.54 / 100
        
        # Read in and mildly blur images
        image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
            
        print('Number of images: ' + str(len(image_list)))
        
        postItChorners = []
        postItChids = []
        postItFocus = []
        
        for i, image in enumerate(image_list):
            print('On image: ' + str(imageNumbers[i]))
            
            parameters = cv2.aruco.DetectorParameters_create()
        
            markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
            
            # if we dont find enough points skip
            if (ids is not None and len(ids) > 8):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                     
                postItChorners.append(chorners)
                postItChids.append(chids)
                postItFocus.append(searchForFocus(foldername + '/AF' + str(imageNumbers[i]) + '.txt'))
        
        # First index = image, second index = model (initial, refined, or poly), third index = component
        poses = np.zeros((len(postItChorners), 3, 6))
        
        for i in range(len(postItChorners)):
            chorners = np.squeeze(postItChorners[i])
            chids = np.squeeze(postItChids[i])
            focus = postItFocus[i]
            
            defaultM = 0.0261 * focus + 8.4172
            
            for j, params in enumerate([initParams, revParams]):
                rvec, tvec, cameraMatrixList, distList, refiningM = \
                    predictPoseFromModel(pixel2world, params, defaultM, focus, chorners, chids, board)
                
                poses[i,j,:3] = rvec
                poses[i,j,3:] = tvec
                
            polyK, polyDist = evaluatedModel(initPolyModel, focus)
            rvec, tvec = predictPoseFromIntrinsics(polyK, polyDist, chorners, chids, board)
            poses[i,2,:3] = rvec
            poses[i,2,3:] = tvec
                
            
        legend = ['Initial Thin Lens Fit', 'Refined Thin Lens Fit', 'Polynomial Fit']
        plt.figure()            
        plt.scatter(imageNumbers - 1, expectedDist, label= 'Measured')
        plt.title('Distance Shift Estimated versus Measured')

        for i in range(poses.shape[1]):
            # trans = poses[:, i, 3:]
            # print('Using ' + legend[i])
            # distances = np.linalg.norm(trans, axis=1)
            
            # poly = np.polyfit(imageNumbers - 1, distances, 1)
            # print('Approximate Estimated Slope: ', poly[0])
            # print('Approximate Estimated Intercept: ', poly[1])
            
            # #relDist = distances - poly[1]
            # relDist = distances - distances[0]
            
            relDist = np.linalg.norm(poses[:, i, 3:] - poses[0, i, 3:], axis=1)
            
            plt.scatter(imageNumbers - 1, relDist, label=legend[i])
        plt.legend()
        
        legend = ['Initial Thin Lens Fit', 'Refined Thin Lens Fit', 'Polynomial Fit']
        plt.figure()            
        plt.title('Distance Shift Expected - Estimated')

        for i in range(poses.shape[1]):
            #trans = poses[:, i, 3:]
            #print('Using ' + legend[i])
            #distances = np.linalg.norm(trans, axis=1)
            
            
            #poly = np.polyfit(imageNumbers - 1, distances, 1)
            #print('Approximate Estimated Slope: ', poly[0])
            #print('Approximate Estimated Intercept: ', poly[1])
            
            # Just subtract the first entry, no polyfit intercept business
            #relDist = distances - poly[1]
            
            relDist = np.linalg.norm(poses[:, i, 3:] - poses[0, i, 3:], axis=1)
            plt.scatter(imageNumbers - 1, expectedDist - relDist, label=legend[i])
        plt.legend()
        
        
        # Repeat but only using Z-component
        
        plt.figure()            
        plt.scatter(imageNumbers - 1, expectedDist, label='Measured')
        plt.title('Z Shift Estimated versus Measured')

        for i in range(poses.shape[1]):
            Z = poses[:, i, -1]
            print('Using ' + legend[i])
            
            poly = np.polyfit(imageNumbers - 1, Z, 1)
            print('Approximate Estimated Slope: ', poly[0])
            print('Approximate Estimated Intercept: ', poly[1])
            
            #relDist = Z - poly[1]
            
            relZ = Z - Z[0]
            
            plt.scatter(imageNumbers - 1, relZ, label=legend[i])
        plt.legend()
        
        legend = ['Initial Thin Lens Fit', 'Refined Thin Lens Fit', 'Polynomial Fit']
        plt.figure()            
        plt.title('Z Shift Expected - Estimated')

        for i in range(poses.shape[1]):
            Z = poses[:, i, -1]
            print('Using ' + legend[i])
            
            poly = np.polyfit(imageNumbers - 1, Z, 1)
            print('Approximate Estimated Slope: ', poly[0])
            print('Approximate Estimated Intercept: ', poly[1])
            
            #relDist = Z - poly[1]
        
            relZ = Z - Z[0]
            
            plt.scatter(imageNumbers - 1, expectedDist - relZ, label=legend[i])
        plt.legend()
        
        
        # legend = ['Initial Thin Lens Fit', 'Refined Thin Lens Fit', 'Polynomial Fit']
        # fig = plt.figure()            
        # plt.title('Visualizing Estimated Translations')
        # ax = fig.add_subplot(111, projection='3d')

        # boardTrans = []
        # # Do a 3d plot to better visualize the estimated motion
        # for i in range(1):
        #     for j in range(poses.shape[0]):
        #         tvec = poses[j,i,3:]
        #         rvec = poses[j,i,:3]
        #         R = cv2.Rodrigues(rvec)[0]
        #         boardTrans.append(-R.T @ tvec)
        
        # boardTrans = np.squeeze(boardTrans)
        # ax.scatter(boardTrans[:,2], boardTrans[:,0], boardTrans[:,1])
        # ax.set_xlabel('Zboard [m]')
        # ax.set_ylabel('Xboard [m]')
        # ax.set_zlabel('Yboard [m]')
        
            
            
       
        
       
        
       
        
       
        
       
        
       
        
       
        
       
        
       
    # ##################
    # # Evaluation     #
    # ##################
    
    # if EVALUATE:
    #     testRounds = 12
    #     trainingRounds = set(range(0, testRounds+1))
    #     testRounds = []
        
    #     testPath = 'C:/Users/aofeldman/Desktop/testCollection9-11'
    #     testPrefixes = 'AFround'
        
    #     # First, load the fixed models using loadFixedModels
    #     intrinsicsList, distortionList, focusList, roundList = \
    #         loadFixedModels(testPath, testPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)    
        
    #     trainingIntrinsics = intrinsicsList[0]
    #     trainingDistortion = distortionList[0]
    #     trainingFocus = focusList[0]
        
    #     F = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
    #     u0 = [camMatrix[0,2] for camMatrix in trainingIntrinsics]
    #     v0 = [camMatrix[1,2] for camMatrix in trainingIntrinsics]
    #     k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
    #     k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
    #     p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
    #     p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
    #     k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]
        
    #     polyModel = []
    #     polyModel.append(np.polyfit(trainingFocus, F, 1))
    #     polyModel.append(np.polyfit(trainingFocus, u0, 0))
    #     polyModel.append(np.polyfit(trainingFocus, v0, 0))
    #     polyModel.append(np.polyfit(trainingFocus, k1, 1))
    #     polyModel.append(np.polyfit(trainingFocus, k2, 1))
    #     polyModel.append(np.polyfit(trainingFocus, p1, 1))
    #     polyModel.append(np.polyfit(trainingFocus, p2, 1))
    #     polyModel.append(np.polyfit(trainingFocus, k3, 1))
        
    #     testErrList = [[], [], []]
    #     avgBundleF = []
    #     avgBundleK = []
        
    #     for i, AFround in enumerate(roundList[0]):
    #         pointErr = []
    #         polyErr = []
    #         bundleErr = []
            
    #         print('Reading in round ' + str(AFround))
    #         foldername = testPath + '/' + testPrefixes + str(AFround)
        
    #         filenames = glob.glob(foldername + '/' + '*.tif')
            
    #         # Load in 3 random images for the given focus setting
    #         filenames = np.random.choice(filenames, 3).tolist()
            
    #         imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
    #         filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
    #         imageNumbers = np.sort(imageNumbers)
            
    #         # Read in and mildly blur images
    #         image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
            
    #         testChorners = []
    #         testChids = []
    #         # Given the image list identify the chorners once
    #         for image in image_list:
    #             parameters = cv2.aruco.DetectorParameters_create()
                
    #             markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
                
    #             # if we dont find enough points skip
    #             if (ids is not None and len(ids) > 8):
    #                 ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
    #                 testChorners.append(chorners)
    #                 testChids.append(chids)
              
    #         bundleF = []
    #         bundleK = []
            
    #         for j in range(len(testChorners)):
    #             chorners = np.squeeze(testChorners[j])
    #             chids = np.squeeze(testChids[j])
    #             focus = trainingFocus[i]
                
    #             objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
                
    #             retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, 
    #                 trainingIntrinsics[i], trainingDistortion[i], None, None, False)
    #             rvec = np.squeeze(rvec)
    #             tvec = np.squeeze(tvec)
                
        
    #             pointErr.append(reprojError(board, chorners, chids,
    #                             rvec, tvec, trainingIntrinsics[i], trainingDistortion[i])[1])
                
    #             polyK, polyDist = evaluatedModel(polyModel, focus)
                
    #             retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, 
    #                 polyK, polyDist, None, None, False)
    #             rvec = np.squeeze(rvec)
    #             tvec = np.squeeze(tvec)
                
    #             polyErr.append(reprojError(board, chorners, chids, rvec, tvec, polyK, polyDist)[1])
                
    #             defaultM = 0.0261 * focus + 8.4172
    #             rvec, tvec, cameraMatrixList, distList, refiningM = \
    #                 predictPoseFromModel(pixel2world, revParams, defaultM, focus, chorners, chids, board)
    #             cameraMatrix = cameraMatrixList[-1]
    #             dist = distList[-1]
                
    #             bundleF.append(cameraMatrix[0,0])
    #             bundleK.append(dist[0,0])
                
    #             rvec = np.squeeze(rvec)
    #             tvec = np.squeeze(tvec)
                
    #             bundleErr.append(reprojError(board, chorners, chids,
    #                             rvec, tvec, cameraMatrix, dist)[1])
            
    #         avgBundleF.append(np.mean(bundleF))
    #         avgBundleK.append(np.mean(bundleK))
            
    #         testErrList[0].append(np.mean(pointErr))
    #         testErrList[1].append(np.mean(polyErr))
    #         testErrList[2].append(np.mean(bundleErr))
        
    #     legend = ['Point', 'Polynomial Fit', 'Bundle Adjustment']
    #     plt.figure()
    #     for i in range(len(testErrList)):
    #         plt.scatter(trainingFocus, testErrList[i], label=legend[i])
    #     plt.xlabel('Focus Position')
    #     plt.ylabel('Average Re-projection Error across Random Subset')
    #     plt.title('Comparing Re-projection Error')
    #     plt.legend()
        
    #     # What does F look like for each of the different results
    #     plt.figure()
    #     plt.scatter(trainingFocus, F, label=legend[0])
    #     polyF = [evaluatedModel(polyModel, focus)[0][0,0] for focus in trainingFocus]
    #     plt.scatter(trainingFocus, polyF, label=legend[1])
    #     plt.scatter(trainingFocus, avgBundleF, label=legend[2])
    #     plt.xlabel('Focus Position')
    #     plt.ylabel('F: Effective Focal Length (Pixel Units)')
    #     plt.legend()
    #     plt.title('Comparing F across Models')
        
    #     # What does k1 look like for each of the different results
    #     plt.figure()
    #     plt.scatter(trainingFocus, k1, label=legend[0])
    #     polyK = [evaluatedModel(polyModel, focus)[1][0,0] for focus in trainingFocus]
    #     plt.scatter(trainingFocus, polyK, label=legend[1])
    #     plt.scatter(trainingFocus, avgBundleK, label=legend[2])
    #     plt.xlabel('Focus Position')
    #     plt.ylabel('k1')
    #     plt.legend()
    #     plt.title('Comparing k1 across Models')