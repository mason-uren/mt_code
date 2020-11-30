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
#from experimenting import altThinLensPose

##########################
# Helper Functions       #
##########################

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

def predictPoseFromIntrinsics(cameraMatrix, dist, chorners, chids, board):
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    chorners = np.squeeze(chorners)
        
    retval, rvec, tvec = cv2.solvePnP(objectPoints, chorners, cameraMatrix, dist)
    
    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)
    
    return rvec, tvec

# point = (x,y), line = (nx, ny, d0) st. nx x' + ny y' = d0 for x',y' on line
def projPoint(point, line, graph=False):
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

# Assumes already undistorted
def assessDistortion(chorners, chids, K, graph=False, image=None):
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
            
            point1 = projPoint((0, 0), (nx, ny, d0))
            point2 = projPoint((7920, 6004), (nx, ny, d0))
            
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
        proj = projPoint(undistorted[i], (nx, ny, d0))
        errVecs += (proj - undistorted[i]).tolist()
        error += np.linalg.norm(proj - undistorted[i])**2
    
    return nx, ny, d0, error, errVecs

def computeCollinearity(cameraMatrix, dist, chorners, chids, graph=False):
    global imageShape
    #print('cameraMatrix', cameraMatrix)
    #print('dist', dist)
    #print('shape chorners', np.shape(chorners))
    #print('shape chids', np.shape(chids))
    #if len(np.shape(chorners)) != 3:
    #    modChorners = np.expand_dims(chorners, axis=1)
    #else:
    #    modChorners = np.copy(chorners)
    modChorners = np.copy(chorners)
    undistorted = cv2.undistortPoints(modChorners, cameraMatrix, dist, np.eye(3), cameraMatrix)
    undistorted = np.squeeze(undistorted)
    # pdb.set_trace()
    if graph:
        plt.figure()
        plt.title('Undistorted')
        plt.scatter(undistorted[:,0], undistorted[:,1], label='Undistorted')
        plt.scatter(np.squeeze(chorners)[:,0], np.squeeze(chorners)[:,1], label='Original')
        plt.legend()
    return assessDistortion(undistorted, chids, cameraMatrix, False, np.zeros(imageShape).astype(np.float32))    
    
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
def objFunc(x, focusList, chornersList, chidsList, degList, graph=False):
    global board
    
    # Because for each have deg + 1 as number of terms
    paramLength = len(degList) + sum(degList)
    
    # Will make -1 deg correspond to constant 0 fit for that parameter
    # so later evaluatedModel should not have a problem
    polyModel = paramToPolyModel(x[:paramLength], degList)
    
    rvecs, tvecs = decomposePose(x[paramLength:])
    
    Rlist = []
    for rvec in rvecs:
        R, _ = cv2.Rodrigues(rvec)
        Rlist.append(R)
    
    Tlist = tvecs.tolist() 
    
    cameraMatrixList = []
    distortionList = []
    
    errorList = []
    # Construct the intrinsics and distortion matrices for each image
    for i in range(len(chornersList)):
        focus = focusList[i]
        cameraMatrix, dist = evaluatedModel(polyModel, focus)
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


# initPolyModel can contain fits for only some of the parameters
# However, degList should specify degree for all parameters, using -1 if
# wish to fix that parameter at 0
# Will return a modified polyModel, featuring 0 constant fit for the -1 deg
def fitIntrinsics(initPolyModel, focusList, chornersList, chidsList, degList, A=None): 
    global board

    # Copy the list so don't alter input
    polyModel = initPolyModel.copy()
    
    rvecs = []
    tvecs = []

    # Make an initial guess for rvec, tvec using the starting parameters
    for i in range(len(chornersList)):
        chorners = np.squeeze(chornersList[i])
        chids = np.squeeze(chidsList[i])
        
        focus = focusList[i]
        
        cameraMatrix, dist = evaluatedModel(completePolyModel(polyModel, degList), focus)
        
        rvec, tvec = predictPoseFromIntrinsics(cameraMatrix, dist, chorners, chids, board)

        rvecs.append(np.squeeze(rvec))
        tvecs.append(np.squeeze(tvec))
        
    flattenedPoses = createPose(rvecs, tvecs)
            
    params = polyModelToParam(polyModel)
    
    # Initialize intrinsics
    x0 = np.array(params.tolist() + flattenedPoses)
      
    print('Intrinsics: ')
    print(x0[:len(params)])

    # Perform nonlinear optimization with objective of minimizing re-projection
    # error to refine the intrinsics parameters
    if A is not None:
        # Can adjust these bounds as desired
        lower = [-np.inf] * len(x0)
        upper = [np.inf] * len(x0)
        
        result = scipy.optimize.least_squares(objFunc, x0, method='trf', jac_sparsity = A, ftol=1e-6, x_scale='jac', verbose=2, bounds=(lower, upper), args=([focusList, chornersList, chidsList, degList]))
    else:            
        result = scipy.optimize.least_squares(objFunc, x0, method='lm', x_scale = 'jac',  ftol=1e-6, args=([focusList, chornersList, chidsList, degList]))

    params = result.x[:len(params)].tolist()
    
    polyModel = paramToPolyModel(params, degList)
    print('Updated Intrinsics: \n', polyModel)
    
    rvecs, tvecs = decomposePose(result.x[len(params):])
    
    print('Cost: ', result.cost)
    print('Per-point-component average abs difference')
    print(np.mean(np.abs(result.fun)))    
    print('Status: ', result.status)
    
    return polyModel, rvecs, tvecs, result.cost, result.fun


def reprojError(board, chorners, chids, rvec, tvec, cameraMatrix, dist):
    R = cv2.Rodrigues(rvec)[0]
    
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))

    predictedPoints = projectPoints(cameraMatrix, dist, R, tvec, objectPoints)
    errorVecs = predictedPoints - chorners
    error = np.mean(np.linalg.norm(errorVecs, axis=1))

    return errorVecs, error, predictedPoints  
    

# Given flattened list of parameters and degrees used for fitting each parameter
#, convert it to list of numpy arrays
# where each array is polynomial fit for the corresponding parameter
# in descending order.
# If deg is -1, will add a 0 constant fit for that parameter
def paramToPolyModel(params, degList):
    offset = 0
    polyModel = []
    
    for deg in degList:
        numTerms = deg + 1
        if numTerms == 0:
            polyModel.append(np.array([0.0]))
        else:
            polyModel.append(np.array(params[offset:offset + numTerms]))
        offset += numTerms
        
    return polyModel

# Flatten
def polyModelToParam(polyModel):
    return np.concatenate(polyModel).ravel()
    
# Given a polyModel which is missing the 0 constant fit for when have -1 deg
# Add these in so that have a corresponding polynomail for each parameter
# So can use in evaluatedModel
def completePolyModel(polyModel, degList):
    params = polyModelToParam(polyModel)
    return paramToPolyModel(params, degList)

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

# Plot the point estimate and the various fits considered
def plotFits(trainingFocus, trainingIntrinsics, trainingDistortion, polyModels, plotPoint=True, legend=None):
    
    # First plot the point estimates

    if len(polyModels[0]) == 8:
        f = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
    elif len(polyModels[0]) == 9:
        fx = [camMatrix[0,0] for camMatrix in trainingIntrinsics]
        fy = [camMatrix[1,1] for camMatrix in trainingIntrinsics]
    else:
        raise IndexError("polyModel should have a polynomial for each of the 8 or 9 parameters")
        
    u0 = [camMatrix[0,2] for camMatrix in trainingIntrinsics]
    v0 = [camMatrix[1,2] for camMatrix in trainingIntrinsics]
    k1 = [distCoeffs[0,0] for distCoeffs in trainingDistortion]
    k2 = [distCoeffs[0,1] for distCoeffs in trainingDistortion]
    p1 = [distCoeffs[0,2] for distCoeffs in trainingDistortion]
    p2 = [distCoeffs[0,3] for distCoeffs in trainingDistortion]
    k3 = [distCoeffs[0,4] for distCoeffs in trainingDistortion]

    if len(polyModels[0]) == 8:
        parameterNames = ['F', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [f, u0, v0, k1, k2, p1, p2, k3]
    elif len(polyModels[0]) == 9:
        parameterNames = ['Fx', 'Fy', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [fx, fy, u0, v0, k1, k2, p1, p2, k3]
   
    for i, parameter in enumerate(parameterSet):
        plt.figure()
        if plotPoint:
            plt.scatter(trainingFocus, parameter)
            plt.title('Point and Polynomial Fits for ' + parameterNames[i])
        else:
            plt.title('Polynomial Fits for ' + parameterNames[i])
        plt.xlabel('Focus Position')
        plt.ylabel(parameterNames[i])
        
        for j, polyModel in enumerate(polyModels):
            p = polyModel[i]
            plt.scatter(trainingFocus, [np.polyval(p, trainingFocus[i]) for i in range(len(trainingFocus))])
        
        if legend is not None:
            plt.legend(legend)
    

    
if __name__ == '__main__':

    ##########################
    # Plot initialization    #
    ##########################

    plt.close('all')
    plt.rcParams["scatter.marker"] = 'o'
    
    ##########################
    # Performance control    #
    ##########################
    
    # If True, use data from
    # post-it experiments
    # for refinement
    REFINEPOSTIT = True
    
    # If True, evaluate on
    # post-it experiments
    POSTIT = True
    
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
    # Camera Information     #
    ##########################
        
    # Need to separate the imperx and ximea
    cameraName = 'ximea' # or imperx
    
    # Vertical, Horizontal
    sensor_size = np.array([27.6, 36.4]) * 1e-3
        
    # Vertical, Horizontal
    imageShape = (6004, 7920)

    pixel2world = sensor_size[0] / imageShape[0]
    
    ###########################
    # Data Read Info          #
    ###########################
    
    # Fix the seed for choosing "randomly" train and test images
    np.random.seed(0)
    
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
    
    if not REFINEPOSTIT:
        # Read in the images
        numTrain = 2 # Train on x images per round
        numTest = 2 # Test on x images per round
        
        chornersList = []
        chidsList = []
        correspondingFocus = []
        # List of lists where each inner list corresponds to image used in a 
        # given round
        trainImages = []
        
        testChorners = []
        testChids = []
        testImages = []
        testFocus = []
        
        for i, AFround in enumerate(roundList[0]):
            print('On Round: ' + str(AFround))
            
            foldername = path + '/' + subfolderPrefixes + str(AFround)
            
            # Load in all images for the given focus setting
            filenames = glob.glob(foldername + '/' + '*.tif')
            
            # Randomly choose a subset of the images
            filenames = np.random.choice(filenames, numTrain + numTest, replace=False).tolist()
            
            imageNumbers = [int(filenames[i].replace(foldername + '\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
            filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
            imageNumbers = np.sort(imageNumbers)
            
            trainImages.append(imageNumbers[:numTrain])
            testImages.append(imageNumbers[numTrain:])
            
            # Read in and mildly blur images
            image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
                
            print('Number of images: ' + str(len(image_list)))
            
            for j, image in enumerate(image_list):
                # print('On image: ' + str(j))
                parameters = cv2.aruco.DetectorParameters_create()
            
                markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
                
                # if we dont find enough points skip
                if (ids is not None and len(ids) > 8):
                    ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                         
                    if j < numTrain:
                        chornersList.append(chorners)
                        chidsList.append(chids)
                        correspondingFocus.append(focusList[0][i])
                    else:
                        testChorners.append(chorners)
                        testChids.append(chids)
                        testFocus.append(focusList[0][i])
            
        # Create the sparsity map
        A = sparsityMap(polyModelToParam(initPolyModel), chornersList, True)
        
        # Make an initial guess
        # Then, nonlinear refinement via reprojection minimization
        revPolyModel, rvecs, tvecs, cost, residuals = \
        fitIntrinsics(initPolyModel, correspondingFocus, chornersList, chidsList, degList, A)
        
        # Shape numRounds, 2, 3. First index = round, Second index = train or test,
        # Third index = model
        meanErrList = np.zeros((numRounds, 2, 3))
        collinearityErrList = np.zeros((numRounds, 2, 3))
        
        for i, AFround in enumerate(roundList[0]):
            focus = focusList[0][i]
            
            for j in range(2):
                roundError = [[], [], []]
                roundCollinearity = [[], [], []]
                
                if j == 0:
                    imageInfo = zip(chornersList[numTrain * i: numTrain * (i+1)], chidsList[numTrain * i: numTrain * (i+1)])
                else:
                    imageInfo = zip(testChorners[numTest * i: numTest * (i+1)], testChids[numTest * i: numTest * (i+1)])
            
                for chorners, chids in imageInfo:
                    chorners = np.squeeze(chorners)
                    chids = np.squeeze(chids)
                    
                    bundleK, bundleDist = evaluatedModel(revPolyModel, focus)
                    
                    rvec, tvec = predictPoseFromIntrinsics(bundleK, bundleDist, chorners, chids, board)
                    _, error, predictedPoints = reprojError(board, chorners, chids, rvec, tvec, bundleK, bundleDist)            
                    
                    if False:
                        plt.figure()
                        plt.scatter(predictedPoints[:,0], predictedPoints[:,1], color='g')
                        plt.scatter(chorners[:,0], chorners[:,1], color='r')
                        plt.title('Image ' + str(i+1))
                    
                    collinearity = computeCollinearity(bundleK, bundleDist, chorners, chids, False)[0]
                
                    roundError[2].append(error)
                    roundCollinearity[2].append(collinearity)
                    
                    # Also compute results when using point intrinsics and initial polynomial fit
                    polyK, polyDist = evaluatedModel(completePolyModel(initPolyModel, degList), focus)
                    rvec, tvec = predictPoseFromIntrinsics(polyK, polyDist, chorners, chids, board)
                    
                    roundError[1].append(reprojError(board, chorners, chids, rvec, tvec, polyK, polyDist)[1])
                    roundCollinearity[1].append(computeCollinearity(polyK, polyDist, chorners, chids, False)[0])
                    
                    cameraMatrix = intrinsicsList[0][i]
                    dist = distortionList[0][i]
                    rvec, tvec = predictPoseFromIntrinsics(cameraMatrix, dist, chorners, chids, board)
                    
                    roundError[0].append(reprojError(board, chorners, chids, rvec, tvec, cameraMatrix, dist)[1])
                    roundCollinearity[0].append(computeCollinearity(cameraMatrix, dist, chorners, chids, False)[0])
                
                meanErrList[i, j, :] = np.mean(roundError, axis=1)
                collinearityErrList[i, j, :] = np.mean(roundCollinearity, axis=1)
        
        ##################
        # Comparison     #
        ##################
        
        legend = ['Point', 'Initial Polynomial Fit', 'Refined Polynomial Fit']
        
        for j, name in enumerate(['Train', 'Test']):
            plt.figure()
            for k in range(3):
                plt.scatter(trainingFocus, meanErrList[:, j, k], label=legend[k])
            plt.xlabel('Focus Position')
            plt.ylabel('Average Re-projection Error across Random Subset: ' + name)
            plt.title('Comparing Re-projection Error: ' + name)
            plt.legend()
            
            plt.figure()
            for k in range(3):
                plt.scatter(trainingFocus, collinearityErrList[:, j, k], label=legend[k])
            plt.xlabel('Focus Position')
            plt.ylabel('Average Collinearity Error across Random Subset: ' + name)
            plt.title('Comparing Collinearity Error: ' + name)
            plt.legend()
            
        # What does each parameter look like for the different results
        polyModels = [completePolyModel(initPolyModel, degList), revPolyModel]
        plotFits(trainingFocus, trainingIntrinsics, trainingDistortion, polyModels, True, legend)
        
    else:
        # Load in the images from appropriate post-it experiment directory
        foldername = 'C:/Users/aofeldman/Desktop/knownZexpanded/rightBackwardNoOutlier'    
        numRounds = 18
        
        # Load in all images for the given focus setting
        filenames = glob.glob(foldername + '/' + '*.tif')
        
        imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
        filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
        imageNumbers = np.sort(imageNumbers)
        
        expectedDist = (imageNumbers - 1) * 3 * 2.54 / 100
        
        # Read in and mildly blur images
        image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
            
        print('Number of images: ' + str(len(image_list)))
        
        chornersList = []
        chidsList = []
        correspondingFocus = []
        
        for i, image in enumerate(image_list):
            print('On image: ' + str(imageNumbers[i]))
            
            parameters = cv2.aruco.DetectorParameters_create()
        
            markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
            
            # if we dont find enough points skip
            if (ids is not None and len(ids) > 8):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
                     
                chornersList.append(chorners)
                chidsList.append(chids)
                correspondingFocus.append(searchForFocus(foldername + '/AF' + str(imageNumbers[i]) + '.txt'))
                
        # Create the sparsity map
        A = sparsityMap(polyModelToParam(initPolyModel), chornersList, True)
        
        # Make an initial guess
        # Then, nonlinear refinement via reprojection minimization
        revPolyModel, rvecs, tvecs, cost, residuals = \
        fitIntrinsics(initPolyModel, correspondingFocus, chornersList, chidsList, degList, A)
        
        meanErrList = np.zeros((numRounds, 2))
        collinearityErrList = np.zeros((numRounds, 2))
        
        # Now, get re-projection error from original and refined model
        for i in range(len(chornersList)):
            chorners = np.squeeze(chornersList[i])
            chids = np.squeeze(chidsList[i])
            focus = correspondingFocus[i]
            
            bundleK, bundleDist = evaluatedModel(revPolyModel, focus)
            
            rvec, tvec = predictPoseFromIntrinsics(bundleK, bundleDist, chorners, chids, board)
            _, error, predictedPoints = reprojError(board, chorners, chids, rvec, tvec, bundleK, bundleDist)            
            
            if False:
                plt.figure()
                plt.scatter(predictedPoints[:,0], predictedPoints[:,1], color='g')
                plt.scatter(chorners[:,0], chorners[:,1], color='r')
                plt.title('Image ' + str(i+1))
            
            collinearity = computeCollinearity(bundleK, bundleDist, chorners, chids, False)[0]
        
            meanErrList[i, 1] = error
            collinearityErrList[i, 1] = collinearity
            
            # Also compute results when using point intrinsics and initial polynomial fit
            polyK, polyDist = evaluatedModel(completePolyModel(initPolyModel, degList), focus)
            rvec, tvec = predictPoseFromIntrinsics(polyK, polyDist, chorners, chids, board)
            
            meanErrList[i, 0] = reprojError(board, chorners, chids, rvec, tvec, polyK, polyDist)[1]
            collinearityErrList[i, 0] = computeCollinearity(polyK, polyDist, chorners, chids, False)[0]
        
        legend = ['Initial Fit', 'Refined Fit']
        
        plt.figure()
        for k in range(2):
            plt.scatter(correspondingFocus, meanErrList[:, k], label=legend[k])
        plt.xlabel('Focus Position')
        plt.ylabel('Re-projection Error for Each Image')
        plt.title('Comparing Re-projection Error')
        plt.legend()
        
        plt.figure()
        for k in range(2):
            plt.scatter(correspondingFocus, collinearityErrList[:, k], label=legend[k])
        plt.xlabel('Focus Position')
        plt.ylabel('Collinearity Error for Each Image')
        plt.title('Comparing Collinearity Error')
        plt.legend()
            
        # What does each parameter look like for the different results
        polyModels = [completePolyModel(initPolyModel, degList), revPolyModel]
        plotFits(correspondingFocus, [], [], polyModels, False, legend)
        
    
    if POSTIT:
        # Load in the images from appropriate directory
        foldername = 'C:/Users/aofeldman/Desktop/knownZexpanded/rightBackwardNoOutlier'    
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
        
        initRvecs = []
        initTvecs = []
        revRvecs = []
        revTvecs = []
        
        for i in range(len(postItChorners)):
            chorners = np.squeeze(postItChorners[i])
            chids = np.squeeze(postItChids[i])
            focus = postItFocus[i]
            
            bundleK, bundleDist = evaluatedModel(revPolyModel, focus)
            
            rvec, tvec = predictPoseFromIntrinsics(bundleK, bundleDist, chorners, chids, board)
    
            revRvecs.append(rvec)
            revTvecs.append(tvec)
            
            if False:
                plt.figure()
                plt.scatter(predictedPoints[:,0], predictedPoints[:,1], color='g')
                plt.scatter(chorners[:,0], chorners[:,1], color='r')
                plt.title('Image ' + str(i+1))
        
            # Also compute results when using initial polynomial fit
            polyK, polyDist = evaluatedModel(completePolyModel(initPolyModel, degList), focus)
            rvec, tvec = predictPoseFromIntrinsics(polyK, polyDist, chorners, chids, board)
            
            initRvecs.append(rvec)
            initTvecs.append(tvec)
        
        plt.figure()            
        plt.scatter(imageNumbers - 1, expectedDist, label= 'Measured')
        plt.title('Translation Shift Predicted using Initial and Refined Polynomial Fit')

        for i, trans in enumerate([initTvecs, revTvecs]):
            prefix = (1-i) * 'Initial' + i * 'Refined'
            print('Using ' + prefix + ' Polynomial Fit')
            distances = np.linalg.norm(trans, axis=1)
            
            poly = np.polyfit(imageNumbers - 1, distances, 1)
            print('Approximate Estimated Slope: ', poly[0])
            print('Approximate Estimated Intercept: ', poly[1])
            
            relDist = distances - poly[1]
            
            plt.scatter(imageNumbers - 1, relDist, label=prefix + ' Polynomial Fit')
        plt.legend()
        
        
        plt.figure()            
        plt.title('Translation Shift Expected - Predicted using Initial and Refined Polynomial Fit')

        for i, trans in enumerate([initTvecs, revTvecs]):
            prefix = (1-i) * 'Initial' + i * 'Refined'
            distances = np.linalg.norm(trans, axis=1)
            
            poly = np.polyfit(imageNumbers - 1, distances, 1)
           
            relDist = distances - poly[1]
            
            plt.scatter(imageNumbers - 1, expectedDist - relDist, label=prefix + ' Polynomial Fit')
        plt.legend()
        
        

    