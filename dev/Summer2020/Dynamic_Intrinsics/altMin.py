# Used to do alternating gradient-based minimization procedure
# Last Modified: 7/16/2020 Changes made by Aaron
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
import pdb
import scipy.optimize
import math
import time


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
# Helper Functions       #
##########################

class dataPoint:
    def __init__(self, image, focus, rvec, tvec, AFround, chorners, chids):
        self.image = image
        self.focus = focus
        self.rvec = rvec
        self.tvec = tvec
        self.AFround = AFround
        self.chorners = chorners
        self.chids = chids


def simpleEvaluatedModel(f, simpleX0, remainingParam):
    fxa0, fxa1, fxa2, u0, v0 = simpleX0
    k1, k2, p1, p2, k3a0, k3a1 = remainingParam
    
    intrinsics = np.array([[fxa0 * f**2 + fxa1 * f + fxa2, 0, u0], 
                  [0, fxa0* f**2 + fxa1 * f + fxa2, v0],
                  [0, 0, 1]])
    dist = np.expand_dims([k1, k2, p1, p2, k3a0 * f + k3a1], 0)
    
    return intrinsics, dist

def gradSingleImage(simpleX0, remainingParam, dataPoint):
    global board
    grad = np.zeros(np.shape(simpleX0))
    
    fxa0, fxa1, fxa2, u0, v0 = simpleX0
    f = dataPoint.focus

    intrinsics, dist = simpleEvaluatedModel(f, simpleX0, remainingParam)
    
    scaleFocus = fxa0 * f**2 + fxa1 * f + fxa2

    errVecs, err = \
        charuco_reprojection_error_for_every_corner(board, dataPoint.chorners, dataPoint.chids, dataPoint.rvec, dataPoint.tvec, intrinsics, dist)
    # n x 2 shape where each row is of form (uHat - u), (vHat - v)
    pixelDiffs = -1 * np.array(errVecs) # Above method gives back u-uHat form
    
    # n x 2 where each row is uHat, vHat
    predictedPixels = pixelDiffs + np.squeeze(np.array(dataPoint.chorners))
    
    # n x 2 where each row is x'', y''
    xDoublePrimes = 1 / scaleFocus * (predictedPixels - np.array([u0, v0]))
    
    # sum across row first
    baseForm = np.sum(np.sum(pixelDiffs * xDoublePrimes, axis=1) / np.array(err))
    
    grad[0] = 1 / pixelDiffs.shape[0] * baseForm * f**2
    grad[1] = 1 / pixelDiffs.shape[0] * baseForm * f
    grad[2] = 1 / pixelDiffs.shape[0] * baseForm
    grad[3] = 1 / pixelDiffs.shape[0] * np.sum(pixelDiffs[:,0] / np.array(err))
    grad[4] = 1 / pixelDiffs.shape[0] * np.sum(pixelDiffs[:,1] / np.array(err))

    return grad

def gradOverall(simpleX0, remainingParam, dataPointList):
    grad = np.zeros(np.shape(simpleX0))
    for point in dataPointList:
        grad += gradSingleImage(simpleX0, remainingParam, point)
    return grad

def stepIntrinsics(simpleX0, remainingParam, eta, dataPointList):
    global board
    # Can adjust this to be SGD for improved speed
    grad = np.zeros(np.shape(simpleX0))
    for dataPoint in dataPointList:
        indGrad = gradSingleImage(simpleX0, remainingParam, dataPoint)
        err = scipy.optimize.check_grad(singleImageObjFunc, gradSingleImage, simpleX0, remainingParam, dataPoint, epsilon = 1e3 * np.finfo(float).eps)
        print('Error in gradient is: ' + str(err))
        grad += indGrad
    grad = grad / len(dataPointList)
    return eta * grad
    
def stepExtrinsics(simpleX0, remainingParam, dataPointList):
    dataPointListUpdated = []
    for data in dataPointList:
        f = data.focus
        intrinsics, dist = simpleEvaluatedModel(f, simpleX0, remainingParam)
        
        # Seems like rvec, tvec actually vary significantly with the intrinsics parameters
        retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(data.chorners, data.chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
        
        newPoint = dataPoint(data.image, data.focus, rvec, tvec, data.AFround, data.chorners, data.chids)
        dataPointListUpdated.append(newPoint)
    return dataPointListUpdated

def singleImageObjFunc(x0, remainingParam, point):
    global board
    
    start = time.time()
    
    (fxa0, fxa1, fxa2, u0, v0) = x0
    (k1, k2, p1, p2, k3a0, k3a1) = remainingParam
    
    f = point.focus
    intrinsics = np.array([[fxa0 * f**2 + fxa1 * f + fxa2, 0, u0], 
                  [0, fxa0* f**2 + fxa1 * f + fxa2, v0],
                  [0, 0, 1]])
    dist = np.expand_dims([k1, k2, p1, p2, k3a0 * f + k3a1], 0)
            
    return charuco_reprojection_error(board, point.chorners, point.chids,
                                        point.rvec, point.tvec, intrinsics, dist)
        

def objFunc(x0, remainingParam, dataPointList):
    global board
    
    #start = time.time()
    
    (fxa0, fxa1, fxa2, u0, v0) = x0
    (k1, k2, p1, p2, k3a0, k3a1) = remainingParam
    
    totErr = 0

    for point in dataPointList:
        f = point.focus
        intrinsics = np.array([[fxa0 * f**2 + fxa1 * f + fxa2, 0, u0], 
                      [0, fxa0* f**2 + fxa1 * f + fxa2, v0],
                      [0, 0, 1]])
        dist = np.expand_dims([k1, k2, p1, p2, k3a0 * f + k3a1], 0)
                
        totErr += charuco_reprojection_error(board, point.chorners, point.chids,
                                        point.rvec, point.tvec, intrinsics, dist)
    
    #end = time.time()
    #print("Time elapsed: " + str(end - start))

    return 1 / len(dataPointList) * totErr


# Should consider it to have converged when the decrease in error is small
# Potentially use a validation set and early stopping
# Look at how error progresses with iterations to see if overfitting occurs

def alternatingMinimization(simpleX0, remainingParam, dataPointList, batchSize = 10, maxIter = 1000, eta=1e-8):
    iteration = 0
    x = np.copy(simpleX0)
    dataPointListUpdated = dataPointList.copy()
    while iteration < maxIter:
        print('iteration: ' + str(iteration))
        # Alternatingly update the extrinsics and then intrinsics
        dataPointListUpdated = stepExtrinsics(x, remainingParam, dataPointListUpdated)
        if iteration % 10 == 0:
            print('Loss after extrinsics update: \n' + str(objFunc(x, remainingParam, dataPointListUpdated)))
        #scaledGrad = stepIntrinsics(x, remainingParam, eta, np.random.choice(dataPointListUpdated, size=3))
        #print('grad is: ', scaledGrad)
        #x -= scaledGrad
        
        # Derivative-free alternatives to consider are Nelder-Mead and Powell
        optResult = scipy.optimize.minimize(objFunc, x, args=(remainingParam, np.random.choice(dataPointListUpdated, size=batchSize)), method='BFGS', jac=gradOverall, options={'maxiter':1000})
        x = optResult.x
        
        print('x is: ', x)
        if iteration % 10 == 0:
            print('Loss after intrinsics update: \n' + str(objFunc(x, remainingParam, dataPointListUpdated)))
        iteration += 1
    return x, dataPointListUpdated

def testGradApprox(x, remainingParam, point, epsList):
    approxGradList = []
    for eps in epsList:
        # Do numerical gradient approximation for singleImageObjFunc
        grad = scipy.optimize.approx_fprime(x, singleImageObjFunc, eps, remainingParam, point)
        approxGradList.append(grad)
    # Each gradient is a row so compute std down each column
    return approxGradList, np.mean(np.array(approxGradList), axis=1), \
        np.std(np.array(approxGradList), axis=1), gradSingleImage(x, remainingParam, point)
