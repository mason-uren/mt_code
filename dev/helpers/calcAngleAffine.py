import cv2
import numpy as np
from helpers import *
import math

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0][0] * R[0][0] +  R[1][0] * R[1][0])
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2][1] , R[2][2])
        y = math.atan2(-R[2][0], sy)
        z = math.atan2(R[1][0], R[0][0])
    else :
        x = math.atan2(-R[1][2], R[1][1])
        y = math.atan2(-R[2][0], sy)
        z = 0
    return np.rad2deg(y)

if __name__ == "__main__":
    # specify path to images
    images_path = os.path.dirname(__file__)
    images_path = os.path.join(images_path,"calibration_images")

    # load images
    images = load_images_from_folder(images_path)

    # produce a list of points for both images
    grid_size = (6,8)    #number of checkerboard squares (width,height)
    ret, corners_image_0 = cv2.findChessboardCorners(images[0], grid_size, None)
    ret, corners_image_1 = cv2.findChessboardCorners(images[1], grid_size, None)

    # from the list of points estimate a affine transform
    transform, inliners = cv2.estimateAffine2D(corners_image_0,corners_image_1)

    # convert affine transform to homography by padding [0,0,1]
    arr = np.array([[0,0,1]])
    homography = np.vstack((transform,arr))

    intrinsics = np.fromfile('intrinsics.dat')
    _ , rotation, translation, normals = cv2.decomposeHomographyMat(homography,intrinsics)

    print(rotationMatrixToEulerAngles(rotation[0]))