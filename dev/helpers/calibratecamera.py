import os
import glob
import cv2
import numpy as np
import time
from tqdm import tqdm

def load_images_from_folder(folder):
    images = []
    for filename in sorted(os.listdir(folder)):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    return images

if __name__ == "__main__":
    #define constants used for calibration
    grid_size = (6,8)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points
    objp = np.zeros((6 * 8, 3), np.float32)
    objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    #specify path to images
    images_path = os.path.dirname(__file__)
    images_path = os.path.join(images_path,"calibration_images")

    #store names of each file
    image_names = sorted(os.listdir(images_path))

    #apply our image transforms
    images = load_images_from_folder(images_path)
    images = [cv2.resize(x, (1500, 1000)) for x in images]
    # images = [cv2.cvtColor(x, cv2.COLOR_BGR2GRAY) for x in images]

    #create a solutions folder
    solution_path = os.path.join(images_path,"solution")
    if not os.path.exists(solution_path):
        print("making directory")
        os.makedirs(solution_path)
    else:
        #since the folder exists remove it from the image_names list
        image_names = image_names[:-1]

    #find checkerboard corners and store it in a new directory
    for counter , image in enumerate(tqdm(images)):
        ret, corners = cv2.findChessboardCorners(image, grid_size, None)

        if ret == True:
            # Display the corners
            cv2.drawChessboardCorners(image, (8, 6), corners, ret)

            #write to image to solution directory
            write_name = 'CornersFound_' + str(image_names[counter]) + ".jpeg"
            write_name = os.path.join(solution_path, write_name)
            cv2.imwrite(write_name, image)

            #find subcorners to increase accuracy
            objpoints.append(objp)

            #convert image to grayscale for subpix function to work
            image  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners2 = cv2.cornerSubPix(image, corners, (10, 10), (-1, -1), criteria)
            imgpoints.append(corners2)
        else:
            print("Pattern Not Found: " + str(image_names[counter]))

    time.sleep(.01)
    print("Finished processing image calibration from grid")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, images[0].shape[1::-1], None, None)

    #calculate reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    print("total error: {}".format(mean_error / len(objpoints)))







