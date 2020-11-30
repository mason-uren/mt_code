import cv2
import os
from tqdm import tqdm
import numpy as np
import math

def adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")

    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)

def decompose_Extrinsics(extrinsics_4x4):
    tvec = extrinsics_4x4[0:3, 3]
    rvec = extrinsics_4x4[0:3, 0:3]
    return rvec,tvec

def load_images_from_folder(folder):
    print("Processing Images: Loading to a dict")

    images = []
    for filename in tqdm(sorted(os.listdir(folder))):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            # img = cv2.resize(img,(2000,2000))
            images.append(img)

    print("\n\n")
    return images


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
