from hardware.imperx.Client.imperxClient import *
from Charuco_Specific.CharucoBoards import *
from Charuco_Specific.ChArUcoHelpers import *
import cv2
import numpy as np
from misc.GenericHelpers import *

imperx_intrinsics = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_intrinsics.npy")
imperx_dist = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_distCoeffs.npy")
print(imperx_intrinsics)

# ximea_intrinsics = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_intrinsics.npy")
# ximea_dist = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_distCoeffs.npy")

squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

##########################
# System Setup           #
##########################
cam = Imperx_recieve_camera()
cam.start()

while(True):
    img = cam.get_latest_image()
    image , pixel_coordinates_in_orignal_image = search_by_crop_for_charucoBoard(img,aruco_dict,[15,15])
    ext = estimate_Pose_Charucoboard_Imperx(img, board, imperx_intrinsics, imperx_dist, subsampling=True, debug_verbose=True)
    print(ext)
    cv2.imshow("iamge",image)
    cv2.waitKey(1)