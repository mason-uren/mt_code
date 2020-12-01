from hardware.imperx.Client.imperxClient import *
from Charuco_Specific.ChArUcoHelpers import *
from Charuco_Specific.CharucoBoards import *
import numpy as np
import math
import time
import shutil

##########################
# Charuco Board Consts   #
##########################
squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

##########################
# Graphng Setup          #
##########################
start_time = time.time()

cam = Imperx_recieve_camera()
cam.start()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

calcorners = []
calids = []

##########################
# Load Images from Path  #
##########################
import glob
filenames = glob.glob("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/Calibration/calibration_images/*")
image_list = [cv2.imread(img) for img in filenames]

print("Number of Images in Folder: " + str(len(image_list)))

for image in image_list:
    # image = cv2.resize(image,(5120, 5120))
    corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
    if ids is not None:
        ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, image, board)

        calcorners.append(chcorners)
        calids.append(chids)
        ids = np.reshape(ids, (ids.shape[0],))
        print(len(chcorners))

rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    calcorners, calids, board, (5120,5120), None, None)
print("Inital Seed Reprojection Error: " + str(rms))

# while True:
#     stuff = []
#     image = cam.get_latest_image()
#
#     image = cv2.resize(image,(5120, 5120))
#     corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
#
#     disp_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
#
#     if ids is not None:
#         ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
#             corners, ids, image, board)
#
#
#
#         cv2.aruco.drawDetectedCornersCharuco(disp_image, chcorners, chids, (0, 255, 0))
#
#         print(ids.shape)
#     else:
#         print("NO IDS DETECTED")
#
#     disp_image = cv2.resize(disp_image, (1000, 1000))
#     cv2.imshow("image", disp_image)
#
#     key = cv2.waitKey(1)
#     if key == ord('q'):
#         print("Calculating Intrinsics")
#
#         calids = np.asarray(calids)
#
#         print("Numer of Images Used: " + str(calids.shape[0]))
#
#         rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
#             calcorners, calids, board, (5120,5120), None, None)
#
#         print("RMS Error: " + str(rms))
#
#         print(cameraMatrix)
#
#         print(distCoeffs)
#
np.save('imperx_distCoeffs', distCoeffs)
np.save('imperx_intrinsics', cameraMatrix)
np.savetxt('imperx_distCoeffs', distCoeffs)
np.savetxt('imperx_intrinsics', cameraMatrix)
#
#         while(True):
#             image = cam.get_latest_image()
#             corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
#             if(ids is not None):
#                 ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
#                     corners, ids, image, board)
#                 ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
#                     corners, ids, image, board)
#                 retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chcorners, chids, board, cameraMatrix, distCoeffs,
#                                                                         useExtrinsicGuess=False)
#                 if(rvec is not None):
#                     points = cv2.projectPoints(np.float32([[0, 0, 0], [0, squareLength * charucoY, 0]]), rvec, tvec,
#                                                cameraMatrix, distCoeffs)
#                     points = points[0]
#                     image = cv2.resize(image,(1000,1000))
#
#                     for point in points:
#                         # point = np.asarray(point)
#                         point = np.asarray((point[0]))
#                         cv2.circle(image, (math.ceil(point[0]/ (5120 / 1000)),math.ceil(point[1]/ (5120 / 1000))), 10,
#                                    [255, 0, 0], 2, -1)  # BGR
#
#             image = cv2.resize(image, (1000, 1000))
#             cv2.imshow("image",image)
#             cv2.waitKey(1)
#
#
#
#     elif key == ord('w'):
#         print("Saving Image")
#
#         stuff = time.time()
#
#         stuff = "imperx_intrinsics_" + str(stuff) + ".jpg"
#
#         cv2.imwrite(stuff,image)
#         shutil.move(stuff,"calibration_images")
#
#         calcorners.append(chcorners)
#         calids.append(chids)
#
#         ids = np.reshape(ids, (ids.shape[0],))
