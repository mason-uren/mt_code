from hardware.ximea.Driver.client.ximea_client import *
from Charuco_Specific.ChArUcoHelpers import *
from Charuco_Specific.CharucoBoards import *
import numpy as np
import math
import time
import shutil
import glob
import tqdm

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

cam = ximea_recieve_camera()
cam.start()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

calcorners = []
calids = []

##########################
# Load Images from Path  #
##########################
import glob
filenames = glob.glob("calibration_images/*")
# filenames = filenames.sort()
image_list = [cv2.imread(img) for img in filenames]
# print(image_list)

print("Number of Images in Folder: " + str(len(image_list)))

for image in image_list:
    image = cv2.blur(image,(3,3))
    corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
    if ids is not None:
        ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, image, board)

        calcorners.append(chcorners)
        calids.append(chids)
        # ids = np.reshape(ids, (ids.shape[0],))
        # print(len(chcorners))

    cv2.aruco.drawDetectedCornersCharuco(image, chcorners, chids, (0, 255, 0))
    image = cv2.resize(image,(1000, 1000))
    # cv2.imshow("window", image)
    # cv2.waitKey(1)

rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    calcorners, calids, board, (image_list[0].shape[1],image_list[0].shape[0]), None, None)


print("Inital Seed Reprojection Error: " + str(rms))

print(cameraMatrix)
print(distCoeffs)

np.save('ximea_distCoeffs', distCoeffs)
np.save('ximea_intrinsics', cameraMatrix)
np.savetxt('ximea_distCoeffs', distCoeffs)
np.savetxt('ximea_intrinsics', cameraMatrix)

extrinsics,reprojection_error = estimate_Pose_Charucoboard_Ximea(image_list[0], board, cameraMatrix, distCoeffs)
print(reprojection_error)

rvec_ximea,tvec_ximea = decompose_Extrinsics(extrinsics)

quiver_image = generate_reprojection_error_quiver_plot(image_list[0], aruco_dict, board, rvec_ximea, tvec_ximea,
                                                       cameraMatrix, distCoeffs)

cv2.imwrite("quiver_image.jpg",quiver_image)

# while True:
#     stuff = []
#     image = cam.get_latest_image()
#     image = cv2.blur(image,(3,3))
#
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
#             calcorners, calids, board, (image[1],image[0]), None, None)
#
#         print("RMS Error: " + str(rms))
#
#         print("RVEC")
#         print(rvecs[0])
#
#         print("Tvec")
#         print(tvecs[0])
#
#         print(cameraMatrix)
#
#         print(distCoeffs)
#
#         np.save('ximea_distCoeffs', distCoeffs)
#         np.save('ximea_intrinsics', cameraMatrix)
#         np.savetxt('ximea_distCoeffs', distCoeffs)
#         np.savetxt('ximea_intrinsics', cameraMatrix)
#     elif key == ord('q'):
#         print("Saving Image")
#
#         stuff = time.time()
#
#         stuff = "ximea_intrinsics_" + str(stuff) + ".jpg"
#
#         cv2.imwrite(stuff,image)
#         # shutil.move(stuff,"/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/Calibration/calibration_images")
#
#         calcorners.append(chcorners)
#         calids.append(chids)
#
#
#
#         while(True):
#             image = cam.get_latest_image()
#             corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
#             if(ids is not None):
#                 ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
#                     corners, ids, image, board)
#                 retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chcorners, chids, board, cameraMatrix, distCoeffs,
#                                                                         useExtrinsicGuess=False)
#
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
#         stuff = "ximea_intrinsics_" + str(stuff) + ".jpg"
#
#         cv2.imwrite(stuff,image)
#         shutil.move(stuff,"/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/Calibration/calibration_images")
#
#         calcorners.append(chcorners)
#         calids.append(chids)