# Last Modified: 6/18/2020 Changes made by Aaron
import pdb
import matplotlib.pyplot as plt

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

##########################
# Charuco Board Consts   #
##########################

# Aaron Changed to TV_4 was TV_3
squareLength = boards['TV_4']['squareLength']
markerLength = boards['TV_4']['markerLength']
charucoX = boards['TV_4']['charucoX']
charucoY = boards['TV_4']['charucoY']

##########################
# Graphng Setup          #
##########################
start_time = time.time()

# Aaron commented out because don't actually care about running camera
#cam = ximea_recieve_camera()
#cam.start()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

# Aaron added to verify that board is indeed what are seeing in the images
#dispBoard = board.draw((1000, 1000))
#cv2.imshow('board', dispBoard)
#cv2.waitKey(0)

calcorners = []
calids = []

##########################
# Load Images from Path  #
##########################
import glob

# Aaron modified the path

#filenames = glob.glob("calibration_images/*.jpg")
filenames = glob.glob('C:/Users/aofeldman/Desktop/XimeaToolImgCapture/*.tif')

# filenames = filenames.sort()

# Aaron added in reading in as grayscale
image_list = [cv2.imread(img, 0) for img in filenames]

print(image_list[0].shape[0])

print("Number of Images in Folder: " + str(len(image_list)))

for image in image_list:
    image = cv2.blur(image,(3,3))
    corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
    if ids is not None:
        ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, image, board)

        calcorners.append(chcorners)
        calids.append(chids)
        ids = np.reshape(ids, (ids.shape[0],))
        print(len(chcorners))

    cv2.aruco.drawDetectedCornersCharuco(image, chcorners, chids, (0, 255, 0))
    image = cv2.resize(image,(1000, 1000))
    #cv2.imshow("window", image)
    #cv2.waitKey(1)

testDistortionModels = 0

if testDistortionModels: 
    flagList = [0, cv2.CALIB_ZERO_TANGENT_DIST, cv2.CALIB_RATIONAL_MODEL, cv2.CALIB_THIN_PRISM_MODEL, 
                cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL]
    rmsList = []
    coeffsList = []
    for i, flags in enumerate(flagList):    
        print('i', i)
        # Make sure to put in None for rvec and tvec
        rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            calcorners, calids, board, (image_list[0].shape[1],image_list[0].shape[0]), None, None, None, None, flags)
        rmsList.append(rms)
        coeffsList.append(distCoeffs)
    print('RMS values obtained: \n', rmsList)
    for i in range(len(coeffsList)):
        print('Coefficient for model: ' + str(i))
        print(coeffsList[i])
    
# Tentative Conclusion: Using the default distortion model is sufficient
    
# Now, try different termination criteria
        
#iterList = [100*i for i in range(1,11)]
#iterList = [100, 200, 300, 400, 500]
iterList = [100000]
rmsListIter = []
for i, max_iter in enumerate(iterList):
    criteria = (cv2.TERM_CRITERIA_MAX_ITER, max_iter, -1)
    rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        calcorners, calids, board, (image_list[0].shape[1],image_list[0].shape[0]), None, None, None, None, 0, criteria)
    rmsListIter.append(rms)

plt.figure()
plt.plot(iterList, rmsListIter)
plt.show()

pdb.set_trace()
    
    
print("Inital Seed Reprojection Error: " + str(rms))

print(cameraMatrix)

np.save('ximea_distCoeffs', distCoeffs)
np.save('ximea_intrinsics', cameraMatrix)
np.savetxt('ximea_distCoeffs', distCoeffs)
np.savetxt('ximea_intrinsics', cameraMatrix)


# Aaron added the following two blocks of code
mean = 0
for i, image in enumerate(image_list):
    error = charuco_reprojection_error(board, calcorners[i], calids[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs)
    #error = charuco_RMS_reprojection_error(board,calcorners[i],calids[i],rvecs[i],tvecs[i],cameraMatrix,distCoeffs)
    print('i', i, 'error', error)
    mean += error / len(image_list)
print('meanError', mean)
# print(distCoeffs)


for i, image in enumerate(image_list):
    # Generate quiver plots
    # Also returns corners but not needed
    extrinsics,reprojection_error, _ = estimate_Pose_Charucoboard_Ximea(image_list[i], board, cameraMatrix, distCoeffs)
    print('i', i, 'error', reprojection_error)
        
    rvec_ximea,tvec_ximea = decompose_Extrinsics(extrinsics)
    
    quiver_image1 = generate_reprojection_error_quiver_plot_ids(image_list[i], aruco_dict, boards['TV_4'], rvec_ximea, tvec_ximea,
                                                           cameraMatrix, distCoeffs)
    
    #quiver_image2 = generate_reprojection_error_quiver_plot_ids(image_list[i], aruco_dict, boards['TV_4'], rvec_ximea, tvec_ximea,
    #                                                       cameraMatrix, distCoeffs)
    # for i, image in enumerate([quiver_image1, quiver_image2]):
    #     width = int(image.shape[1] / 4)
    #     height = int(image.shape[0] / 4)
    #     dim = (width, height)
    #     newIm = cv2.resize(image, dim)
    #     cv2.imshow('quiver' + str(i), newIm)
    
    width = int(quiver_image1.shape[1] / 4)
    height = int(quiver_image1.shape[0] / 4)
    dim = (width, height)
    newIm = cv2.resize(quiver_image1, dim)
    cv2.imshow('quiver' + str(i), newIm)
    cv2.imwrite('C:/Users/aofeldman/Desktop/quivers/quiver'+str(i) + '.tif', quiver_image1)
    cv2.waitKey(1)
cv2.destroyAllWindows()
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
        # np.save('ximea_distCoeffs', distCoeffs)
        # np.save('ximea_intrinsics', cameraMatrix)
        # np.savetxt('ximea_distCoeffs', distCoeffs)
        # np.savetxt('ximea_intrinsics', cameraMatrix)
#     elif key == ord('w'):
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
