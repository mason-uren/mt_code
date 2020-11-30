import sys
sys.path.extend(['../../storage/DynamicExtrinsics_PythonSystem', '.', '..'])
#print(sys.path)

# Aaron added for personal import setup
#from setupImports import setup_path 
#setup_path()

#from hardware.ximea.Driver.client.ximea_client import * <<--- does not seem to be needed?
from Charuco_Specific.ChArUcoHelpers import *
from Charuco_Specific.CharucoBoards import *
import numpy as np
import math
import time
import shutil
import glob
import pdb
import os

##########################
# Charuco Board Consts   #
##########################

# Aaron Changed to TV_4 was TV_3, put back to TV_3 since used big monitor again
squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

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
dispBoard = board.draw((1000, 1000))
#cv2.imshow('board', dispBoard)
#cv2.waitKey(0)

calcorners = []
calids = []

##########################
# Load Images from Path  #
##########################
import glob

# Display heatmap & quiver plots while processing?
DISP_IMG = False

# Need to separate the imperx and ximea
cameraName = 'imperx' #'ximea'  or imperx 

# dirs to save results
intrinsics_save_dir = "./imperx_intrinsics/"
quiver_plot_save_dir = "./revisedQuiver/"
heatmap_save_dir = "./revisedHeatmap/"

import re
def get_img_number(filepath_tif):
    p = '[0-9]+.tif'

# Aaron modified the path
#filenames = glob.glob("calibration_images/*.jpg")
#filenames = glob.glob('C:/Users/aofeldman/Desktop/XimeaToolImgCapture/*.tif')
#filenames = glob.glob('C:/Users/aofeldman/Desktop/'+cameraName+'Data/*.tif')
filenames = glob.glob('/Users/ychen/Desktop/2020-09-16.ImperxInstrinsics/*.tiff')

filenames.sort()
print(filenames)

# Aaron changed so would respect ordering of images in the data set
#revFilenames = []
#for i in range(0, len(filenames)):
#    revFilenames.append('/Users/ychen/Desktop/FullDataset/'+cameraName+ str(i) + '.tif')
    
#filenames = revFilenames

# Previously used, but now made folders with better name convention
# rounds = ['Round1', 'Round2', 'Round3']
# prefix =  'C:/Users/aofeldman/Desktop/2020-06-19_XimeaIntrinsics/'

# image_list = []
# filenames = []
# for round in rounds:
#     folder = prefix + round
#     for filename in os.listdir(folder):
#         if cameraName in filename:
#             fullName = os.path.join(folder, filename)
#             filenames.append(fullName)
#             img = cv2.imread(fullName, 0)
#             if img is not None:
#                 image_list.append(img)

# # Separate into a folder for easier later access
# for i, img in enumerate(image_list):
#     cv2.imwrite('C:/Users/aofeldman/Desktop/' + cameraName + 'Data/'+cameraName + str(i) + '.tif', img)
    
# filenames = filenames.sort()

# Aaron added in reading in as grayscale
image_list = [cv2.imread(img, 0) for img in filenames]

print(image_list[0].shape[0])

# See what happens when remove the images which produced high reprojection error
#bad_images = [23, 24, 26, 27]
#for ind in bad_images:
#    image_list.pop(ind)

print("Number of Images in Folder: " + str(len(image_list)))

for i, image in enumerate(image_list):
    print("image = ", filenames[i])
    #print(i, " before blur")
    #print(image)
    #image = cv2.blur(image,(3,3))
    #print(i, "after blur")
    #print(image)
    # Aaron modified so now potentially use non-default parameters
    # corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
    
    # Aaron testing out different parameter options
    parameters = cv2.aruco.DetectorParameters_create()
    
    # Empirically observed that for the dataset slightly reducing this value
    # from the default results in significantly better detection for the 
    # imperx
    if cameraName == 'imperx':
        parameters.minMarkerPerimeterRate = 0.025
    
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
    
    if ids is not None:
        ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, image, board)

        calcorners.append(chcorners)
        calids.append(chids)
        ids = np.reshape(ids, (ids.shape[0],))
        print(len(chcorners))

    disp = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    cv2.aruco.drawDetectedCornersCharuco(disp, chcorners, chids, (0, 0, 255))
    #disp = cv2.resize(disp,(1000, 1000))
    #cv2.imshow("window", disp)
    #cv2.waitKey(1)
    cv2.imwrite('C:/Users/aofeldman/Desktop/testingParameters/'+str(i) + '.jpg', disp)
#cv2.destroyAllWindows()

#pdb.set_trace()

# Aaron added testing of other distortion models

# Set to True to test the other models and False otherwise
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

# Proceed with default model
rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    calcorners, calids, board, (image_list[0].shape[1],image_list[0].shape[0]), None, None)

# Aaron temporarily testing thin prism model
#rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
#    calcorners, calids, board, (image_list[0].shape[1],image_list[0].shape[0]), None, None, None, None, cv2.CALIB_THIN_PRISM_MODEL)


print("Inital Seed Reprojection Error: " + str(rms))

print(cameraMatrix)
print(distCoeffs)

if not os.path.exists(intrinsics_save_dir):
    os.mkdir(intrinsics_save_dir)
np.save(intrinsics_save_dir + cameraName + '_distCoeffs.npy', distCoeffs)
np.save(intrinsics_save_dir + cameraName + '_cameraMatrix.npy', cameraMatrix)
np.savetxt(intrinsics_save_dir + cameraName + '_distCoeffs.txt', distCoeffs)
np.savetxt(intrinsics_save_dir + cameraName + '_cameraMatrix.txt', cameraMatrix)


# Aaron added the following two blocks of code
mean = 0
for i, image in enumerate(image_list):
    error = charuco_reprojection_error(board, calcorners[i], calids[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs)
    #error = charuco_RMS_reprojection_error(board,calcorners[i],calids[i],rvecs[i],tvecs[i],cameraMatrix,distCoeffs)
    #print('i', i, 'error', error)
    mean += error / len(image_list)
print('meanError', mean)
# print(distCoeffs)

if not os.path.exists(quiver_plot_save_dir):
    os.mkdir(quiver_plot_save_dir)
if not os.path.exists(heatmap_save_dir):
    os.mkdir(heatmap_save_dir)

for i, image in enumerate(image_list):
    print('image: ' + str(i))
    # Generate quiver plots
    # Also returns corners but not needed
    extrinsics,reprojection_error, _ = estimate_Pose_Charucoboard_Ximea(image_list[i], board, cameraMatrix, distCoeffs)
    #print('i', i, 'error', reprojection_error)
        
    rvec_ximea,tvec_ximea = decompose_Extrinsics(extrinsics)
    
    # Aaron testing out different parameter options
    parameters = cv2.aruco.DetectorParameters_create()
    if cameraName == 'imperx':
        parameters.minMarkerPerimeterRate = 0.025
    
    quiver_image1 = generate_reprojection_error_quiver_plot_ids(image_list[i], aruco_dict, boards['TV_3'], rvec_ximea, tvec_ximea,
                                                           cameraMatrix, distCoeffs, parameters)
    
    width = int(quiver_image1.shape[1] / 4)
    height = int(quiver_image1.shape[0] / 4)
    dim = (width, height)
    newIm = cv2.resize(quiver_image1, dim)
    if DISP_IMG:
        cv2.imshow('quiver' + str(i), newIm)
    cv2.imwrite(quiver_plot_save_dir +'/quiver'+str(i) + '.jpg', quiver_image1)
    
    # Try using the new heatmap
    #heat_image = generate_reprojection_error_heat(image_list[i], aruco_dict, boards['TV_4'], rvec_ximea, tvec_ximea, cameraMatrix, distCoeffs)
    
    
    heat_image = generate_reprojection_error_heat(image_list[i], aruco_dict, boards['TV_3'], rvec_ximea, tvec_ximea, cameraMatrix, distCoeffs, parameters, 16)
    width = int(heat_image.shape[1] / 4)
    height = int(heat_image.shape[0] / 4)
    dim = (width, height)
    newIm = cv2.resize(heat_image, dim)
    if DISP_IMG:
        cv2.imshow('heatmap'+str(i), newIm)
        cv2.waitKey(1)
    cv2.imwrite(heatmap_save_dir+'/heatmap'+str(i) + '.jpg', newIm)

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