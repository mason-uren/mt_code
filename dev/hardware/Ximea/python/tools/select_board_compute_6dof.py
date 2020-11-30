from hardware.ximea.Driver.client.ximea_client import *
from Charuco_Specific.CharucoBoards import *
import cv2
import numpy as np
from misc.GenericHelpers import *

#**************************#
#      Camera Constancs    #
#**************************#
CAMERA_X_RESOLUTION = 6000
CAMERA_Y_RESOLUTION = 7900

ximea_intrinsics = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_intrinsics.npy")
ximea_dist = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_distCoeffs.npy")

print("Intrinsics")
print(ximea_intrinsics)
print("Distortion")
print(ximea_dist)

squareLength = boards['Fiducial']['squareLength']
markerLength = boards['Fiducial']['markerLength']
charucoX = boards['Fiducial']['charucoX']
charucoY = boards['Fiducial']['charucoY']

cam = ximea_recieve_camera()
cam.start()
img = cam.get_latest_image()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

# Callback Function for Selecting Imperx FOV from large image
selecting = False
left_rect_point = (0,0)
right_rect_point = (0,0)
imperx_corners = None
imperx_ids = None

def select_imperx(event, x, y, flags, param):
    global left_rect_point, right_rect_point, selecting, imperx_corners,imperx_ids, img
    if event == cv2.EVENT_LBUTTONDOWN:
        left_rect_point = (x, y)
        selecting = True

    elif event == cv2.EVENT_MOUSEMOVE and selecting == True:
        right_rect_point = (x, y)


    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # crop imperx image
        left_y = int((left_rect_point[0]) * (CAMERA_Y_RESOLUTION / 1000))
        right_y = int((right_rect_point[0]) * (CAMERA_Y_RESOLUTION / 1000))
        left_x = int((left_rect_point[1]) * (CAMERA_X_RESOLUTION / 1000))
        right_x = int((right_rect_point[1]) * (CAMERA_X_RESOLUTION / 1000))

        img_cropped = img[left_x:right_x, left_y:right_y]

        # compute extrinsics for imperx
        if(img_cropped.shape[0] > 0 and img_cropped.shape[1] > 0):
            imperx_corners, imperx_ids, rejectedImgPts = cv2.aruco.detectMarkers(img_cropped, aruco_dict)

            if(imperx_ids is not None and len(imperx_ids) > 5):
                ret, imperx_corners, imperx_ids = cv2.aruco.interpolateCornersCharuco(
                    imperx_corners, imperx_ids, img_cropped, board)

                # Offset Detected Image to correspond with upsampled imperx
                offset_imper_cornerx = np.array([[left_y, left_x]])
                offset_imper_cornerx = np.tile(offset_imper_cornerx,(imperx_corners.shape[0],1,1))

                imperx_corners = np.add(imperx_corners,offset_imper_cornerx)

                for imgpt in imperx_corners:
                    cv2.circle(img, (int(imgpt[0][0]), int(imgpt[0][1])), 15, (0, 255, 0))

                retval, rvec1, tvec1 = cv2.aruco.estimatePoseCharucoBoard(imperx_corners, imperx_ids, board,
                                                                          ximea_intrinsics,
                                                                          ximea_dist, useExtrinsicGuess=False)

                if tvec1 is not None:
                    print("\n")
                    print("number of corners")
                    print(len(imperx_corners))

                    print("Tvec" + str(tvec1))
                    rvec1 = cv2.Rodrigues(rvec1)
                    print(rotationMatrixToEulerAngles(rvec1[0])*(180/math.pi))

                cv2.rectangle(img,   (left_y,left_x),(img_cropped.shape[1] + left_y,img_cropped.shape[0] + left_x), (0, 255, 0),thickness=40)
                stuff = cv2.resize(img,(1000,1000))
                # cv2.imshow("imperx",stuff)

                # cv2.imwrite("imperx_6_dof_right.jpg",stuff)

                selecting = False
                right_rect_point = (x, y)


##########################
# System Setup           #
##########################
cv2.namedWindow("img")
cv2.setMouseCallback("img",select_imperx)

while(True):
    img = cam.get_latest_image()
    cv2.imwrite("6dof_ximea.jpg", img)

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    img_downsampled = cv2.resize(img,(1000,1000))


    if (((right_rect_point[0] - left_rect_point[0]) > 0) and ((right_rect_point[1] - left_rect_point[1]) > 0)):
        distance = right_rect_point[0] - left_rect_point[0]
        # right_rect_point = (right_rect_point[0], left_rect_point[1])

        if (selecting == True):
            cv2.rectangle(img_downsampled, left_rect_point, right_rect_point, (255, 0, 0))
        else:
            if(imperx_corners is not None and len(imperx_corners) > 0):
                cv2.rectangle(img_downsampled, left_rect_point, right_rect_point, (0, 255, 0))
            else:
                cv2.rectangle(img_downsampled, left_rect_point, right_rect_point, (0, 0, 255))


    cv2.imshow("img",img_downsampled)
    key = cv2.waitKey(1)
    if key == ord('w'):
        cv2.imwrite("E.jpg",img)