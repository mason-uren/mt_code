from hardware.imperx.Client.imperxClient import *
import math as m
import visdom
from Charuco_Specific.CharucoBoards import boards
from Charuco_Specific.ChArUcoHelpers import *
import json
from Dynamic_Extrinsics.Main.Deepaks_Model import ccm
from Dynamic_Extrinsics.Main.Kennys_Model import clm
from misc.GenericHelpers import *
import h5py
import cv2
from hardware.PTU.PID_Controller.Image_PID import *

intrinsics = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_intrinsics.npy')
dist = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_distCoeffs.npy')

imperx_intrinsics = np.load(
    '/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_intrinsics.npy')
imperx_dist = np.load(
    '/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_distCoeffs.npy')

squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

cam = ximea_recieve_camera()
image = cam.get_latest_image()

imperx_cam = Imperx_recieve_camera()
imperx_cam.start()
imperx_image = imperx_cam.get_latest_image()

imperx_image = cv2.cvtColor(imperx_image,cv2.COLOR_GRAY2BGR)
image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)

ximea_markers,imperx_markers = get_common_charuco_marker_ids(image,imperx_image,board,aruco_dict)

# print(ximea_markers)
# print(imperx_markers)

# Extract Ximea Marker
ximea_marker_point = ximea_markers[list(ximea_markers.keys())[0]]
ximea_marker_point = np.asarray(ximea_marker_point)
ximea_marker_point = ximea_marker_point.reshape(1,4,2)

# Extract Imperx Marker
imperx_marker_point = imperx_markers[list(imperx_markers.keys())[0]]
imperx_marker_point = np.asarray(imperx_marker_point)
imperx_marker_point = imperx_marker_point.reshape(1,4,2)

# Get 6 dof to both markers
rvec_ximea,tvec_ximea,_ = cv2.aruco.estimatePoseSingleMarkers(ximea_marker_point,markerLength,intrinsics,dist)
rvec_imperx, tvec_imperx,_ = cv2.aruco.estimatePoseSingleMarkers(imperx_marker_point,markerLength,imperx_intrinsics,imperx_dist)

# Extract UV point for both markers
point = cv2.projectPoints(np.array([[0.0,0.0,0.0]]),rvec_ximea,tvec_ximea,intrinsics,dist)
point_imperx = cv2.projectPoints(np.array([[0.0,0.0,0.0]]),rvec_imperx,tvec_imperx,imperx_intrinsics,imperx_dist)

u_ximea = math.ceil(point[0][0][0][0])
v_ximea = math.ceil(point[0][0][0][1])

u_imperx = math.ceil(point_imperx[0][0][0][0])
v_imperx = math.ceil(point_imperx[0][0][0][1])

print(u_ximea)
print(v_ximea)

cv2.circle(image,(u_ximea,v_ximea),100,(0,255,0),-1)
cv2.circle(imperx_image,(u_imperx,v_imperx),10,(0,255,0),-1)

image = cv2.resize(image,(1000,1000))
imperx_image = cv2.resize(imperx_image,(1000,1000))
cv2.imshow("ximea",image)
cv2.imshow("imperx",imperx_image)
cv2.waitKey(1000000)