from Dynamic_Extrinsics.Main.Deepaks_Model import ccm
from hardware.PTU.Client.PTU_ZMQ import  *
from hardware.ximea.Driver.client.ximea_client import  *
from hardware.imperx.Client.imperxClient import *
from Charuco_Specific.ChArUcoHelpers import *
from Charuco_Specific.CharucoBoards import *
import cv2
import numpy as np
import math as m
from scipy.optimize import least_squares

#*******************#
#   Global Consts   #
#*******************#
MINIMUM_NUMBER_OF_POINTS = 10

#Not implemeneted yet
MAXIMUM_REPROJECTION_ERROR = 4

#*******************#
#    Setup Hardware #
#*******************#
ptu_angles = PTU_Driver()
ptu_angles.start()
ptu_angles.set_control_mode(PTU.Position)

ximea_cam = ximea_recieve_camera()
ximea_cam.start()

Imperx_cam = Imperx_recieve_camera()
Imperx_cam.start()

#*******************#
#  Load Intrinsics  #
#*******************#
ximea_intrinsics = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_intrinsics.npy")
ximea_dist = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_distCoeffs.npy")
imperx_intrinsics = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_intrinsics.npy')
imperx_dist = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_distCoeffs.npy')

#**************************#
#      Board Settings      #
#**************************#
squareLength = boards['Fiducial']['squareLength']
markerLength = boards['Fiducial']['markerLength']
charucoX = boards['Fiducial']['charucoX']
charucoY = boards['Fiducial']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

#*******************#
#   Sample Settings #
#*******************#
Pan_Slots = 5
Tilt_Slots = 5

start_pan =  85.58    # start_pan must be less than end_pan
start_tilt = 4.54

end_pan =  90.57
end_tilt = 7.4

#*************************#
# Initialize Global Vars  #
#*************************#
pan_tilt_list = []
impx_ext = []
ximea_ext = []
ximea_in_imperx = []
combined_ext = []
UV_Imperx = []
UV_Ximea = []

#************************8
# Deepaks code init
#***********************
import json

with open('/home/sdrad/PycharmProjects/ClosedLoopMetrology/Dynamic_Extrinsics/Main/Deepaks_Model/cad_model.json', 'r') as f:
    cad_model = json.load(f)
cadmdl = cad_model['cad_model']
print('15-parameter model: {}'.format(cad_model['comments']))
print('15 parameters = {}'.format(cadmdl))
xn = [cadmdl['yaw_ximea_tilt'],
          cadmdl['pitch_ximea_tilt'],
          cadmdl['roll_ximea_tilt'],
          cadmdl['x_ximea_tilt'],
          cadmdl['y_ximea_tilt'],
          cadmdl['z_ximea_tilt'],
          cadmdl['x_tilt_pan'],
          cadmdl['y_tilt_pan'],
          cadmdl['z_tilt_pan'],
          cadmdl['yaw_base_imperx'],
          cadmdl['pitch_base_imperx'],
          cadmdl['roll_base_imperx'],
          cadmdl['x_base_imperx'],
          cadmdl['y_base_imperx'],
          cadmdl['z_base_imperx'],
          ]

print("*****************************")
print("* Moving to First Position  *")
print("*****************************\n\n")

ptu_angles.set_angles(start_pan, start_tilt)
while(ptu_angles.get_status() is PTU_SETPOINT_STATUS.Moving):
    pan , tilt = ptu_angles.get_angles()
    continue

print("*****************************")
print("*   Data Collection Start   *")
print("*****************************\n\n")


# # while (abs(set_pan) < abs(end_pan) or set_tilt < end_tilt or set_tilt > end_tilt):
pan, tilt = ptu_angles.get_angles()

pan_pos = [(start_pan + (i * (end_pan - start_pan)/Pan_Slots)) for i in range(Pan_Slots)]
tilt_pos = [(start_tilt + (i * (end_tilt - start_tilt)/(Tilt_Slots/1))) for i in range(Tilt_Slots)]

for pan_angle in pan_pos:
    for tilt_angle in tilt_pos:
        print("PAN: " + str(pan_angle) + " TILT: " +  str(tilt_angle))
        ptu_angles.set_angles(pan_angle, tilt_angle)

        imperx_img = Imperx_cam.get_latest_image()
        ximea_img = ximea_cam.get_latest_image()


        while (ptu_angles.get_status() is PTU_SETPOINT_STATUS.Moving):
                    imperx_img = Imperx_cam.get_latest_image()


                    imperx_img_downsampled = cv2.resize(imperx_img, (500, 500))

                    ximea_img = ximea_cam.get_latest_image()


                    ximea_img_downsampled = cv2.resize(ximea_img, (500, 500))

                    imgs_side_to_side = np.concatenate((imperx_img_downsampled, ximea_img_downsampled), axis=1)

                    cv2.imshow("img", imgs_side_to_side)
                    cv2.waitKey(1)
                    continue


        if(ptu_angles.get_status() is not PTU_SETPOINT_STATUS.Moving):
            ximea_img = ximea_cam.get_latest_image()
            imperx_img = Imperx_cam.get_latest_image()

            ximea_markers = None
            imperx_markers = None

            try:
                ximea_markers, imperx_markers = get_common_charuco_marker_ids(ximea_img, imperx_img, board, aruco_dict)
            except:
                continue

            if(ximea_markers is None):
                print("Skipping Image: Not enough points of light")
                continue

            # Extract Ximea Marker
            ximea_marker_point = ximea_markers[list(ximea_markers.keys())[0]]
            ximea_marker_point = np.asarray(ximea_marker_point)
            ximea_marker_point = ximea_marker_point.reshape(1, 4, 2)

            # Extract Imperx Marker
            imperx_marker_point = imperx_markers[list(imperx_markers.keys())[0]]
            imperx_marker_point = np.asarray(imperx_marker_point)
            imperx_marker_point = imperx_marker_point.reshape(1, 4, 2)

            # Get 6 dof to both markers
            rvec_ximea, tvec_ximea, _ = cv2.aruco.estimatePoseSingleMarkers(ximea_marker_point, markerLength,
                                                                            ximea_intrinsics, ximea_dist)
            rvec_imperx, tvec_imperx, _ = cv2.aruco.estimatePoseSingleMarkers(imperx_marker_point, markerLength,
                                                                              imperx_intrinsics, imperx_dist)

            # Extract UV point for both markers
            point_ximea = cv2.projectPoints(np.array([[0.0, 0.0, 0.0]]), rvec_ximea, tvec_ximea, ximea_intrinsics, ximea_dist)
            point_imperx = cv2.projectPoints(np.array([[0.0, 0.0, 0.0]]), rvec_imperx, tvec_imperx, imperx_intrinsics,
                                             imperx_dist)

            u_ximea = math.ceil(point_ximea[0][0][0][0])
            v_ximea = math.ceil(point_ximea[0][0][0][1])


            u_imperx = math.ceil(point_imperx[0][0][0][0])
            v_imperx = math.ceil(point_imperx[0][0][0][1])

            UV_Ximea.append((u_ximea,v_ximea))
            UV_Imperx.append((u_imperx,v_imperx))


            tvec_ximea = tvec_ximea[0][0]
            tvec_imperx = tvec_imperx[0][0]

            tvec_ximea = np.reshape(tvec_ximea,(3,1))
            tvec_imperx = np.reshape(tvec_imperx,(3,1))

            rvec_ximea, _ = cv2.Rodrigues(rvec_ximea)
            rvec_imperx , _ = cv2.Rodrigues(rvec_imperx)

            # Create camera matrices
            ximea_extrinsics = np.hstack((rvec_ximea, tvec_ximea))
            ximea_extrinsics = np.vstack((ximea_extrinsics, [[0, 0, 0, 1]]))
            ximea_ext.append(ximea_extrinsics)

            imperx_extrinsics = np.hstack((rvec_imperx,tvec_imperx))
            imperx_extrinsics = np.vstack((imperx_extrinsics,[[0,0,0,1]]))
            impx_ext.append(imperx_extrinsics)

            ximea_extrinsics_inverted = np.linalg.inv(ximea_extrinsics)
            composed_extrinsics = np.matmul(imperx_extrinsics, ximea_extrinsics_inverted)

            # Run Deepaks Model
            # Get Pan Tilt Angles
            ptu_pan, ptu_tilt = ptu_angles.get_angles()
            angles = np.hstack((ptu_pan, ptu_tilt))
            pan_tilt_list.append(angles)

            tf = ccm.dynamic_extrinsics_dk15params(xn, tilt=-1 * m.radians(ptu_tilt),
                                                   pan=m.radians((90 - ptu_pan)))
            ximea_IN_imperx = np.matmul(tf, ximea_extrinsics)
            ximea_in_imperx_tvec = ximea_IN_imperx[0:3, 3]

            composed_extrinsics_tvec = composed_extrinsics[0:3, 3]

            ximea_in_imperx.append(ximea_IN_imperx)

            print("XIMEA TVEC")
            print(tvec_ximea)

            print("IMPERX TVEC")
            print(tvec_imperx)

            print("Combined Extrinsics")
            print(composed_extrinsics_tvec)

            print("Ximea in Imperx Space")
            print(ximea_in_imperx_tvec)

            print("\n\n")

pan_tilt_list = np.asarray(pan_tilt_list)
ximea_ext = np.asarray(ximea_ext)
impx_ext = np.asarray(impx_ext)
composed_extrinsics = np.asarray(composed_extrinsics)
ximea_in_imperx = np.asarray(ximea_in_imperx)

np.save("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Validation_Dataset/pan_tilt",pan_tilt_list)
np.save("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Validation_Dataset/ximea_ext",ximea_ext)
np.save("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Validation_Dataset/impx_ext",impx_ext)
np.save("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Validation_Dataset/combined_ext",composed_extrinsics)
np.save("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Validation_Dataset/ximea_in_imperx",ximea_in_imperx)
np.save("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Validation_Dataset/UV_Imperx",UV_Imperx)
np.save("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Validation_Dataset/UV_Ximea",UV_Ximea)

print("Finished")
exit()