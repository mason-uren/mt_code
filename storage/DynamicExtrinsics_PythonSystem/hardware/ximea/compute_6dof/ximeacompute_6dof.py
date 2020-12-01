from misc.GenericHelpers import *
from Charuco_Specific.CharucoBoards import *
from Charuco_Specific.ChArUcoHelpers import *
from hardware.ximea.Driver.client.ximea_client import *
from Dynamic_Extrinsics.Main.Deepaks_Model.ccm import dynamic_extrinsics_correct_order
import json
import math as m
import numpy as np

cam = ximea_recieve_camera()

squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

intrinsics = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_intrinsics.npy")
dist = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_distCoeffs.npy")

print(intrinsics)

with open(
        '/home/sdrad/PycharmProjects/ClosedLoopMetrology/Dynamic_Extrinsics/Main/Deepaks_Model/cad_models/cad_model_4_z0_angle_imp_5.json',
        'r') as f:
    cad_model = json.load(f)
cadmdl = cad_model['cad_model']
print('cad_model= {}'.format(cadmdl))
print('Comments= {}'.format(cad_model['Comments']))
x = [cadmdl['yaw_ximea_tilt'],
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

pan = 89.99954986572266
tilt = 3.9995999336242676

print("Pan: " + str(pan))
print("Tilt: " +  str(tilt))

pan = m.radians(90.0 - pan)
tilt = m.radians(-tilt)

image = cam.get_latest_image()
image = cv2.blur(image,(3,3))

mat = estimate_Pose_Charucoboard(image, board, intrinsics, dist, aruco_dict,
                                 debug_verbose=True)

print("Ximea Extrinsics")
print(mat)
print("\n")

print("Dynamic Extrinsics:")
dynamic = dynamic_extrinsics_correct_order(x, tilt=-1 * m.radians(tilt),
                                                           pan=m.radians((90 - pan)))
print(dynamic)
print("\n")

print("Composed Extrinsics")
print(ximea_to_Imperx_frame(dynamic,mat))


image = cv2.resize(image,(1000,1000))
cv2.imshow("ximea_left",  image)

cv2.waitKey(0)