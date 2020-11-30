# same uv point in both images
# x,y,z in 3d space (both cameras)
#

#
# Add intepolate corners for both cameras and save to np array
#


from hardware.PTU.Python.Client.PTU_ZMQ import  *
from hardware.PTU.Python.PID_Controller.Image_PID import *
from hardware.Ximea.python.Driver.client.ximea_client import  *
from hardware.imperx.python.Driver.Client.imperxClient import *
from helpers.Charuco_Specific.CharucoBoards import *
from helpers.Charuco_Specific.ChArUcoHelpers import *
import cv2
import numpy as np
from itertools import product
from tqdm import tqdm
import time

#*******************#
#   Global Consts   #
#*******************#
DEBUG_VERBOSE = False

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
ximea_intrinsics = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/hardware/Ximea/intrinsics/ximea_intrinsics.npy")
ximea_dist = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/hardware/Ximea/intrinsics/ximea_distCoeffs.npy")
imperx_intrinsics = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/hardware/imperx/intrinsics/imperx_intrinsics.npy')
imperx_dist = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/hardware/imperx/intrinsics/imperx_distCoeffs.npy')

#**************************#
#      Board Settings      #
#**************************#
squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

#*******************#
#   Sample Settings #
#*******************#
Pan_Slots = 16
Tilt_Slots = 16

start_pan =  90    # start_pan must be less than end_pan
start_tilt = 3

end_pan =  (start_pan+12)              #12
end_tilt = (start_tilt+4.5)              #4.5

#*************************#
# Initialize Global Vars  #
#*************************#
pan_tilt_list = []
impx_ext = []
ximea_ext = []
combined_ext = []
reproj = []
ximea_chids = []
imperx_chids = []
UV_corners_ximea = []
UV_corners_imperx = []


ximea_extrinsics = None
imperx_extrinsics = None

#************************8
# Kennys code init
#***********************
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




for pan_angle,tilt_angle in tqdm(list(product(pan_pos,tilt_pos))):
    # for tilt_angle in tilt_pos:
    print("\n\n")
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
        time.sleep(1)
        ximea_img = ximea_cam.get_latest_image()
        ximea_extrinsics, reprojection_error, uv_corners_ximea, corners_ximea_chids = estimate_Pose_Charucoboard_Ximea(ximea_img, board, ximea_intrinsics, ximea_dist, aruco_dict, debug_verbose=DEBUG_VERBOSE, subsampling=False)

        imperx_img = Imperx_cam.get_latest_image()

        imperx_extrinsics, reprojection_error, uv_corners_imperx, corners_imperx_chids = estimate_Pose_Charucoboard_Imperx(imperx_img, board, imperx_intrinsics, imperx_dist, aruco_dict, debug_verbose=DEBUG_VERBOSE, subsampling=False)

        if((ximea_extrinsics is None) or (imperx_extrinsics is None)):
            print("Skipping Image:     No Board Found")
            continue
        else:
            rvec_ximea, tvec_ximea = decompose_Extrinsics(ximea_extrinsics)
            rvec_imperx, tvec_imperx = decompose_Extrinsics(imperx_extrinsics)

            # quiver_image = generate_reprojection_error_quiver_plot(ximea_img,aruco_dict,board, rvec_ximea, tvec_ximea, ximea_intrinsics, ximea_dist)
            # quiver_image_downsampled = cv2.resize(quiver_image,(1000,1000))
            # cv2.imshow("Ximea_Quiver",quiver_image_downsampled)
            # cv2.imwrite("Ximea_Quiver"+str(time.time())+".jpg",quiver_image)

            print("\n XIMEA TVEC")
            print(tvec_ximea)

            print("Ximea Mean Reprojection Error: " + str(reprojection_error))

            print("IMPERX TVEC")
            print(tvec_imperx)

            print("Ximea to Imperx")

            reproj.append(reprojection_error)

            pan, tilt = ptu_angles.get_angles()
            angles = np.hstack((pan, tilt))
            pan_tilt_list.append(angles)

            ximea_ext.append(ximea_extrinsics)

            impx_ext.append(imperx_extrinsics)

            composed_extrinsics = relative_Extrinsics_Ximea_to_Imperx(ximea_extrinsics, imperx_extrinsics)
            R, T = decompose_Extrinsics(composed_extrinsics)
            print(T)
            print("\n \n")

            combined_ext.append(composed_extrinsics)

            UV_corners_ximea.append(uv_corners_ximea)
            UV_corners_imperx.append(uv_corners_imperx)

            ximea_chids.append(corners_ximea_chids)
            imperx_chids.append(corners_imperx_chids)


pan_tilt_list = np.asarray(pan_tilt_list)
ximea_ext = np.asarray(ximea_ext)
impx_ext = np.asarray(impx_ext)
combined_ext = np.asarray(combined_ext)
UV_corners_ximea = np.asarray(UV_corners_ximea)
UV_corners_imperx = np.asarray(UV_corners_imperx)

# from  DynamicExtrinsics.python.relative_extrinsics_model import *
# from  DynamicExtrinsics.python.load_dataset import *
# from  DynamicExtrinsics.python.measure_model_error import *
#
# pan, tilt = transform_pan_tilt_from_npy_to_tuple(pan_tilt_list)
#
# dataset = {"ximea_ext":ximea_ext,
#            "impx_ext":impx_ext,
#            "n_records":impx_ext.shape[0],
#            "pan": pan,
#            "tilt": tilt}
#
# error_func = lambda model, ds=dataset: pose_mae_scalar(model, ds)

# Alternatively, don't bother doing anything fancy at all.
# start = time.time()
#
# print("Start Time")
# print(start)
# model = RelativeExtrinsicsModel.from_dataset(dataset, error_func=error_func)
# model.to_json('cad_model_mae.json')
# model.show()
# print("\n\n\n End Time")
# print(time.time() - start)

np.save("pan_tilt",pan_tilt_list)
np.save("ximea_ext",ximea_ext)
np.save("impx_ext",impx_ext)
np.save("combined_ext",combined_ext)
np.save("UV_corners_ximea",UV_corners_ximea)
np.save("UV_corners_imperx",UV_corners_imperx)
np.save("ximea_chids",ximea_chids)
np.save("imperx_chids",imperx_chids)

print("Finished")