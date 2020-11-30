import h5py
import cv2
from hardware.Ximea.python.Driver.client.ximea_client import  *
from hardware.imperx.python.Driver.Client.imperxClient import *
import math as m
import visdom
from helpers.Charuco_Specific.CharucoBoards import boards
from helpers.Charuco_Specific.ChArUcoHelpers import *
import json

from DynamicExtrinsics.python import ccm
from DynamicExtrinsics.python.clm import *

from helpers.GenericHelpers import *
from hardware.PTU.Python.Client.PTU_ZMQ import  *
from DynamicExtrinsics.python.relative_extrinsics_model import *

from hardware.PTU.Python.PID_Controller.Image_PID import  *
from transforms3d.euler import  mat2euler

class computeExtrinsicsImperx():
    def __init__(self, cam):
        self.cam = cam
        self.rvec = None
        self.tvec = None

    def start(self):
        self.cam.start()
        Thread(target=self.calc, args=()).start()
        return self

    def calc(self):
        while (True):
            image = self.cam.get_latest_image()

            extr,_,_ = estimate_Pose_Charucoboard_Imperx(image,board,imperx_intrinsics,imperx_dist,subsampling=True,debug_verbose=False)

            if(extr is not None):
                self.rvec, self.tvec = decompose_Extrinsics(extr)

                self.tvec = np.array(self.tvec)
                self.rvec = mat2euler(self.rvec)

    def get_Extrinsics(self):
        return self.rvec, self.tvec

class Ximea_Quiver_Plot():
    def __init__(self):
        self.image = None
        self.rvec = None
        self.tvec = None
        self.rms_error = None

        self.ptu_status = None

    def start(self):
        Thread(target=self.calc_image, args=()).start()

    def set_extrins(self,rvec,tvec):
        self.rvec = rvec
        self.tvec = tvec

    def calc_image(self):
        global ximea_image, intrinsics, dist, ptu_status

        while(True):
            if(self.rvec is not None):
                try:
                    self.rms_error = RMS_Error_from_charuco_image(ximea_image, aruco_dict, board, self.rvec, self.tvec, intrinsics,
                                                              dist)
                    if(self.rms_error < 10):
                        pass
                        #quiver_plot = generate_reprojection_error_quiver_plot_ids(ximea_image,aruco_dict,boards['Fiducial'],self.rvec,self.tvec,intrinsics,dist,exageration_factor = 8)
                        #self.image = cv2.resize(quiver_plot,(1000,1000))
                except:
                     pass

    def get_rms_error(self):
        return self.rms_error

    def get_image(self):
        return self.image


class CLM():
    def __init__(self, cam, ptu):
        ##########################
        # Kennys Alg             #
        ##########################
        self.imperx, self.ximea = dict(), dict()

        self.ids = None
        self.corners = None

        self.cad_model = RelativeExtrinsicsModel.from_json('/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/DynamicExtrinsics/python/cad_models/2019-07-11-cad-model.json')

        self.cam = cam
        self.ptu = ptu

        self.tvec_w1 = None
        self.rvec_wl = None

        self.ximea_extrinsics = None

    def start(self):
        self.ptu.start()
        Thread(target=self.calc, args=()).start()
        return self

    def calc(self):
        while (True):
            image = ximea_cam.get_latest_image()

            try:
                ximea_extrinsics = estimate_Pose_Charucoboard_Ximea(image,board,intrinsics,dist,subsampling=True,debug_verbose=DEBUG_VERBOSE)
            except:
                ximea_extrinsics = None

            ptu_pan, ptu_tilt = self.ptu.get_angles()

            tf = self.cad_model.get_relative_extrinsics(tilt = -m.radians(ptu_tilt), pan = m.radians((90 - ptu_pan)))

            if (ximea_extrinsics is not None):
                try:
                    combined_extrinsics = ximea_to_Imperx_frame(ximea_extrinsics[0],tf)
                    self.ximea_extrinsics = ximea_extrinsics[0]

                    self.rvec_wl, self.tvec_w1 = decompose_Extrinsics(combined_extrinsics)
                    self.rvec_wl = mat2euler(self.rvec_wl)
                except:
                    self.rvec_wl, self.tvec_w1 =  None,None

    def get_combined_extrinsics(self):
        return self.tvec_w1, self.rvec_wl

    def get_ximea_extrinsics(self):
        if(self.ximea_extrinsics is not None):
            return decompose_Extrinsics(self.ximea_extrinsics)
        else:
            return None,None

##########################
# Display Settings       #
##########################
DEBUG_VERBOSE = True
RESOLUTION = 1000  # Pixels
CENTER = int(RESOLUTION / 2)  # Pixels
PAUSED = False
TravelSpeed = 800

##########################
# Charuco Board Consts   #
##########################
squareLength = boards['Fiducial']['squareLength']
markerLength = boards['Fiducial']['markerLength']
charucoX = boards['Fiducial']['charucoX']
charucoY = boards['Fiducial']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

axis = np.float32(
    [[0, 0, 0], [0, squareLength * charucoY, 0], [squareLength * charucoX, squareLength * charucoY, 0],
     [squareLength * charucoX, 0, 0]])

##########################
# Camera Intrinsics      #
##########################
intrinsics = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/hardware/Ximea/intrinsics/ximea_intrinsics.npy")
dist = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/hardware/Ximea/intrinsics/ximea_distCoeffs.npy")
imperx_intrinsics = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/hardware/imperx/intrinsics/imperx_intrinsics.npy')
imperx_dist = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology_Synced_with_SVN/hardware/imperx/intrinsics/imperx_distCoeffs.npy')


##########################
# Graphng Setup          #
##########################
# vis = visdom.Visdom()
start_time = time.time()
running_time = 0

##########################
# System Setup           #
##########################
imperx_cam = Imperx_recieve_camera()
imperx_cam.start()

imperx_cam1 = Imperx_recieve_camera()
imperx_thread = computeExtrinsicsImperx(imperx_cam1)
imperx_thread.start()

ximea_cam = ximea_recieve_camera()
ximea_cam.start()

cam2 = ximea_recieve_camera()
pid = PTU_PID(cam2)
pid.start()

ximea_plot = Ximea_Quiver_Plot()
ximea_plot.start()

ptu_angles = PTU_Driver()
ptu_angles.start()
ptu_angles.set_control_mode(PTU.Velocity)

cam4 = ximea_recieve_camera()
ptu_reader = PTU_Angle_Reader()
clm_prog = CLM(cam4, ptu_reader)
clm_prog.start()


##########################
# Setup Common Variables #
##########################
pan_vel = 0
tilt_vel = 0

prev_stat = None
Z_xyz = 0
Y_xyz = 0
X_xyz = 0

yaw = 0
pitch = 0
roll = 0

imperx_corners = 0
event_queue = [0, 0, 0]

ptu_status = None

while True:
    ################################################################
    # Opencv Specific Setup
    ################################################################
    key = cv2.waitKey(1)
    ################################################################

    ################################################################
    # Grab Images from each camera for display purposes
    ################################################################
    imperx_image = imperx_cam.get_latest_image()
    imperx_image_downsampled = cv2.resize(imperx_image, (1000, 1000))
    imperx_image_downsampled = cv2.cvtColor(imperx_image_downsampled, cv2.COLOR_GRAY2RGB)

    ximea_image = ximea_cam.get_latest_image()
    image = cv2.resize(ximea_image, (RESOLUTION, RESOLUTION))
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    ################################################################


    ################################################################
    # Display PID Error
    ################################################################
    xyz_w1, rvec_wl = clm_prog.get_combined_extrinsics()

    if(rvec_wl is not None):
        Alpha, Betta, Gamma = None,None,None

        roll = rvec_wl[2] * (180 / math.pi)
        pitch = rvec_wl[1] * (180 / math.pi)
        yaw = rvec_wl[0] * (180 / math.pi)

        X_xyz = xyz_w1[0]
        Y_xyz = xyz_w1[1]
        Z_xyz = xyz_w1[2]


        if (xyz_w1 is not None):
            z = "Z:" + "{:>7}".format("%+.3f" % float(Z_xyz)*1000)
            x = "X:" + "{:>7}".format("%+.3f" % float(X_xyz)*1000)
            y = "Y:" + "{:>7}".format("%+.3f" % float(Y_xyz)*1000)

            cv2.putText(image, str(X_xyz * 1000), (RESOLUTION - 165, 30), cv2.FONT_HERSHEY_COMPLEX, 1,
                        (255, 255, 0),
                        lineType=cv2.LINE_AA)
            cv2.putText(image, str(Y_xyz * 1000), (RESOLUTION - 165, 60), cv2.FONT_HERSHEY_COMPLEX, 1,
                        (255, 255, 0),
                        lineType=cv2.LINE_AA)
            cv2.putText(image, str(Z_xyz * 1000), (RESOLUTION - 165, 90), cv2.FONT_HERSHEY_COMPLEX, 1,
                        (255, 255, 0),
                        lineType=cv2.LINE_AA)


            Alpha = "Yaw:  " + ("%+.3f" % float(yaw)).zfill(8)
            Betta = "Pitch: " + ("%+.3f" % float(pitch)).zfill(8)
            Gamma = "Roll:  " + ("%+.3f" % float(roll)).zfill(8)

            cv2.putText(image, Alpha, (RESOLUTION - 263, 120), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 255),
                        lineType=cv2.LINE_AA)
            cv2.putText(image, Betta, (RESOLUTION - 263, 150), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 255),
                        lineType=cv2.LINE_AA)
            cv2.putText(image, Gamma, (RESOLUTION - 263, 180), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 255),
                        lineType=cv2.LINE_AA)
    ################################################################



    ################################################################
    # Average PID stats for smoother display
    ################################################################
    # x, y = corner.get_SetPoint_Location()
    status = pid.get_SETPOINT_status()

    # apply smoothing to the PID flag,  the data at times can be noisy
    if (status == PTU_PID_SETPOINT_STATUS.Found):
        event_queue.pop(0)
        event_queue.append(6)
    elif (status == PTU_PID_SETPOINT_STATUS.NotFound):
        event_queue.pop(0)
        event_queue.append(1)
    elif (status == PTU_PID_SETPOINT_STATUS.NoIds):
        event_queue.pop(0)
        event_queue.append(2)
    elif (status == PTU_PID_SETPOINT_STATUS.Converged):
        event_queue.pop(0)
        event_queue.append(12)

    counter = 0
    for elem in event_queue:
        counter += elem

    counter = counter / 3
    counter = math.ceil(counter)
    if (counter >= 3 and counter < 6):
        status = PTU_PID_SETPOINT_STATUS.Found
    elif (counter == 1):
        status = PTU_PID_SETPOINT_STATUS.NotFound
    elif (counter == 2):
        status = PTU_PID_SETPOINT_STATUS.NoIds
    elif (counter >= 7):
        status = PTU_PID_SETPOINT_STATUS.Converged


    # Display a dot indictating if we see the setpoint for PID
    if (status is PTU_PID_SETPOINT_STATUS.NoIds):
        image = cv2.circle(image, (500, 500), 5, (0, 0, 255), -1)
    elif (status is PTU_PID_SETPOINT_STATUS.NotFound):
        image = cv2.circle(image, (500, 500), 5, (255, 0, 0), -1)
    elif (status is PTU_PID_SETPOINT_STATUS.Found):
        image = cv2.circle(image, (500, 500), 5, (0, 255, 0), -1)
    elif (status is PTU_PID_SETPOINT_STATUS.Converged):
        image = cv2.circle(image, ((500, 500)), 8, (255, 255, 255), -1)
    ################################################################


    ################################################################
    # Add the ability to pause the PID tracker
    ################################################################
    if (key == ord('p')):
        if (PAUSED == True):
            PAUSED = False
        else:
            PAUSED = True
    else:
        if (PAUSED == True):
            cv2.rectangle(image, (0, 0), (RESOLUTION, RESOLUTION), color=(0, 255, 255), thickness=10)
            if (key == ord('w')):
                tilt_vel = -TravelSpeed
            elif (key == ord('a')):
                pan_vel = TravelSpeed
            elif (key == ord('s')):
                tilt_vel = TravelSpeed
            elif (key == ord('d')):
                pan_vel = -TravelSpeed
            else:
                pan_vel = 0
                tilt_vel = 0
        else:
            if (status is PTU_PID_SETPOINT_STATUS.NoIds):
                pan_vel = 0
                tilt_vel = 0
            elif (status is PTU_PID_SETPOINT_STATUS.NotFound):
                pan_vel = 0
                tilt_vel = 0
            else:
                pan_vel, tilt_vel = pid.get_control()
    ################################################################



    ################################################################
    # Display PID Error
    ################################################################
    pan_error, tilt_error = pid.get_Error()

    pan_error_debug = "Pan Error: " + str(int(pan_error))
    pan_vel_debug = "Pan Vel: " + str(int(pan_vel))

    tilt_error_debug = "Tilt Error: " + str(int(tilt_error))
    tilt_vel_debug = "Tilt Vel: " + str(int(tilt_vel))

    cv2.putText(image, pan_error_debug, (0, 30), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), lineType=cv2.LINE_AA)
    cv2.putText(image, tilt_error_debug, (0, 60), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), lineType=cv2.LINE_AA)
    cv2.putText(image, pan_vel_debug, (0, 90), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 0), lineType=cv2.LINE_AA)
    cv2.putText(image, tilt_vel_debug, (0, 120), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 0), lineType=cv2.LINE_AA)

    ptu_pan, ptu_tilt = ptu_angles.get_angles()
    ptu_pan_string = "Pan Angle: " + "%.2f" % ptu_pan
    ptu_tilt_string = "Tilt Angle: " + "%.2f" % ptu_tilt

    ################################################################



    ptu_angles.set_velocity(pan_vel, tilt_vel)

    cv2.putText(image, str(ptu_pan_string), (0, 150), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255),
                lineType=cv2.LINE_AA)
    cv2.putText(image, str(ptu_tilt_string), (0, 180), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255),
                lineType=cv2.LINE_AA)

    image = adjust_gamma(image, gamma=2)
    display_image = np.concatenate((image, imperx_image_downsampled), axis=1)

    imperx_rvec, imperx_tvec = imperx_thread.get_Extrinsics()

    if(imperx_tvec is not None):
        cv2.putText(display_image, str(imperx_tvec[0] * 1000), ((RESOLUTION*2) - 165, 30), cv2.FONT_HERSHEY_COMPLEX, 1,
                    (255, 255, 0),
                    lineType=cv2.LINE_AA)
        cv2.putText(display_image, str(imperx_tvec[1] * 1000), ((RESOLUTION*2) - 165, 60), cv2.FONT_HERSHEY_COMPLEX, 1,
                    (255, 255, 0),
                    lineType=cv2.LINE_AA)
        cv2.putText(display_image, str(imperx_tvec[2] * 1000), ((RESOLUTION*2) - 165, 90), cv2.FONT_HERSHEY_COMPLEX, 1,
                    (255, 255, 0),
                    lineType=cv2.LINE_AA)

        roll = imperx_rvec[2] * (180 / math.pi)
        pitch = imperx_rvec[1] * (180 / math.pi)
        yaw = imperx_rvec[0] * (180 / math.pi)

        Alpha = "Yaw:  " + ("%+.3f" % float(yaw)).zfill(8)
        Betta = "Pitch: " + ("%+.3f" % float(pitch)).zfill(8)
        Gamma = "Roll:  " + ("%+.3f" % float(roll)).zfill(8)


        cv2.putText(display_image, Alpha, ((RESOLUTION*2) - 263, 120), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 255),
                    lineType=cv2.LINE_AA)
        cv2.putText(display_image, Betta, ((RESOLUTION*2) - 263, 150), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 255),
                    lineType=cv2.LINE_AA)
        cv2.putText(display_image, Gamma, ((RESOLUTION*2) - 263, 180), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 255),
                    lineType=cv2.LINE_AA)
    ################################################################

    ximea_rvec, ximea_tvec = clm_prog.get_ximea_extrinsics()
    ximea_plot.set_extrins(ximea_rvec,ximea_tvec)
    ximea_quiver_plot_image = ximea_plot.get_image()
    if(ximea_quiver_plot_image is not None):
        cv2.imshow("quiver_plot", ximea_quiver_plot_image)

    cv2.imshow("img", display_image)

    if (key == ord('q')):
        stuff = time.time()
        name = 'Screenshot_' + str(stuff) + '.jpg'
        cv2.imwrite(name, display_image)