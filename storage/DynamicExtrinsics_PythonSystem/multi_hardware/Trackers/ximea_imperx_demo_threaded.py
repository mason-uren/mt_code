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
from transforms3d.euler import  mat2euler

##############################################################################################################################
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

            extr = estimate_Pose_Charucoboard_Imperx(image,board,imperx_intrinsics,imperx_dist,subsampling=True,debug_verbose=False)
            if(extr is not None):
                self.rvec, self.tvec = decompose_Extrinsics(extr)

                self.tvec = np.array(self.tvec)
                self.rvec = mat2euler(self.rvec)

    def get_Extrinsics(self):
        return self.rvec, self.tvec

##############################################################################################################################
class CLM():
    def __init__(self, cam, ptu):
        ##########################
        # Kennys Alg             #
        ##########################
        self.imperx, self.ximea = dict(), dict()

        self.ids = None
        self.corners = None

        with open('/home/sdrad/PycharmProjects/ClosedLoopMetrology/Dynamic_Extrinsics/Main/Deepaks_Model/cad_models/cad_model_4_z0_angle_imp_5.json', 'r') as f:
            cad_model = json.load(f)
        cadmdl = cad_model['cad_model']
        print('15-parameter model: {}'.format(cad_model['Comments']))
        print('15 parameters = {}'.format(cadmdl))
        self.x = [cadmdl['yaw_ximea_tilt'],
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

        self.cam = cam
        self.ptu = ptu

        self.tvec_w1 = None
        self.rvec_wl = None

    def start(self):
        self.ptu.start()
        Thread(target=self.calc, args=()).start()
        return self

    def calc(self):
        while (True):
            image = ximea_cam.get_latest_image()

            ximea_extrinsics = estimate_Pose_Charucoboard_Ximea(image, board, intrinsics, dist, subsampling=True, debug_verbose=DEBUG_VERBOSE)


            ptu_pan, ptu_tilt = self.ptu.get_angles()

            tf = ccm.dynamic_extrinsics_correct_order(self.x, tilt=-1 * m.radians(ptu_tilt), pan=m.radians((90 - ptu_pan)))
            # print(tf)

            if (ximea_extrinsics is not None):
                try:
                    ximea_extrinsics = np.array(ximea_extrinsics)
                    combined_extrinsics = ximea_to_Imperx_frame(ximea_extrinsics,tf)
                    self.rvec_wl, self.tvec_w1 = decompose_Extrinsics(combined_extrinsics)
                    self.rvec_wl = mat2euler(self.rvec_wl)
                except:
                    return None,None

    def get_combined_extrinsics(self):
        return self.tvec_w1, self.rvec_wl

##############################################################################################################################
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
intrinsics = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_intrinsics.npy')
dist = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_distCoeffs.npy')
print(intrinsics)

imperx_intrinsics = np.load(
    '/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_intrinsics.npy')
imperx_dist = np.load(
    '/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_distCoeffs.npy')

##########################
# Graphng Setup          #
##########################
vis = visdom.Visdom()
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

    image = ximea_cam.get_latest_image()
    image = cv2.resize(image, (RESOLUTION, RESOLUTION))
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
            elif (key == ord('f')):
                print("Hit FFFFFf")
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


    display_image = np.concatenate((image, imperx_image_downsampled), axis=1)
    display_image = adjust_gamma(display_image, gamma=1.8)

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


    cv2.imshow("img", display_image)

    if (key == ord('q')):
        stuff = time.time()
        name = 'Screenshot_' + str(stuff) + '.jpg'
        cv2.imwrite(name, display_image)