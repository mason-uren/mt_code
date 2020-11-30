# Develop an optimization algorithm to find the needed pan & tilt to
# aim the Ximea camera to a given point in 3D in world coordinates.
# 2020-09-21 byYang Chen (ychen@hrl.com)

from DynamicExtrinsics.python.extrinsics import PanTiltModel
import numpy as np
from scipy.optimize import minimize, root
from tqdm import tqdm

test_2020_10_01=True

if test_2020_10_01:
	model_path = "../../../src/Config/DynamicExtrinsics/Ximea_id0_Imperx/2020_10_01_ptu0-cad-model.json"
else:
	model_path = "../../../src/Config/DynamicExtrinsics/Ximea_id0_Imperx/2020-07-23-cad-model.json"
cad_model={}
print('Using Dynamic Extrinsics model from: ', model_path)
cad_model = PanTiltModel.from_json(model_path)
print('Pre-trained model parameters: ')
print(cad_model.params)

ximeaCameraMatrixFile='../../../src/Config/CameraIntrinsics/Ximea_id0/2020-07-15/ximea_cameraMatrix.npy'
ximeaCameraMatrix = np.load(ximeaCameraMatrixFile)
#ximeaCameraMatrix = np.eye(3)
#ximeaCameraMatrix[0,2]=7920/2.0
#ximeaCameraMatrix[1,2]=6004/2.0
print('Camera Matrix (intrinsics):\n', ximeaCameraMatrix)

# For Ximea camera:
imageWidth=7920
imageHeight=6004

# In our test cases, the ChArUco board is an 8 x 12 board with square
# size = 12 mm, laid on its side. The origin of the board is at 
# the lower-left corner,  therefore the center of the board (which we track)
# is at:
centerOffset=[0.072, 0.048, 0]

def pantilt_loss(model, p3d, pan, tilt, cameraMatrix=None):
	'''Calculate the loss with respect to aiming the PTU/camera towards
	a given 3D point in world coordinate frame for the given dynamic 
	extrinsics model & pan & tilt.
	Input:
	  model: a PanTiltModel (extrinsics.py) object representing the dynamic extrinsic model
	  p3d: 3D point in world coordinate, as numpy array of 3 x 1.
	  pan & tilt: in radians, to calculate the loss for these pan & tilt.
	'''

	# First calculate p3d transformed into Ximea coordinate frame using the dynamic extrinsics model:
	extrinsics = np.eye(4)  # start w/ eye
	#extrinsics[0:3,3] = p3d   # assign p3d to the tvec part
	extrinsics = np.expand_dims(extrinsics, 0) # first (dummy) dim is required by PanTiltModel functions
	dynamic_ext = model.apply(pans=np.array([pan]), tilts=np.array([tilt]), world_exts=extrinsics);
	dynamic_ext = np.squeeze(dynamic_ext) # get rid of the extra dimension in axis 0.

	# ext_ximea is p3d in ximea coordinate frame with the tvec part defining the 3D point
	# of p3d in ximea coordinate frame, Pc:
	Pc = dynamic_ext @ np.append(p3d, [1.0])

	# We want both X and Y components of Pc to be 0.0, therefore define the loss to be the normal of Pc[0:2]:
	ret_val = None
	if cameraMatrix is None:
		ret_val = np.linalg.norm(Pc[0:2])
	else:
		fx = cameraMatrix[0, 0]
		fy = cameraMatrix[1, 1]
		cx = cameraMatrix[0, 2]
		cy = cameraMatrix[1, 2]
		ex = fx*Pc[0]/Pc[2] - imageWidth/2.0 + cx
		ey = fy*Pc[1]/Pc[2] - imageHeight/2.0 + cy
		ret_val = np.linalg.norm(np.array([ex, ey]))

	return ret_val


def fit_pantilt(model, p3d, pan_init=None, tilt_init=None, loss_fn=pantilt_loss, cameraMatrix=None):
    """Fit pan & tilt that achieves minimal pantilt_loss
       pan & tilt: initial values given in radians
       p3d: coordinates of P in 3D in world coords given in meters as a list or 
            numpy array of 1x3 or 3x1
       pan_init, tilt_init: initial guess of the pan & tult sought, in degrees
    """

    pan = pan_init or 0.0
    tilt = tilt_init or 0.0
    pan = np.radians(pan)
    tilt = np.radians(tilt)

    if not isinstance(p3d, (list, np.ndarray)) or not len(p3d)==3:
    	raise('fit_pantilt(): p3d must be a list or ndarray of 3 elements')


    with tqdm() as pbar:
        
        def objective(x):
            # Replace the old parameters with the new
            pan = x[0]
            tilt = x[1]
            
            loss = loss_fn(model, p3d, pan, tilt, cameraMatrix=cameraMatrix)
            
            return loss
            
        def update_pbar(x):
            pbar.update()
            pbar.set_description("Loss: {:.20f} ".format(objective(x)))
        
        # Magic black box minimization algorithm courtesy of scipy.
        res = minimize(objective, [pan, tilt], callback=update_pbar, method='lm')

    pan, tilt = res.x

    return pan, tilt

def find_needed_pantilt_1(p3d, pan_init=None, tilt_init=None):
	'''Estimate the needed pan and tilt angles to place p3d (world coord.)
	   into the center of Ximea camera mounted on the PTU.
	   p3d: coordinates of P in 3D in world coords given in meters
	'''
	# cad_model is loaded from model_path at the top of the file
	# ximeaCameraMatrix is loaded at the top of the file as well.
	pan, tilt = fit_pantilt(cad_model, p3d, cameraMatrix=ximeaCameraMatrix,
							pan_init=pan_init, tilt_init=tilt_init) 

	return np.degrees(pan), np.degrees(tilt)

def get_charucoboard_center(ext4x4, center=centerOffset):
	'''Return the 3D coordinates of the center of the charuco board
	   given the board's pose as ext4x4. 'center' is the 3D coordinate
	   of the center of the board in the board's own coordinate frame.
	   center: expected to be a tuple of 3 numbers.
	'''
	return ( ext4x4 @ np.array(center + [1.0]))[0:3]

def find_needed_pantilt_2(p3d, pan_init=None, tilt_init=None):

	# need from global space: cad_model, ximeaCameraMatrix
	model = cad_model
	cameraMatrix = ximeaCameraMatrix  # or None to use the simplified camera model
	pix_scale = 8000.0 # needed when cameraMatrix is None

	def pc_eq_zero(pantilt):

		# First calculate p3d transformed into Ximea coordinate frame using the dynamic extrinsics model:
		extrinsics = np.eye(4)  # start w/ eye
		#extrinsics[0:3,3] = p3d   # assign p3d to the tvec part
		extrinsics = np.expand_dims(extrinsics, 0) # first (dummy) dim is required by PanTiltModel functions
		dynamic_ext = model.apply(pans=np.array([pantilt[0]]), tilts=np.array([pantilt[1]]), world_exts=extrinsics);
		dynamic_ext = np.squeeze(dynamic_ext) # get rid of the extra dimension in axis 0.

		# ext_ximea is p3d in ximea coordinate frame with the tvec part defining the 3D point
		# of p3d in ximea coordinate frame, Pc:
		Pc = dynamic_ext @ np.append(p3d, [1.0])

		# We want both X and Y components of Pc to be 0.0, therefore define the loss to be the normal of Pc[0:2]:
		ret_val = None
		if cameraMatrix is None:
			# the reason for 8000 multiplier is that we are off the scale otherwise:
			# The terms to be minimized should be on the scale of pixels, which
			# is (fx*Pc[0]/Pc[2] - imageWidth/2.0 + cx) and (fy*Pc[1]/Pc[2] - imageHeight/2.0 + cy).
			# In our simplified case, the last 2 items cancel either other, and fx/Pc[2] & fy/Pc[2]
			# are in the range of 8000 (~ 40000/5.0).
			return (Pc[0]*pix_scale, Pc[1]*pix_scale)
		else:
			fx = cameraMatrix[0, 0]
			fy = cameraMatrix[1, 1]
			cx = cameraMatrix[0, 2]
			cy = cameraMatrix[1, 2]
			ex = fx*Pc[0]/Pc[2] - imageWidth/2.0 + cx
			ey = fy*Pc[1]/Pc[2] - imageHeight/2.0 + cy
			print ("Err: " + str(ex) + ", " + str(ey))
			return (ex, ey)

	# Use scipy.optimize.root to solve for pan & tilt
	pan = pan_init or 0.0
	tilt = tilt_init or 0.0
	pantilt = [np.radians(pan), np.radians(tilt)]

	sol = root(pc_eq_zero, pantilt)  # Works too, but a bit slower: method='lm'

	if sol.success:
		res_pantilt = np.degrees(sol.x)
		print('Solution (deg): ', res_pantilt)
		print('  sol.fun: ', sol.fun, ' sol.nfev:', sol.nfev)
		#print('  sol.nit: ', sol.nit)
		return res_pantilt[0],res_pantilt[1]
	else:
		print('Solver failed: ', sol.message)


if __name__ == '__main__':

	# I got the output of a few tracking tests as p3d input, obtained as follows:
	# combined_extrinsics (ext4x4): is the board pose in world coordinates
	# It is also the origin of the board in the world coordinate frame.
	# The source of these tracking test logs can be found in 
	# smb://netapp1-svm3/issl/issl_archive/Archives/2019/Boeing-DR/ClosedLoopMetrology/2020/Weekly-Progress/Yang/FiducialTracking/
	
	find_needed_pantilt = find_needed_pantilt_2

	if not test_2020_10_01:

		# A test case from 2020-09-03-test-ptu-id3/liveDemo_output182.txt, frame 238
		# where we got the combined extrinsic (when the board is off center) for the charuco board as:
		ext4x4 = [[-0.0003545231900102634, -0.9868268356033267, -0.1617796984990141, 1.004966029393896],
		      [-0.9945503149483599, 0.01721463267724019, -0.1028266865064967, 0.8766027772856584],
		      [0.1042571117450767, 0.1608615956495268, -0.981455043135274, 4.453245575966124],
		      [0, 0, 0, 1]]
		Pw = get_charucoboard_center(np.array(ext4x4))
		print('Pw = ', Pw)
		pan, tilt = find_needed_pantilt(Pw)
		print('Pan, Tilt = ', pan, tilt)
		#result: Pan, Tilt =  11.842305953140677 5.459690258617352
		# simple estimate from current code: "Executing pan/tilt command for pan,tilt =12.1139, 12.1139" 
		# This is a bug in FiducialTracker, fixed in rev170335, should have been:
		ifov_estimate = [12.1139, 5.63887]
		print('Difference in pan/tilt estimates:', np.array([pan, tilt])-ifov_estimate, '\n')

	    # Test 2, liveDemo_output182.txt, frame 340, combined extrinsics is
		ext4x4 = [[-0.02103312466090345, -0.9868440016024124, -0.1603013542311388, 1.026714682584063],
                  [-0.9946050603284869, 0.03694409810603352, -0.09693248982719453, 0.8762999407947074],
 	              [0.101579435103599, 0.1573977449535728, -0.9822970875695214, 4.455521206032103],
 				  [0, 0, 0, 1]]
	    # at this frame, the board is not centered, and Ximea extrinsics tvec = [0.49809057, 0.0611699, 4.87760]
		Pw = get_charucoboard_center(np.array(ext4x4))
		print('Pw = ', Pw)
		pan, tilt = find_needed_pantilt(Pw)
		print('Pan, Tilt = ', pan, tilt)
		# result: Pan, Tilt =  11.5984332526248 5.4609236482232095
		# IFOV estimates to achieve centering of the board from: "Executing pan/tilt command for pan,tilt =11.8798, 11.8798" (bug in FiducialTracker)
		ifov_estimate = [11.8798, 5.5342]
		print('Difference in pan/tilt estimates:', np.array([pan, tilt])-ifov_estimate, '\n')

		# Test 3, liveDemo_output270, frame 119, the Ximea extrinsics tvec = [0.073436, 0.050578, 5.07049]
		# which means X & Y are very close to 0.0.  combined extrinsics tvec is:
		ext4x4 = [[0.009694276456354688, -0.9912029420110292, -0.13199526034168, 1.123980863806926],
 				  [-0.9781459479316961, 0.01801723052341368, -0.2071373552719785, 0.8958032363733314],
 				  [0.207693344979543, 0.1311186758358484, -0.9693664772923923, 4.626688208290798],
 				  [0, 0, 0, 1]]
		Pw = get_charucoboard_center(np.array(ext4x4))
		print('Pw = ', Pw)
		pan, tilt = find_needed_pantilt(Pw)
		print('Pan, Tilt = ', pan, tilt)
		# result Pan, Tilt =  10.443849831878257 5.236826821937532
		# "Current PT position: pan=10.6852, tilt=5.43825; not moving P/T"
		current_pt = [10.6852, 5.43825]
		print('Difference in pan/tilt estimates:', np.array([pan, tilt])-current_pt, '\n')

		# test 4, liveDemo_output270, frame 231, Ximea extrinssics tvec = [0.0871220, 0.0697770, 5.0129569]
		# board is centered, X & Y are ~ 0.0. Combined extrinsics  is:
		ext4x4 = [[-0.02102907159857606, -0.9902679981102555, -0.1375756885005812, 1.322583040931534],
 				  [-0.9976010748657925, 0.02986127371341157, -0.06245318053412552, 0.7985867158620827],
 				  [0.06595357135377375, 0.1359323223185703, -0.9885203741829315, 4.581180873441279],
 				  [0, 0, 0, 1]]
		Pw = get_charucoboard_center(np.array(ext4x4))
		print('Pw = ', Pw)
		pan, tilt = find_needed_pantilt(Pw)
		print('Pan, Tilt = ', pan, tilt)
		# result: Pan, Tilt =  8.143035092577337 4.242087687271931
		# "Current PT position: pan=8.52165, tilt=4.20975; not moving P/T"
		current_pt = [8.52165, 4.20975]
		print('Difference in pan/tilt estimates:', np.array([pan, tilt])-current_pt, '\n')
	
	else: #2020-10-01 test cases
		# liveDemo_output281.txt, frame 200, fiducial is centered; the combined extrinsics is:
		ext4x4 = [[-0.002093218365355407, -0.9996626578942127, -0.02588800588599971, 0.4337741388788915],
	              [-0.999961585140658, 0.002312789339993262, -0.008454540108821476, 0.2451575786858086],
	              [ 0.008511561540504545, 0.02586931420326857, -0.999629097166942, 4.525695884418442],
	 			  [ 0, 0, 0, 1]]
		Pw = get_charucoboard_center(np.array(ext4x4))
		print('Pw = ', Pw)
		pan, tilt = find_needed_pantilt(Pw)
		print('Pan, Tilt = ', pan, tilt)
		current_pt = [-75.838, 5.842]
		print('Difference in pan/tilt estimates:', np.array([pan, tilt])-current_pt, '\n')
		#Difference in pan/tilt estimates: [-0.50815682 -0.1494245 ]

		# liveDemo_output281.txt, frame 212, fiducial off center; the combined extrinsics is:
		# ext4x4 = [[-0.001249680444996192, -0.9996615955110575, -0.02598331693748318, 0.4350450221257331],
		#           [ -0.999993774872958, 0.001334990951570476, -0.003266192659959852, 0.2458051701095814],
		#           [ 0.00329977485870539, 0.0259790734909375, -0.9996570408027122, 4.527438100904586],
		#           [ 0, 0, 0, 1]]
		# Pw = get_charucoboard_center(np.array(ext4x4))
		# print('Pw = ', Pw)
		# pan, tilt = find_needed_pantilt(Pw)
		# print('Pan, Tilt = ', pan, tilt)
		# #Executing pan/tilt command for pan,tilt =-76.1155, 6.01706
		# ifov_estimate = [-76.1155, 6.01706]
		# print('Difference in pan/tilt estimates:', np.array([pan, tilt])-ifov_estimate, '\n')
		# #Difference in pan/tilt estimates: [-0.24782191 -0.31886772]
		#
		# #liveDemio_output280.txt, frame 382, board is off center, combined extrinsics:
		# ext4x4 = [[0.0474997473085421, -0.9985576522284105, -0.02502776821272445, 0.4424225072221597],
 		# 		  [-0.9914645794769295, -0.04408709815544726, -0.1226960285374687, 0.2252125214354113],
 		# 		  [0.1214156565203186, 0.03064217603756636, -0.9921286687720614, 4.523532721952875],
 		# 		  [0, 0, 0, 1]]
		# Pw = get_charucoboard_center(np.array(ext4x4))
		# print('Pw = ', Pw)
		# pan, tilt = find_needed_pantilt(Pw)
		# print('Pan, Tilt = ', pan, tilt)
		# # Result: Pan, Tilt =  -76.49348250424477 5.443584297297899
		# # "Executing pan/tilt command for pan,tilt =-76.2072, 5.72887"
		# ifov_estimate = [-76.2072, 5.72887]
		# print('Difference in pan/tilt estimates:', np.array([pan, tilt])-ifov_estimate, '\n')
		# #Difference in pan/tilt estimates: [-0.2862825 -0.2852857]
		#
		# #liveDemio_output280.txt, frame 482, board is off center to the right, combined extrinsics:
		# ext4x4 = [[0.03812140140905286, -0.9986032810032401, -0.03658204373971694, 0.7311344915955558],
 		# 		  [-0.9912993500246629, -0.03317634335348279, -0.1273771128671586, 0.2268155878130758],
 		# 		  [0.1259855443901836, 0.04111955023169034, -0.9911795120932699, 4.491704750974595],
 		# 		  [0, 0, 0, 1]]
		# Pw = get_charucoboard_center(np.array(ext4x4))
		# print('Pw = ', Pw)
		# pan, tilt = find_needed_pantilt(Pw)
		# print('Pan, Tilt = ', pan, tilt)
		# # Result: Pan, Tilt =  -79.75397372768275 5.518634962182625
		# # "Executing pan/tilt command for pan,tilt =-79.4713, 5.8213"
		# ifov_estimate = [-79.4713, 5.8213]
		# print('Difference in pan/tilt estimates:', np.array([pan, tilt])-ifov_estimate, '\n')
		# #Difference in pan/tilt estimates: [-0.28267373 -0.30266504]
		#
		# #liveDemio_output280.txt, frame 582, board is off center, combined extrinsics:
		# ext4x4 = [[0.04366327369301942, -0.9979803624687893, -0.04613799580685182, 0.2973297848468853],
 		# 		  [-0.9926749600564635, -0.03813116801442172, -0.1146404714869709, 0.2226792598635662],
 		# 		  [0.1126496436182004, 0.05080561142748433, -0.9923350480760883, 4.501685948000334],
 		# 		  [0, 0, 0, 1]]
		# Pw = get_charucoboard_center(np.array(ext4x4))
		# print('Pw = ', Pw)
		# pan, tilt = find_needed_pantilt(Pw)
		# print('Pan, Tilt = ', pan, tilt)
		# # Result: Pan, Tilt =  -74.81447376320169 5.4190267497491895
		# # "Executing pan/tilt command for pan,tilt =-74.396, 5.71076"
		# ifov_estimate = [-74.396, 5.71076]
		# print('Difference in pan/tilt estimates:', np.array([pan, tilt])-ifov_estimate, '\n')
		# # Difference in pan/tilt estimates: [-0.41847376 -0.29173325]
		# # This one is less accurate since we only see 1/2 of the board, therefore the ext4x4 is not that good
		#
		#
		# #liveDemio_output280.txt, frame 676, board is off center, combined extrinsics:
		# ext4x4 = [[-0.06612874604782149, -0.9978110593149568, 0.0002808111589558127, 0.4536657189882669],
 		# 		  [-0.9976381691658074, 0.06612252201570426, 0.01859826620357904, 0.03035919051447244],
 		# 		  [-0.0185761236440551, 0.0009497320922041974, -0.9998269978547861, 4.476594940762917],
 		# 		  [0, 0, 0, 1]]
		# Pw = get_charucoboard_center(np.array(ext4x4))
		# print('Pw = ', Pw)
		# #pan, tilt = find_needed_pantilt(Pw, -50, -5)
		# pan, tilt = find_needed_pantilt(Pw)
		# print('Pan, Tilt = ', pan, tilt)
		# # Result: Pan, Tilt =  -433.9632631088309 -5936.817847705684   What &*%!
		# # "Executing pan/tilt command for pan,tilt =-76.2053, 3.60665"
		# ifov_estimate = [-76.2053, 3.60665]
		# print('Difference in pan/tilt estimates:', np.array([pan, tilt])-ifov_estimate, '\n')
		# #Difference in pan/tilt estimates: [ -357.75796311 -5940.42449771]
		#
		# #liveDemio_output280.txt, frame 1056, board is off center, combined extrinsics:
		# ext4x4 = [[-0.0495936469274104, -0.9959043225097768, -0.07559795361504282, 0.6870874521919084],
 		# 		  [-0.9971169313020216, 0.05372191180773797, -0.05358900542612302, 0.03386750649540726],
 		# 		  [0.05743078873982855, 0.07272232530704989, -0.9956973274578262, 4.506868062590968],
 		# 		  [0, 0, 0, 1]]
		# Pw = get_charucoboard_center(np.array(ext4x4))
		# print('Pw = ', Pw)
		# pan, tilt = find_needed_pantilt(Pw)
		# print('Pan, Tilt = ', pan, tilt)
		# # Result: Pan, Tilt =  -79.176696482005 3.334598142754291
		# # "Executing pan/tilt command for pan,tilt =-78.9804, 3.64128"
		# ifov_estimate = [-78.9804, 3.64128]
		# print('Difference in pan/tilt estimates:', np.array([pan, tilt])-ifov_estimate, '\n')
		# #Difference in pan/tilt estimates: [-0.19629648 -0.30668186]

