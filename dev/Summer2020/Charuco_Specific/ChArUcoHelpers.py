import numpy as np
import cv2
from helpers.GenericHelpers import *
from functools import reduce
from itertools import *
import math as m
import colorsys
import matplotlib.pyplot as plt
import pdb

def calibrate_camera_Charuco(imgs, charucoX = 24, charucoY = 36, squareLength = 40, markerLength = 30, dictionary = cv2.aruco.DICT_4X4_1000):
    calcorners = []  # 2d points in image
    calids = []  # ids found in image
    h, w = imgs[0].shape

    board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, dictionary)

    # so we know a little bit about the camera, so
    # start off the algorithm with a simple guess
    f = max(h, w)  # focal length is a function of image size in pixels
    K = np.array([
        [f, 0, w // 2],
        [0, f, h // 2],
        [0, 0, 1]
    ])

    for i, im in enumerate(imgs):
        if len(im.shape) > 2:
            gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        else:
            gray = im.copy()

        corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(gray, dictionary)

        if ids is not None and len(ids) > 0:
            ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, board)


            calcorners.append(chcorners)
            calids.append(chids)

            # chids = np.reshape(chids,(chids.shape[0],))
            ids = np.reshape(ids,(ids.shape[0],))
            chcorners = np.asarray(chcorners)
            # corners = np.reshape(corners,(corners.shape[0],2))

        else:
            print("Cant find img: " + str(i))
            continue

        cv2.aruco.drawDetectedCornersCharuco(im, chcorners, chids,(0,255,0))
        # cv2.aruco.drawDetectedMarkers(im, corners, ids=ids, borderColor=(100, 0, 240))

        placeholder = cv2.resize(im, (1000, 1000))

        cv2.imshow("image", placeholder)
        cv2.waitKey(100)

    flags = 0
    flags |= cv2.CALIB_USE_INTRINSIC_GUESS

    rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        calcorners, calids, board, (w, h), K, None, flags=flags)

    h, w = board.chessboardCorners.shape

    imgpts = calcorners
    objpts = [board.chessboardCorners.reshape((h, 1, 3)) for c in calcorners]

    return rms, cameraMatrix, distCoeffs, rvecs, tvecs, objpts, imgpts

def draw_Charuco_markers(img,img_dict,annotate_ids=False):
    try:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    except:
        pass

    ids = np.array(list(img_dict.keys()))
    ids = np.resize(ids, (len(ids),))

    pixel_coordinates = np.array(list(img_dict.values()))
    pixel_coordinates = np.reshape(pixel_coordinates, (len(ids), 1, 2))

    for i,pixel in enumerate(pixel_coordinates):
        pixel = (int(pixel[0][0]), int(pixel[0][1]))

        if annotate_ids == True:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, str(ids[i]), pixel, font, 1, (0, 255, 0), 3, cv2.LINE_AA)
        else:
            cv2.circle(img, pixel, 30, (0, 255, 0), 10)

    return img

def get_common_charuco_corner_ids(img1_dict, img2_dict, board, dictionary):
    img1_dict, _ = charuco_coordinates_as_list(img1_dict, board, dictionary)
    img2_dict, _ = charuco_coordinates_as_list(img2_dict, board, dictionary)

    # if a image dosnt contain a charuco throw away the image
    if(img1_dict is None or img2_dict is None):
        return None,None
    else:
        keys = img1_dict.keys() & img2_dict.keys()

        # grab overlapping ids in both images
        img1_dict = {x: img1_dict[x] for x in keys if x in img1_dict}
        img2_dict = {x: img2_dict[x] for x in keys if x in img2_dict}

        return img1_dict, img2_dict

def get_common_charuco_marker_ids(cameraA, cameraB, board, dictionary):
    _ , img1_dict = charuco_coordinates_as_list(cameraA, board, dictionary)
    _ , img2_dict = charuco_coordinates_as_list(cameraB, board, dictionary)

    # if a image dosnt contain a charuco throw away the image
    if(img1_dict is None or img2_dict is None):
        return None,None
    else:
        keys = img1_dict.keys() & img2_dict.keys()

        # grab overlapping ids in both images
        img1_dict = {x: img1_dict[x] for x in keys if x in img1_dict}
        img2_dict = {x: img2_dict[x] for x in keys if x in img2_dict}

        return img1_dict, img2_dict

# given a image return a dict with a index and (x,y)
# representing the image coordinates
def generate_grid_in_pixels(image, gridsize):
    grid_x, grid_y = gridsize[:2]

    # Create a grid of points based on gridsize of the image in pixel coordinates
    horizontal_grid = [int((i * (image.shape[1]) / grid_x)) for i in range(grid_x+1)]
    vertical_grid = [int((i * (image.shape[0]) / grid_y)) for i in range(grid_y+1)]

    # Convert the list to a dict with keys representing the grid number and pixels
    # example print(grid_dict_x[0],grid_dict_x[2])
    #               (0,1500) , (1500,3000)
    grid_dict_x = {}
    for elem in range(grid_x):
        grid_dict_x[elem] = (horizontal_grid[elem], horizontal_grid[elem + 1])

    grid_dict_y = {}
    for elem in range(grid_y):
        grid_dict_y[elem] = (vertical_grid[elem], vertical_grid[elem + 1])

    return grid_dict_x,grid_dict_y

# given a image of a charucoboard, divide it into grids,
# for every overlapping grid containing a charuco board combine
# upscale points to original image and return
def search_by_crop_for_charucoBoard(image, aruco_dict, cropsize, search_Grid = (300,300)):
    grid_dict_x, grid_dict_y = generate_grid_in_pixels(image,cropsize)

    found_markers_y = []
    found_markers_x = []
    # check if a grid contains a charucoboard
    for i in grid_dict_x:
        for j in grid_dict_y:
            xmin,xmax = grid_dict_x[i]
            ymin,ymax = grid_dict_y[j]
            image_cropped = image[ xmin:xmax , ymin:ymax]


            image_cropped = cv2.resize(image_cropped,search_Grid)

            corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_cropped,aruco_dict)

            if ids is not None:
                # print("ids found")
                found_markers_y.append(ymin)
                found_markers_y.append(ymax)

                found_markers_x.append(xmin)
                found_markers_x.append(xmax)

            # cv2.imshow("img", image_cropped)
            # cv2.waitKey(100000)


    min_x = 0
    max_x = 0

    min_y = 0
    max_y = 0

    try:
        min_x  = min(found_markers_x)
        max_x = max(found_markers_x)

        min_y = min(found_markers_y)
        max_y = max(found_markers_y)
    except:
        return None , None

    # pad image if smaller than original
    max_x += 100
    max_y += 100

    if(min_y < 100):
        min_y = 0
    else:
        min_y -= 100

    if(min_x < 100):
        min_x = 0
    else:
        min_x -= 100
    
    return image[min_x:max_x ,min_y:max_y], np.array([[min_y, min_x]])

def estimate_Pose_Charucoboard_Ximea(img, board, intrinsics, dist, aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), subsampling=False, debug_verbose = True):
    extrinsics_4x4 = None
    reprojection_error = None
    # Aaron: From now on assume that image passed in is blurred
    # img = cv2.blur(img,(3,3))
    # img = adjust_gamma(img,1.5)

    if subsampling == False:
        extrinsics_4x4
        markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(img, aruco_dict)
        if (ids is not None and len(ids) > 5):
            # use original image for interpolation
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markers, ids, img, board)
            # Offset Detected Image to correspond with upsampled imperx
            extrinsics_4x4,reprojection_error = get_pose(markers, ids, chorners, chids, board, intrinsics, dist, debug_verbose)
    else:
        img, pixel_coordinates_in_orignal_image = search_by_crop_for_charucoBoard(img, aruco_dict, [3, 3], (1000, 1000))
        if img is not None:
            markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(img, aruco_dict)
            if (ids is not None and len(ids) > 5):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markers, ids, img, board)

                # Offset Detected Image to correspond with upsampled imperx
                offset_imper_cornerx = np.tile(pixel_coordinates_in_orignal_image, (chorners.shape[0], 1, 1))
                chorners = np.add(chorners, offset_imper_cornerx)

                if(debug_verbose is True):
                    print("Ximea Extrinsics")
                extrinsics_4x4, _ = get_pose(markers, ids, chorners, chids, board, intrinsics, dist, debug_verbose)

    return extrinsics_4x4,reprojection_error,chorners

def estimate_Pose_Charucoboard_Imperx(img, board, intrinsics, dist, aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), subsampling=False , debug_verbose = True):
    extrinsics_4x4 = None
    reprojection_error = None
    chorners = None
    
    if subsampling == False:
        extrinsics_4x4
        markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(img, aruco_dict)

        if (ids is not None and len(ids) > 5):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markers, ids, img, board)
            # Offset Detected Image to correspond with upsampled imperx
            extrinsics_4x4, reprojection_error = get_pose(markers, ids, chorners, chids, board, intrinsics, dist, debug_verbose)
    else:
        img, pixel_coordinates_in_original_image = search_by_crop_for_charucoBoard(img, aruco_dict, [3, 3], (1000, 1000))
        pdb.set_trace()
        if img is not None:
            markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(img, aruco_dict)
            if (ids is not None and len(ids) > 5):
                ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markers, ids, img, board)

                # Offset Detected Image to correspond with upsampled imperx
                offset_imper_cornerx = np.tile(pixel_coordinates_in_orignal_image, (chorners.shape[0], 1, 1))
                chorners = np.add(chorners, offset_imper_cornerx)

                if(debug_verbose is True):
                    print("Imperx Extrinsics")

                extrinsics_4x4, _ = get_pose(markers, ids, chorners, chids, board, intrinsics, dist, debug_verbose)


    return extrinsics_4x4,reprojection_error,chorners

def estimate_Pose_Charucoboard(img, board, intrinsics, dist, aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), debug_verbose = False):
    extrinsics_4x4 = None
    markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(img, aruco_dict)

    # if we dont find enough points return None
    if (ids is not None and len(ids) > 8):
        ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markers, ids, img, board)
        retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, useExtrinsicGuess=False)

        extrinsics_4x4 = get_pose(markers,chorners,chids,board,intrinsics,dist,debug_verbose)

    return extrinsics_4x4

def get_pose(markers,ids,chorners,chids,board,intrinsics,dist,debug_verbose):
    extrinsics_4x4 = []
    
    # Aaron modified because was throwing error that missing rvec, tvec so just put None, None
    retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)

    if rvec is None:
        return extrinsics_4x4

    R, _ = cv2.Rodrigues(rvec)
    stacked = np.hstack(([R,tvec]))
    extrinsics_4x4 = np.vstack((stacked,[0,0,0,1]))

    mean_error = charuco_reprojection_error(board,chorners,chids,R,tvec,intrinsics,dist)

    if(debug_verbose == True):

        print("Number of Features: " + str(len(markers)))

        # print(len(chorners))

        print("Nuber of Detected Corners: " + str(len(chorners)))

        if(extrinsics_4x4 is None):
            print("TVEC: None")
            print("RVEC: None")
            print("\n")
        else:
            R,tvec = decompose_Extrinsics(extrinsics_4x4)
            print("TVEC: " + str(tvec))
            print("RVEC: " + str(rvec))
            print("ERROR: " + str(mean_error))
            print("\n")


    return extrinsics_4x4, mean_error


def ximea_to_Imperx_frame(Ximea_Extrinsics_4x4,rel_extrinsics_4x4):
    return np.matmul(rel_extrinsics_4x4,Ximea_Extrinsics_4x4)

def relative_Extrinsics_Ximea_to_Imperx(ximea_4x4,imperx_4x4):
    ximea_4x4_intverted = np.linalg.inv(ximea_4x4)
    return np.matmul(imperx_4x4,ximea_4x4_intverted)

def charuco_coordinates_as_list(img,board,dictionary):
    # Convert image to greyscale
    if len(img.shape) > 2:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    # extract coordinates in each image
    corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(gray, dictionary)

    # if image has ids
    if ids is not None and len(ids) > 0:
        ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board)

        # charuco corner U,V position
        chids = np.reshape(chids, (chids.shape[0],))
        chids = chids.tolist()
        chcorners = np.reshape(chcorners, (chcorners.shape[0], 2))
        chcorners = chcorners.tolist()

        # shove info into a dict for easier processing later
        id_and_pixel = dict((chids[i], c) for i, c in enumerate(chcorners))


        # charuco marker points
        ids = np.reshape(ids, (ids.shape[0],))
        ids = ids.tolist()

        corners = np.asarray(corners)
        corners = np.reshape(corners, (corners.shape[0], 4, 2))
        corners = corners.tolist()

        # shove info into a dict for easier processing later
        id_and_marker = dict((ids[i], c) for i, c in enumerate(corners))
    else:
        id_and_pixel = None
        id_and_marker = None

    return id_and_pixel, id_and_marker

def charuco_coordinates_as_list(chids,chcorners):
    # if image has ids
    if chids is not None and len(chids) > 0:
        chids = np.reshape(chids, (chids.shape[0],))

        chids = chids.tolist()
        chcorners = np.reshape(chcorners, (chcorners.shape[0], 2))
        chcorners = chcorners.tolist()

        # shove info into a dict for easier processing later
        id_and_pixel = dict((chids[i], c) for i, c in enumerate(chcorners))
    else:
        id_and_pixel = None

    return id_and_pixel


#################################
#  corners, ids is for markers  #
#################################
def charuco_reprojection_error(board,corners,ids,rvec,tvec,intrinsics,dist,markers=False):
    # if(markers == True):
    #     charuco_RMS_reprojection_error(board, corners, ids, rvec, tvec, intrinsics, dist)
    # else:
    # Aaron modified so ignore the error vector and only look at norm
    _, error = charuco_reprojection_error_for_every_corner(board, corners, ids, rvec, tvec, intrinsics, dist)
    mean_error = reduce(lambda x, y: x + y, error) / len(error)

    return mean_error

def charuco_squared_reprojection_error(board,corners,ids,rvec,tvec,intrinsics,dist):
    _, error = charuco_reprojection_error_for_every_corner(board, corners, ids, rvec, tvec, intrinsics, dist)
    return np.mean(np.square(np.array(error)))
    
# computes RMS error given a img and objpoints
def charuco_RMS_reprojection_error(board,corners,ids,rvec,tvec,intrinsics,dist):
    #objpoints , imgpoints = cv2.aruco.getBoardObjectAndImagePoints(board,corners,ids)
    #rp_l, _ = cv2.projectPoints(objpoints, rvec, tvec,intrinsics,dist)
    ## Aaron changed to be np.sqrt on outside was np.square
    #mean_error = np.sqrt(np.mean(np.square(np.float64(imgpoints - rp_l))))
    ## mean_error = np.sqrt(tot_error / len(objpoints))
    err_vecs, _ = charuco_reprojection_error_for_every_corner(board, corners, ids, rvec, tvec, intrinsics, dist)
      
    rms_error = np.linalg.norm(err_vecs)/math.sqrt(2*len(err_vecs))
    
    return rms_error

def RMS_Error_from_charuco_image(image,aruco_dict,board,rvec,tvec,intrinsics,dist):
    # Aaron: Will assume that image passed in is blurred
    #image_copied = cv2.blur(image,(3,3))
    image_copied = np.copy(image)
    markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_copied, aruco_dict)
    return charuco_RMS_reprojection_error(board,markers,ids,rvec,tvec,intrinsics,dist)

# generates a list of reprojection errors given a image and objpoints
def charuco_reprojection_error_for_every_marker(board, corners, ids, rvec, tvec, intrinsics, dist):
    objpoints, imgpoints = cv2.aruco.getBoardObjectAndImagePoints(board, corners, ids)
    rp_l, _ = cv2.projectPoints(objpoints, rvec, tvec, intrinsics, dist)
    rp_l_list = rp_l.tolist()
    stufflist = zip(imgpoints.tolist(), rp_l_list)

    #euclidean distance
    return list(map(lambda x: cv2.norm(np.array(x[0]) - np.array(x[1])), stufflist))

def charuco_reprojection_error_for_every_corner(board, chcorners, chids, rvec, tvec, intrinsics, dist):
    rp_l, _ = cv2.projectPoints(board.chessboardCorners, rvec, tvec, intrinsics, dist)
    rp_l = rp_l.tolist()
    rp_l = list(map(lambda x: x[0], rp_l))

    chids = chids.tolist()
    chids = list(map(lambda x: x[0], chids))
    rp_l = list(map(lambda x: rp_l[x], chids))

    chcorners = chcorners.tolist()
    chcorners = list(map(lambda x: x[0], chcorners))

    stufflist = zip(chcorners, rp_l)

    # Aaron modified so returns both a list of the error vectors and 
    # a separate list of the corresponding norms
    
    error_vectors = list(map(lambda x: np.array(x[0]) - np.array(x[1]), stufflist))
    error_dists = [cv2.norm(err) for err in error_vectors]
    #list(map(lambda x: cv2.norm(np.array(x[0]) - np.array(x[1])), stufflist))
        
    return error_vectors, error_dists

def estimate_pose_ransac(board, chcorners, chids, intrinsics, dist, iterations=100, maxErr=8, fractionInlier=0.9):
    # First, identify the corresponding chessboard corners
    objectPoints = np.squeeze(np.array([board.chessboardCorners[chid] for chid in chids]))
    # Image points are the chcorners
    return cv2.solvePnPRansac(objectPoints, chcorners, intrinsics, dist, None, None, False, iterations, maxErr, fractionInlier)

# Function will generate a quiver plot of the actual dectected points vs the reprojected points
# marker based detection pass in marker = True, corner based marker = False
# exageration_factor = how much we want to exagerate the error
# Aaron added that can pass in modified parameters
def generate_reprojection_error_quiver_plot_markers(image,aruco_dict,board,rvec,tvec,intrinsics,dist,parameters=cv2.aruco.DetectorParameters_create(),exageration_factor = 8,QUIVER=True):
    squareLength = board['squareLength']
    markerLength = board['markerLength']
    charucoX = board['charucoX']
    charucoY = board['charucoY']

    board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

    # board positions
    # Aaron: Will from now on assume that image passed in is already blurred
    #image_copied = cv2.blur(image,(3,3))
    image_copied = np.copy(image)
    image_copied = cv2.cvtColor(image_copied,cv2.COLOR_GRAY2BGR)

    # markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_copied, aruco_dict)
    markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_copied, aruco_dict, None, None, parameters)
    markers, ids, rej, recovered_ids = cv2.aruco.refineDetectedMarkers(image_copied, board_aruco, markers, ids, rejectedImgPts)

    if(QUIVER == False):
        objpoints, imgpoints = cv2.aruco.getBoardObjectAndImagePoints(board, markers, ids)
        reprojection_error = charuco_reprojection_error_for_every_marker(board, markers, ids, rvec, tvec, intrinsics,
                                                                         dist)
        rp_l, _ = cv2.projectPoints(objpoints, rvec, tvec, intrinsics, dist)

        for i, point in enumerate(imgpoints):
            cv2.circle(image_copied, (point[0][0], point[0][1]), exageration_factor * int(reprojection_error[i]), (0, 0, 255), 20)

    objpoints, imgpoints = cv2.aruco.getBoardObjectAndImagePoints(board, markers, ids)
    reprojection_error = charuco_reprojection_error_for_every_marker(board, markers, ids, rvec, tvec, intrinsics,
                                                                     dist)
    rp_l, _ = cv2.projectPoints(objpoints, rvec, tvec, intrinsics, dist)

    for i, point in enumerate(imgpoints):
        error_term = exageration_factor*reprojection_error[i]
        cv2.arrowedLine(image_copied, tuple(point[0]), tuple(np.add(point[0],error_term).astype(int)), (0, 0, 255),exageration_factor,tipLength=.6)

    return image_copied

# Aaron added function which will instead plot a heat map for the error
# image: image to draw heat map on
# aruco_dict: the aruco dictionary to use for detection
# board: dict describing the board parameters
# rvec: Rotation vector to use for projection
# tvec: translation vector to use for projection
# intrinsics: cameraMatrix
# dist: distortion coeffs
# radius: radius of drawn circles
# scale: tuple of form (low, high) where any error <= low gets mapped to 
# coolest color and >= high mapped to hottest color
def generate_reprojection_error_heat(image,aruco_dict,board,rvec,tvec,intrinsics,dist,parameters=cv2.aruco.DetectorParameters_create(),radius=8,scale=None):    
    # board positions
    # Aaron: Will from now on assume that image passed in is already blurred
    # image_copied = cv2.blur(image,(3,3))
    image_copied = np.copy(image)
    image_copied = cv2.cvtColor(image_copied,cv2.COLOR_GRAY2BGR)

    squareLength = board['squareLength']
    markerLength = board['markerLength']
    charucoX = board['charucoX']
    charucoY = board['charucoY']

    board_aruco = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
    rp_l, _ = cv2.projectPoints(board_aruco.chessboardCorners, rvec, tvec, intrinsics, dist)
    # print(rp_l)

    #markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_copied, aruco_dict)
    markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_copied, aruco_dict, None, None, parameters)
    markers, ids, rej, recovered_ids = cv2.aruco.refineDetectedMarkers(image_copied, board_aruco, markers, ids, rejectedImgPts)
    ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
        markers, ids, image_copied, board_aruco)

    # First reurn value of the vectors can be ignored in this func
    _, reprojection_error = charuco_reprojection_error_for_every_corner(board_aruco, chcorners, chids, rvec, tvec, intrinsics, dist)

    if scale is None:
        scale = (np.min(reprojection_error), np.max(reprojection_error))
    
    for i, point in enumerate(chcorners):
        # On a [0,1] scale if used default
        color = (reprojection_error[i] - scale[0]) / (scale[1] - scale[0])
        # Bound to be 0 to 1
        color = np.clip(color, 0, 1)
        # Actually rescale it so only goes up to 0.75 and 0 is red so invert
        r,g,b = colorsys.hls_to_rgb(0.75 * (1 - color), 0.5, 1) # Set l and s by default
        # Make them int and 0 to 255
        r = int(r * 255)
        g = int(g * 255)
        b = int(b * 255)
        cv2.circle(image_copied, (point[0][0], point[0][1]), radius, tuple([b, g, r]), 20)

    return image_copied

# this function usees the boards dict not the board object used by cv2.aruco
def generate_reprojection_error_quiver_plot_ids(image,aruco_dict,board,rvec,tvec,intrinsics,dist,parameters=cv2.aruco.DetectorParameters_create(),exageration_factor = 30):
    # board positions
    # Aaron: Will from now on assume that image is blurred when passed in
    #image_copied = cv2.blur(image,(3,3))
    image_copied = np.copy(image)
    image_copied = cv2.cvtColor(image_copied,cv2.COLOR_GRAY2BGR)

    squareLength = board['squareLength']
    markerLength = board['markerLength']
    charucoX = board['charucoX']
    charucoY = board['charucoY']

    board_aruco = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
    rp_l, _ = cv2.projectPoints(board_aruco.chessboardCorners, rvec, tvec, intrinsics, dist)
    # print(rp_l)


    # markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_copied, aruco_dict)
    # Aaron added so that could change detection parameters
    markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(image_copied, aruco_dict, None, None, parameters)
    markers, ids, rej, recovered_ids = cv2.aruco.refineDetectedMarkers(image_copied, board_aruco, markers, ids, rejectedImgPts)
    ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
        markers, ids, image_copied, board_aruco)

    error_vectors, reprojection_error = charuco_reprojection_error_for_every_corner(board_aruco, chcorners, chids, rvec, tvec, intrinsics, dist)
        
    for i, point in enumerate(chcorners):
        # Aaron modified so would use the error vector not norm
        #error_term = exageration_factor*reprojection_error[i]
        error_term = exageration_factor*error_vectors[i]
        cv2.arrowedLine(image_copied, tuple(point[0]), tuple(np.add(point[0],error_term).astype(int)), (0, 0, 255),exageration_factor,tipLength=.6)

    return image_copied