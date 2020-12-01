// PoseEstimator.cpp : Source file for determining the 6DOF pose of a fiducial in a single frame of video
//

#include "PoseEstimator.h"
#include "CppNpy/numpy.h"
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

#ifdef WIN32 
#define _USE_MATH_DEFINES // for C++
#include <cmath>
// the above does not work
#define	M_PI 3.141592653589793238462643383279502884197169399375105820974944592307
#else
#include <math.h>
#endif

using namespace std;


template<typename T>
constexpr T SQ(T x) { return ((x)*(x)); }

double rad2deg(double radians) { return radians * 180.0 / M_PI; }
double deg2rad(double degrees) { return degrees * M_PI / 180.0; }
const int num_markers_threshold = 20;


//#######################################################################################
bool isRotationMatrix(const cv::Mat &R)
{
    // Determine if a matrix is a valid rotation matrix
    // https://www.learnopencv.com/rotation-matrix-to-euler-angles/

    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return cv::norm(I, shouldBeIdentity) < 1.e-6;
}

//######################################################################################
void dumpRvecTvec(const cv::Mat & rvec, cv::Mat & tvec)
{
    //Print rvec (given in radians, print in degrees) and tvec on the console
    cv::Mat rvecDeg = rvec.clone();
    std::cout << "rvec(deg) = "<< rvecDeg.t()/M_PI*180.0 << endl;
    std::cout << "tvec = "<< tvec.t() << endl;
}

//#######################################################################################
bool PoseEstimator::init(string modelFilename, string ximea_distCoeffs, string ximea_intrinsics, const double scaleFactor, bool Debug)
{
    debug_verbose = Debug;
    if (!cad.loadModelFile(modelFilename)) {
        cout << "ERROR! Unable to load model JSON file: " << modelFilename << endl;
        return false;
    }
    aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    board = cv::aruco::CharucoBoard::create(charucoX, charucoY, squareLength, markerLength, aruco_dict);

    // Load the intrinsics & distCoeffs from .npy files, a 3x3 and a 1x5 matrix respectively
    vector<int>shape;
    vector<double> data;
    //load camera matrix
    try
    {
        aoba::LoadArrayFromNumpy(ximea_intrinsics, shape, data);
        if (shape[0]!=shape[1] || shape[0] !=3)
            throw "Invalid intrinsics matrix dimensions, expecting 3 x 3.";
    }
    catch(exception & e)
    {
        std::cerr <<"Error while trying to load camera intrinsics: "<< e.what() << std::endl;
    }
    //std::cout << "Ximea intrinsics: " << shape[0] << ", "<< shape[1] << std::endl;
    intrinsics = cv::Mat(shape, CV_64F, data.data()).clone();
    if (scaleFactor != 1.0)
    {
        intrinsics.at<double>(0,0) /= scaleFactor; // focal length in X
        intrinsics.at<double>(1,1) /= scaleFactor; // focal length in Y
        intrinsics.at<double>(0,2) /= scaleFactor; // principle point in X
        intrinsics.at<double>(1,2) /= scaleFactor; // principle point in Y
    }
    if(debug_verbose)
        std::cout << intrinsics << std::endl;
    // Load distCoeffs
    try
    {
        aoba::LoadArrayFromNumpy(ximea_distCoeffs, shape, data); // there is only 1 row of data (a vector)
        if (shape[0] !=1 || shape[1] != 5)
            throw "Invalid distCoeffs matrix dimensions, expecting 1 x 5.";
    }
    catch(exception & e)
    {
        std::cerr <<"Error while trying to load camera distCoeffs: "<< e.what() << std::endl;
    }
    //std::cout << "Ximea distCoeffs: " << shape[0] << ", "<< shape[1] << std::endl;
    distCoeffs = cv::Mat(shape, CV_64F, data.data()).clone();
    if(debug_verbose)
        std::cout << distCoeffs << std::endl;
    
    // NOTE: I hard-code instead of reading from files here, but this will not affect the program speed since this is a one-time step at the beginning of the run.
    // I'm going to hardcode all of this for now and read from a file later.
    // NOTE: Since we performed the intrinsic calibration on the full resolution and want to experiment on reduced resolution video, I introduced
    //       the 'scaleFactor' value to adjust the intrinsics to account for the smaller frame. This is far from perfect and won't be used in the "real" 
    //       system, but it at least puts our 6 DOF estimates in the correct ballpark and does not affect the speed of the algorithm in any way.
    /*
    intrinsics = cv::Mat(cv::Size(3, 3), CV_64F);
    intrinsics.at<double>(0, 0) = 4.017940916843801824e+04/scaleFactor;     intrinsics.at<double>(0, 1) = 0.0;                                      intrinsics.at<double>(0, 2) = 3.327041961354727391e+03/scaleFactor;
    intrinsics.at<double>(1, 0) = 0.0;                                      intrinsics.at<double>(1, 1) = 4.015287513689287880e+04/scaleFactor;     intrinsics.at<double>(1, 2) = 3.036369767164601399e+03/scaleFactor;
    intrinsics.at<double>(2, 0) = 0.0;                                      intrinsics.at<double>(2, 1) = 0.0;                                      intrinsics.at<double>(2, 2) = 1.0;

    // TODO: Load the distortion coefficients from text file. It contains a single row of 5 values.
    distCoeffs.resize(5);
    distCoeffs[0] =  1.102432398361482324e+00;        // Again, hard-coding the values for the Dec 13 milestone and read from file later.
    distCoeffs[1] =  2.057761241969464638e-01;
    distCoeffs[2] = -7.627109305368789905e-04;
    distCoeffs[3] = -2.394940093281002630e-02;
    distCoeffs[4] =  1.230897535935005571e+01;
    */
    
    // Preallocate memory for our transform matrices so that we aren't constantly reallocating and freeing space.
    r_base_imperx = cv::Mat::eye(cv::Size(4, 4), CV_64F);
    t_base_imperx = cv::Mat::eye(cv::Size(4, 4), CV_64F);
    r_pan_base = cv::Mat::eye(cv::Size(4, 4), CV_64F);
    t_tilt_pan = cv::Mat::eye(cv::Size(4, 4), CV_64F);
    r_tilt_pan = cv::Mat::eye(cv::Size(4, 4), CV_64F);
    t_ximea_tilt = cv::Mat::eye(cv::Size(4, 4), CV_64F);
    r_ximea_tilt = cv::Mat::eye(cv::Size(4, 4), CV_64F);

    tf = cv::Mat::zeros(cv::Size(4, 4), CV_64F);

    return true;
}

// Return average focal length
double PoseEstimator::get_focal_length()
{
    return (intrinsics.at<double>(0, 0) + intrinsics.at<double>(1, 1))/2.0;
}

// Populate pt_model structure with the given JSON file path
// If activateModel is true, then will enable the model so that PoseEstimator::calc() will use it
bool PoseEstimator::init2(const string & jsonModelPath, bool activateModel)
{
    if(debug_verbose) cout <<"Loading JSON PT cad model from: "<<jsonModelPath<< endl;
    pt_model.loadModel(jsonModelPath);
    if (debug_verbose)
    {
        cout << "Contents of Loaded PT cad model: " << endl;
        pt_model.displayModel();
    }
    // turn on the flag to force PoseEstimator::calc() to use
    usePantiltModel = activateModel;
    
    return true;
}


//#######################################################################################
bool PoseEstimator::calc(cv::Mat &frame, const double pan, const double tilt, SixDOF *result)
{
    // For a given image and pan-tilt pair, compute the 6 DOF of a fiducial in the image and return it.

    int charuco_sq_found = -1;
    double repro_err = 0.0;
    cv::Mat ximea_extrinsics, combined_extrinsics;
    if(estimate_pose_charucoboard_ximea(frame, &ximea_extrinsics, &repro_err, &charuco_sq_found))
    {
        // The function 'estimate_pose_charucoboard_ximea' returns 'true' if a board was detected
        // output are placed in ximea_extrinsics, repro_err, and charuco_sq_found
        if (usePantiltModel)
        {
            if (debug_verbose)
            {
                std::cout << "Warning: using PanTiltModel.apply() ..." << std::endl;
                std::cout << "pan, tilt = " << pan << ", " << tilt << " (" << deg2rad(pan) << ", " << deg2rad(tilt) << ")" << std::endl;
                std::cout << "ximea_extrinsics = " << std::endl;
                std::cout << ximea_extrinsics << std::endl;
            }
            //combined_extrinsics = ximea_extrinsics;
            //pt_model.apply(combined_extrinsics, deg2rad(pan), deg2rad(tilt));
            cv::Mat dynamic_extrinsics = cv::Mat::eye(4, 4, CV_64F);
            pt_model.apply(dynamic_extrinsics, deg2rad(pan), deg2rad(tilt));
            combined_extrinsics = dynamic_extrinsics.inv() * ximea_extrinsics;
            if (debug_verbose)
            {
                std::cout << "combined_extrinsics = " << std::endl;
                std::cout << combined_extrinsics << std::endl;
            }
        }
        else
        {
            dynamic_extrinsics_correct_order(-1.0*deg2rad(tilt), deg2rad(90.0-pan));

            //    PYTHON: combined_extrinsics = ximea_to_Imperx_frame(ximea_extrinsics, tf);
            /*
                PYTHON CODE:
                def ximea_to_Imperx_frame(Ximea_Extrinsics_4x4,rel_extrinsics_4x4):
                    return np.matmul(rel_extrinsics_4x4,Ximea_Extrinsics_4x4)
            */
            combined_extrinsics = tf * ximea_extrinsics;
        }
        
        // Decompose the extrinsics matrix back into a 3x3 rotation matrix and a 3x1 translation matrix
        cv::Mat rmat_w1, tvec_w1;
        decompose_extrinsics(combined_extrinsics, &rmat_w1, &tvec_w1);

        // Recover the Euler angles from the 3x3 rotation matrix
        cv::Mat euler_angle_w1;
        
        if(!mat2euler(rmat_w1, &euler_angle_w1)) {
            cout << "ERROR! This is not a valid rotation matrix:" << endl << rmat_w1 << endl;
            return false; // 'rmat' is not a valid rotation matrix
        }
        // Populate the 6 DOF object with the results from rvec and tvec
        result->pitch = rad2deg(euler_angle_w1.at<double>(0, 0));      // rotation about the x-axis (in degrees)
        result->yaw   = rad2deg(euler_angle_w1.at<double>(1, 0));        // rotation about the y-axis (in degrees)
        result->roll  = rad2deg(euler_angle_w1.at<double>(2, 0));       // rotation about the z-axis (in degrees)
        result->x = tvec_w1.at<double>(0, 0);                   // translation along the x-axis (in meters)
        result->y = tvec_w1.at<double>(1, 0);                   // translation along the y-axis (in meters)
        result->z = tvec_w1.at<double>(2, 0);                   // translation along the z-axis (in meters)

        result->pan = pan;                                      // provided pan value of the PT unit
        result->tilt = tilt;                                    // provided tilt value of the PT unit

        result->num_charuco_found = charuco_sq_found;
    }
    else
    {
        // If a ChArUco board is not detected (i.e., 'estimate_pose_charucoboard_ximea' returns 'false'), then there isn't any point 
        // in going through the other processing steps. Just return quickly without doing any processing and grab the next frame.
        result->num_charuco_found = charuco_sq_found;
        return false;
    }

    return true;
}

//#######################################################################################
bool PoseEstimator::mat2euler(const cv::Mat &rmat, cv::Mat *rvec) const
{
    // Transform a rotation matrix into euler angles (roll, pitch, yaw)
    // https://www.learnopencv.com/rotation-matrix-to-euler-angles/

    if(!isRotationMatrix(rmat)) {
        cout << "ERROR! This is not a valid rotation matrix:" << endl << rmat << endl;
        return false;
    }

    rvec->create(3, 1, CV_64F);
    double sy = sqrt(SQ(rmat.at<double>(0, 0)) + SQ(rmat.at<double>(1, 0)));
    if(sy < 1.e-6)
    {
        // singular
        rvec->at<double>(0, 0) = atan2(-rmat.at<double>(1,2), rmat.at<double>(1,1));
        rvec->at<double>(1, 0) = atan2(-rmat.at<double>(2,0), sy);
        rvec->at<double>(2, 0) = 0.0;
    }
    else
    {
        // Not singular
        rvec->at<double>(0, 0) = atan2(rmat.at<double>(2,1), rmat.at<double>(2,2));
        rvec->at<double>(1, 0) = atan2(-rmat.at<double>(2,0), sy);
        rvec->at<double>(2, 0) = atan2(rmat.at<double>(1,0), rmat.at<double>(0,0));
    }
    return true;
}

//#######################################################################################
bool PoseEstimator::decompose_extrinsics(const cv::Mat &extrinsics_4x4, cv::Mat *rmat, cv::Mat *tvec) const
{
    // The rotation matrix is the upper-left 3x3 submatrix of the 4x4 extrinsics matrix
    rmat->create(3, 3, CV_64F);
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            rmat->at<double>(row, col) = extrinsics_4x4.at<double>(row, col);
        }
    }

    // The translation vector is the last column of the 4x4 extrinsics matrix
    tvec->create(3, 1, CV_64F);
    for(int row = 0; row < 3; ++row) {
        tvec->at<double>(row, 0) = extrinsics_4x4.at<double>(row, 3);
    }

    return true;
}

//#######################################################################################
bool PoseEstimator::dynamic_extrinsics_correct_order(const double tilt_rad, const double pan_rad)
{
    // Numerical extrinsics: Return the correct transform matrix based on the supplied pan and tilt angles and the CAD model.
/*
    PYTHON CODE:
    def dynamic_extrinsics_correct_order(cad_model, tilt, pan):
        # Extract the values from cad_model:
        # Numerical Extrinsics
        # Ximea to Tilt consists of an arbitrary translation and rotation
        yaw_ximea_tilt = cad_model[0]
        pitch_ximea_tilt = cad_model[1]
        roll_ximea_tilt = cad_model[2]
        x_ximea_tilt = cad_model[3]
        y_ximea_tilt = cad_model[4]
        z_ximea_tilt = cad_model[5]

        r_ximea_tilt = matrices.rotation(pitch_ximea_tilt, yaw_ximea_tilt, roll_ximea_tilt)
        t_ximea_tilt = matrices.translation(x_ximea_tilt, y_ximea_tilt, z_ximea_tilt)

        # Tilt to Pan is a pitch followed by a translation
        pitch_tilt_pan = tilt
        x_tilt_pan = cad_model[6]
        y_tilt_pan = cad_model[7]
        z_tilt_pan = cad_model[8]

        r_tilt_pan = matrices.rotation(pitch_tilt_pan, 0, 0)
        t_tilt_pan = matrices.translation(x_tilt_pan, y_tilt_pan, z_tilt_pan)

        # Pan to Base is simply a pan (yaw) rotation
        yaw_pan_base = pan;

        r_pan_base = matrices.rotation(0, yaw_pan_base, 0)

        # Base to Imperx is another arbitrary translation and rotation
        yaw_base_imperx = cad_model[9]
        pitch_base_imperx = cad_model[10]
        roll_base_imperx = cad_model[11]
        x_base_imperx = cad_model[12]
        y_base_imperx = cad_model[13]
        z_base_imperx = cad_model[14]

        r_base_imperx = matrices.rotation(pitch_base_imperx, yaw_base_imperx, roll_base_imperx)
        t_base_imperx = matrices.translation(x_base_imperx, y_base_imperx, z_base_imperx)

        return r_base_imperx @ t_base_imperx @ r_pan_base @ t_tilt_pan @ r_tilt_pan @ t_ximea_tilt @ r_ximea_tilt
*/

    // Ximea to Tilt consists of an arbitrary translation and rotation
    doRotation(cad.pitch_ximea_tilt, cad.yaw_ximea_tilt, cad.roll_ximea_tilt, &r_ximea_tilt);
    doTranslation(cad.x_ximea_tilt, cad.y_ximea_tilt, cad.z_ximea_tilt, &t_ximea_tilt);

    // Tilt to Pan is a pitch followed by a translation
    doRotation(tilt_rad, 0.0, 0.0, &r_tilt_pan);
    doTranslation(cad.x_tilt_pan, cad.y_tilt_pan, cad.z_tilt_pan, &t_tilt_pan);

    // Pan to Base is simply a pan (yaw) rotation
    doRotation(0.0, pan_rad, 0.0, &r_pan_base);

    // Base to Imperx is another arbitrary translation and rotation
    doRotation(cad.pitch_base_imperx, cad.yaw_base_imperx, cad.roll_base_imperx, &r_base_imperx);
    doTranslation(cad.x_base_imperx, cad.y_base_imperx, cad.z_base_imperx, &t_base_imperx);

    // What we want to return is the matrix product of all of these matrices
    tf = r_base_imperx * t_base_imperx * r_pan_base * r_tilt_pan * t_tilt_pan * t_ximea_tilt * r_ximea_tilt;

    return true;
}

//#######################################################################################
bool PoseEstimator::estimate_pose_charucoboard_ximea(const cv::Mat &img, cv::Mat *extrinsics_4x4, double *reprojection_error, int *charuco_sq_found)
{
    /*
        PYTHON CODE:
        def estimate_Pose_Charucoboard_Ximea(img, board, intrinsics, dist, aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), subsampling=False, debug_verbose = False):
            extrinsics_4x4 = None
            reprojection_error = None
            img = cv2.blur(img,(3,3))

            if subsampling == False:
                extrinsics_4x4
                markers, ids, rejectedImgPts = cv2.aruco.detectMarkers(img, aruco_dict)
                if (ids is not None and len(ids) > 5):
                    ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markers, ids, img, board)
                    # Offset Detected Image to correspond with upsampled imperx
                    extrinsics_4x4, reprojection_error = get_pose(markers, ids, chorners, chids, board, intrinsics, dist, debug_verbose)
            else:
                ...

            return extrinsics_4x4, reprojection_error
    */

//  img = cv::blur(img, (3, 3));        // skipping this for now since we downsample
    if(!subsampling)
    {
         auto  start_time = chrono::high_resolution_clock::now();
        // No subsampling yet, since we are running on reduced-resolution images anyway
        vector<int> ids;
        vector<vector<cv::Point2f> > markers;
        cv::aruco::detectMarkers(img, aruco_dict, markers, ids);
        *charuco_sq_found = (int)ids.size();
        //https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html
        //cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        
        double processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start_time).count();
            
        // this is here for debugging - not part of the Python code
        cv::aruco::drawDetectedMarkers(img, markers, ids);  

        if (debug_verbose)
            cout << "Estimating pose: number of detected aruco markers: " << ids.size() << " (" << processing_time <<" ms)" << endl;
        if (!ids.empty() && ids.size() > num_markers_threshold)
        {
            // chorners and chids are returned from 'interpolateCornersCharuco': chorners = ChArUco corners; chids = ChArUco ids
            vector<int> chIds;
            vector<cv::Point2f> chCorners;
            cv::aruco::interpolateCornersCharuco(markers, ids, img, board, chCorners, chIds);
            
            if (debug_verbose)
            {
                std::cout << "Estimating pose: number of interpolated chessboard corners: " << chIds.size() << std::endl;
            }
            
            get_pose(markers, ids, chCorners, chIds, extrinsics_4x4, reprojection_error);
            processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start_time).count();
            if (debug_verbose)
            {
                cout << "Total marker detection + pose estimation time: "<< processing_time << " ms" << endl;
                cout << "Reprojection error (RMS): " <<*reprojection_error<< endl;
            }
            // Draw XYZ axes for the board pose; rvec_tmp and tvect_tmp are calculated in get_pose() 
            cv::aruco::drawAxis(img, intrinsics, distCoeffs, rvec_tmp, tvec_tmp, 0.02);
            //cv::drawFrameAxes(img, intrinsics, distCoeffs, rvec_tmp, tvec_tmp, 0.05, 5);
        }
        else
        {
            return false;  // 'extrinsics_4x4' and 'reprojection_error' will not be defined/filled in this case
        }
    }
    else
    {
        cout << "I haven't implemented this option yet because we never used it" << endl;
        return false;
    }

    return true;
}

// Return true if # of detected marker exceeds threshold, and the marker corner of the center marker
// in centerMarkerCorner or (-1.0, -1.0) if the center markers are not detected
// Optionally draw detected markers on the input image if drawMarkers is true.
bool PoseEstimator::detect_aruco_markers(cv::Mat &img, cv::Point2f & centerMarkerCorner, bool drawMarkers)
{
    if (debug_verbose)
        std::cout << "Aruco marker detection ... " << endl;
    
    vector<int> ids;
    vector<vector<cv::Point2f> > markers;
    cv::aruco::detectMarkers(img, aruco_dict, markers, ids);
    
    //https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html
    //cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    
    if(drawMarkers) cv::aruco::drawDetectedMarkers(img, markers, ids);
    
    if (debug_verbose)
        std::cout << "Number of marker detected: " << ids.size() << std::endl;

    bool detected = (int)ids.size() > num_markers_threshold;
    if (detected) // get the marker corner (x, y) closest to the center of the ChaRuCo board
    {
        size_t index=-1;
        for(index = 0; index < ids.size(); index++)
            if (ids[index] == 21 || ids[index] == 26) // 21 or 26 are closest markers to the center of our board w/ 48 markers
                break;
        if (index >= ids.size()) // did not find center marker 21 or 26
        {
            centerMarkerCorner = cv::Point2f(-1.0, -1.0); // mark it as no marker 21 or 26 was found
            detected = false;
        }
        else if (index == 21) // use the 3rd marker corner of id 21
        {
            centerMarkerCorner = markers[index][2];
            if (debug_verbose) cout << "Marker 21" << endl;
        }
        else
        {
            centerMarkerCorner = markers[index][0];
            if (debug_verbose) cout << "Marker 26" << endl;
        }
    }
    return detected;
}

//#######################################################################################
bool PoseEstimator::get_pose(const vector<vector<cv::Point2f> > &markers, const vector<int> &ids, 
                             const vector<cv::Point2f> &chCorners,  const vector<int> &chIds,
                             cv::Mat *extrinsics_4x4, double *reprojection_error)
{
    /*
        PYTHON CODE:
        def get_pose(markers,ids,chorners,chids,board,intrinsics,dist,debug_verbose):
        extrinsics_4x4 = []
        retval, rvec, tvec = solvePNP_Charuco_Board(markers,ids,board,intrinsics,dist)
        if rvec is None:
            return extrinsics_4x4

        rvec, _ = cv2.Rodrigues(rvec)
        stacked = np.hstack(([rvec,tvec]))
        extrinsics_4x4 = np.vstack((stacked,[0,0,0,1]))

        if(debug_verbose == True):
            ...

        mean_error = charuco_reprojection_error(board,markers,ids,rvec,tvec,intrinsics,dist)

        return extrinsics_4x4, mean_error
    */

    // The python code is wrong: it's not using the interpolated chessboard corners! 
    // We do it all in here w/o calling solvePNP_Charuco_Board or reprojection error function - yc
    cv::Mat rvec, tvec;
    //~ if (!solvePNP_Charuco_Board(markers, ids, &rvec, &tvec)) {
        //~ return false;
    //~ }
    //~ bool success = true;
    
    // Use interpolated chessboard corners to estimate the pose, which are much more accurate
    bool success = cv::aruco::estimatePoseCharucoBoard(chCorners, chIds, board, intrinsics, distCoeffs, rvec, tvec);

    if(debug_verbose) dumpRvecTvec(rvec, tvec);

    // keep a copy of rvec & tvec for later use
    rvec_tmp = rvec;
    tvec_tmp = tvec;

    if (success)
    {

        cv::Mat rot3x3;
        cv::Rodrigues(rvec, rot3x3);   // Convert the rotation vector into rotation matrix via Rodrigues algorithm

        // Build extrinsics_4x4 by stacking the rvec and tvec and adding the appropriate padding
        cv::Mat extrinsics4x4 = (cv::Mat_<double>(4, 4) <<  rot3x3.at<double>(0, 0),    rot3x3.at<double>(0, 1),    rot3x3.at<double>(0, 2),    tvec.at<double>(0, 0),
                                                            rot3x3.at<double>(1, 0),    rot3x3.at<double>(1, 1),    rot3x3.at<double>(1, 2),    tvec.at<double>(1, 0),
                                                            rot3x3.at<double>(2, 0),    rot3x3.at<double>(2, 1),    rot3x3.at<double>(2, 2),    tvec.at<double>(2, 0),
                                                            0.0,                        0.0,                        0.0,                        1.0);

        // Explicitly copy the values from the local matrix to the return pointer matrix 'extrinsics_4x4'
        extrinsics_4x4->create(cv::Size(4, 4), CV_64F);
        for(int row = 0; row < 4; ++row) {
            for(int col = 0; col < 4; ++col) {
                extrinsics_4x4->at<double>(row, col) = extrinsics4x4.at<double>(row, col);
            }
        }

        //*reprojection_error = 0.0; //charuco_reprojection_error(markers, ids, rvec, tvec); //marker corners are unreliable, not meaningful for reproj error
        // We use the chessboard corners. First get the object points of the corners from the board that corresponds to the detected chCorners
        if(true)
            {
            vector<cv::Point2f> imgpoints;
            vector<cv::Point3f> objpoints;
            for (size_t i=0; i<chCorners.size(); i++)
            {
                objpoints.push_back(board->chessboardCorners[chIds[i]]); // pick the ones we have detected corners for in chCorners
            }
            // now project the object points to the image to get the imgpoints
            cv::projectPoints(objpoints, rvec, tvec, intrinsics, distCoeffs, imgpoints);
            // Compute the RMS of the image points of the corners and reprojected points under the rvec & tvec
            // The following equation comes from OpenCV 4.1.2, modules/calib3d/src/solvepnp.cpp
            *reprojection_error = cv::norm(chCorners, imgpoints, cv::NORM_L2) / sqrt(2*chCorners.size());
            
            /* The above result differs from the one from the following by a factor of 1/sqrt(2)
            double sum=0.0;
            for(size_t i = 0; i < chCorners.size(); i++)
            {
                double dx = (chCorners[i].x - imgpoints[i].x);
                double dy = (chCorners[i].y - imgpoints[i].y);
                sum += dx*dx + dy*dy;
            }
            *reprojection_error = sqrt(sum/chCorners.size());
            */
        }
    }
    return success;
}
//#######################################################################################
// deprecated; do not use!
bool PoseEstimator::solvePNP_Charuco_Board(const vector<vector<cv::Point2f> > &markers, const vector<int> &ids, 
                                            cv::Mat *rvec, cv::Mat *tvec)
{
    /*
    PYTHON CODE:
        objpoints, imgpoints = cv2.aruco.getBoardObjectAndImagePoints(board, markers, ids)
        flags = cv2.SOLVEPNP_ITERATIVE
        imgpoints = imgpoints.astype(np.float64)

        retval, rvec, tvec = cv2.solvePnP(objpoints, imgpoints, intrinsics, dist, flags = flags, useExtrinsicGuess = False)

        return retval, rvec, tvec
    */

    vector<cv::Point2d> imgpoints;
    vector<cv::Point3d> objpoints;
    cv::aruco::getBoardObjectAndImagePoints(board, markers, ids, objpoints, imgpoints);
    if (!cv::solvePnP(objpoints, imgpoints, intrinsics, distCoeffs, *rvec, *tvec, false, cv::SOLVEPNP_ITERATIVE))
    {
        cout << "ERROR! Problem with cv::solvePnP" << endl;
        return false;
    }
    return true;
}

//#######################################################################################
double PoseEstimator::charuco_reprojection_error(const vector<vector<cv::Point2f> > &corners, const vector<int> &ids, const cv::Mat &rvec, const cv::Mat &tvec) const
{
    /*  
        PYTHON CODE:
        def charuco_reprojection_error(board,corners,ids,rvec,tvec,intrinsics,dist,image = None, verbose = False):

            objpoints , imgpoints = cv2.aruco.getBoardObjectAndImagePoints(board,corners,ids)
            rp_l, _ = cv2.projectPoints(objpoints, rvec, tvec,intrinsics,dist)
            mean_error = np.square(np.mean(np.square(np.float64(imgpoints - rp_l))))

            return mean_error
    */

    vector<int> objpoints, imgpoints;
    cv::aruco::getBoardObjectAndImagePoints(board, corners, ids, objpoints, imgpoints);

    vector<float> rp_l;
    cv::projectPoints(objpoints, rvec, tvec, intrinsics, distCoeffs, imgpoints, rp_l);

    double mean_error = 0.0;   // figure this out later

    return mean_error;
}


cv::Mat yaw(double radians)
{
    // Returns an augmented rotation matrix (4x4) representing a rotation about the y-axis
    cv::Mat rot = cv::Mat::eye(4, 4, CV_64F);
    rot.at<double>(0, 0) = cos(radians);    rot.at<double>(0, 1) = 0.0;             rot.at<double>(0, 2) = -sin(radians);       rot.at<double>(0, 3) = 0.0;
    rot.at<double>(1, 0) = 0.0;             rot.at<double>(1, 1) = 1.0;             rot.at<double>(1, 2) = 0.0;                 rot.at<double>(1, 3) = 0.0;
    rot.at<double>(2, 0) = sin(radians);    rot.at<double>(2, 1) = 0.0;             rot.at<double>(2, 2) = cos(radians);        rot.at<double>(2, 3) = 0.0;
    rot.at<double>(3, 0) = 0.0;             rot.at<double>(3, 1) = 0.0;             rot.at<double>(3, 2) = 0.0;                 rot.at<double>(3, 3) = 1.0;

    return rot;
}

cv::Mat pitch(double radians)
{
    // Returns an augmented rotation matrix (4x4) representing a rotation about the x-axis
    cv::Mat rot = cv::Mat::eye(4, 4, CV_64F);
    rot.at<double>(0, 0) = 1.0;     rot.at<double>(0, 1) = 0.0;             rot.at<double>(0, 2) = 0.0;             rot.at<double>(0, 3) = 0.0;
    rot.at<double>(1, 0) = 0.0;     rot.at<double>(1, 1) = cos(radians);    rot.at<double>(1, 2) = -sin(radians);   rot.at<double>(1, 3) = 0.0;
    rot.at<double>(2, 0) = 0.0;     rot.at<double>(2, 1) = sin(radians);    rot.at<double>(2, 2) = cos(radians);    rot.at<double>(2, 3) = 0.0;
    rot.at<double>(3, 0) = 0.0;     rot.at<double>(3, 1) = 0.0;             rot.at<double>(3, 2) = 0.0;             rot.at<double>(3, 3) = 1.0;

    return rot;
}

cv::Mat roll(double radians)
{
    // Returns an augmented rotation matrix (4x4) representing a rotation about the z-axis
    cv::Mat rot = cv::Mat::eye(4, 4, CV_64F);
    rot.at<double>(0, 0) = cos(radians);    rot.at<double>(0, 1) = -sin(radians);   rot.at<double>(0, 2) = 0.0;             rot.at<double>(0, 3) = 0.0;
    rot.at<double>(1, 0) = sin(radians);    rot.at<double>(1, 1) = cos(radians);    rot.at<double>(1, 2) = 0.0;             rot.at<double>(1, 3) = 0.0;
    rot.at<double>(2, 0) = 0.0;             rot.at<double>(2, 1) = 0.0;             rot.at<double>(2, 2) = 1.0;             rot.at<double>(2, 3) = 0.0;
    rot.at<double>(3, 0) = 0.0;             rot.at<double>(3, 1) = 0.0;             rot.at<double>(3, 2) = 0.0;             rot.at<double>(3, 3) = 1.0;

    return rot;
}

void PoseEstimator::doRotation(double rx, double ry, double rz, cv::Mat *result)
{
    // Returns a 4x4 matrix representing a rotation in 3d. rx, ry, and rz correspond to pitch, yaw, and roll, 
    // respectively, and must be in radians.

    cv::Mat mxRoll = roll(rz);
    cv::Mat mxYaw = yaw(ry);
    cv::Mat mxPitch = pitch(rx);
    cv::Mat r = mxRoll * mxYaw * mxPitch;

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            result->at<double>(row, col) = r.at<double>(row, col);
        }
    }

    // TODO: We should just hard-code the trig for each element in the resulting matrix. 
    //       This method still does a bunch of unnecessary memory allocation, but it is simple and works for now.
}

void PoseEstimator::doTranslation(double tx, double ty, double tz, cv::Mat *result)
{
    // Returns a 4x4 matrix representing a translation in 3d.
    result->at<double>(0, 0) = 1.0;     result->at<double>(0, 1) = 0.0;     result->at<double>(0, 2) = 0.0;     result->at<double>(0, 3) = tx;
    result->at<double>(1, 0) = 0.0;     result->at<double>(1, 1) = 1.0;     result->at<double>(1, 2) = 0.0;     result->at<double>(1, 3) = ty;
    result->at<double>(2, 0) = 0.0;     result->at<double>(2, 1) = 0.0;     result->at<double>(2, 2) = 1.0;     result->at<double>(2, 3) = tz;
    result->at<double>(3, 0) = 0.0;     result->at<double>(3, 1) = 0.0;     result->at<double>(3, 2) = 0.0;     result->at<double>(3, 3) = 1.0;
}
