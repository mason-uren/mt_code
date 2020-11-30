// PoseEstimator.cpp : Source file for determining the 6DOF pose of a fiducial in a single frame of video
// Recommended use:
//   - call detect_aruco_markers() first
//   - call estimate_pose_charucoboard() to get the 6-DOF pose in world coordinates frame (via dynamic extrinsics)
//   - call get_local_pose_charucoboard() to retrieve rvec and tvec in camera coordinate frame

#include "PoseEstimator.h"

using namespace std;

PoseEstimatorWorld::PoseEstimatorWorld(const std::string & cadModelJsonPathname, const CameraModel<double> & cameraModel, const FiducialModel & fiducialModel,
                                       bool debug, double markerDetectScale, double inputImageScaleFactor,
                                       const std::string & caller) :
        PoseEstimator(cameraModel, fiducialModel, debug, markerDetectScale, inputImageScaleFactor, caller)
{
    if(! load_cad_model(cadModelJsonPathname))
    {
        this->eHandler->report("Failed in initializing PoseEstimatorWorld!", Shared::Error::WARN, caller);
        std::cerr << "Failed in initializing PoseEstimatorWorld!" << std::endl;
        throw;
    }
}

//#######################################################################################
// PoseEstimatorWorld constructor - stick to the old "PoseEstimator" style for now
//#######################################################################################
PoseEstimatorWorld::PoseEstimatorWorld(const string & cadModelJsonPathname, const string & cameraMatrixPath, const string & distCoeffsPath, 
                                       bool altCharucoBoard, bool debug, double markerDetectScale, double inputImageScaleFactor, const std::string & caller) :
        PoseEstimatorWorld(cadModelJsonPathname,
            CameraModel<double>{cameraMatrixPath, distCoeffsPath, markerDetectScale, caller},
            FiducialModel{},
            debug, markerDetectScale, inputImageScaleFactor, caller
        )
{}
//    PoseEstimator(cameraMatrixPath, distCoeffsPath, altCharucoBoard, debug, markerDetectScale, inputImageScaleFactor, caller)
//{
//
//    if(! load_cad_model(cadModelJsonPathname))
//    {
//        this->eHandler->report("Failed in initializing PoseEstimatorWorld!", Shared::Error::WARN, caller);
//        std::cerr << "Failed in initializing PoseEstimatorWorld!" << std::endl;
//        throw;
//    }
//}
//#######################################################################################
bool PoseEstimatorWorld::load_cad_model(const string &  modelFilename)
{
    this->eHandler->report("Loading JSON PT cad model from: "+ modelFilename, Shared::Error::INFO, this->getName()); 
    if(debug_verbose)
    {
        cout <<"Loading JSON PT cad model from: "<< modelFilename << endl;
    }
    try 
    {
        pt_model.loadModel(modelFilename);
        if (debug_verbose)
        {
            cout << "Contents of Loaded PT cad model: " << endl;
            pt_model.displayModel();
        }
    }
    catch(exception & e)
    {
        strbuf.clear();
        strbuf<<"Error while loading PT cad model: "<< e.what() ;
        std::cout << strbuf.str() << std::endl;
        this->eHandler->report(strbuf.str(), Shared::Error::WARN, this->getName()); 
        return false;
    }
    return true;
}

PoseEstimator::PoseEstimator(const CameraModel<double> & cameraModel, const FiducialModel & fiducialModel,
							 const bool debug, const double & markerDetectionScale, const double & inputImageScaleFactor,
							 const std::string & caller) :
	ConfigInterface(caller),
	cameraModel(cameraModel),
	fiducialModel(fiducialModel),
	debug_verbose(debug), 
	markerDetectionScale(markerDetectionScale),
    imageWidth(-1), imageHeight(-1),
    reprojErrorThreshold(1.5),	scaleFactor(inputImageScaleFactor),
	scaledImgMkrDetectionSuccessful(false),
	poseEstimationSuccessful(false)
{
	setInstananeousFOV(1.0);
	determine_marker_detection_parameters();
}

//#######################################################################################
// Basic constructor for use with the default 6"x4" fiducial for tracking
PoseEstimator::PoseEstimator(const string & cameraMatrixPath, const string & distCoeffsPath,  
                            bool altCharucoBoard, bool debug, double markerDetectScale, double inputImageScaleFactor,
                            const std::string & caller) :
	ConfigInterface(caller),
    debug_verbose(debug), scaleFactor(inputImageScaleFactor), imageWidth(-1), imageHeight(-1), boardCenter(),
    markerDetectionScale(markerDetectScale), scaledImgMkrDetectionSuccessful(false),
    poseEstimationSuccessful(false), reprojErrorThreshold(1.5),
	cameraModel(CameraModel<double>{cameraMatrixPath, distCoeffsPath, inputImageScaleFactor, caller})
{
    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

    if (altCharucoBoard)
    {
		fiducialModel.setup(Model::Board{ 40, 20, 0.0235F, 0.01665F }, cv::aruco::DICT_4X4_1000); // "TV3" board
        this->eHandler->report("PoseEstimator: Alternative ChArUco board 40x20 (TV3) is initialized", Shared::Error::INFO, caller);
        if(debug_verbose)
        {
            std::cout << "PoseEstimator: Alternative ChArUco board 40x20 (TV3) is initialized" << std::endl;
        }
    }
	else
	{
		fiducialModel.setup(Model::Board{ 8, 12, 0.012F, 0.009F }, cv::aruco::DICT_4X4_1000); // "Fiducial" board, 4" x 6", reprinted 2020-07
		this->eHandler->report("PoseEstimator: Standard ChArUco board 8x12 (Fiducial) is initialized", Shared::Error::INFO, caller);
		if (debug_verbose)
		{
			std::cout << "PoseEstimator: Standard ChArUco board 8x12 (Fiducial) is initialized" << std::endl;
		}
	}

	setInstananeousFOV(1.0);  // 1.3 is a fudge factor because the intrinsics are not correct

	//// OLD
 //   if(! load_camera_intrinsics(cameraMatrixPath, distCoeffsPath))
 //   {
 //       this->eHandler->report("Failed in initializing PoseEstimator!", Shared::Error::WARN, this->name);
 //       std::cerr << "Failed in initializing PoseEstimator!" << std::endl;
 //       throw;
 //   }
    determine_marker_detection_parameters();

}


//#######################################################################################
// Second version allows for custom ChArUco board provided by the caller
PoseEstimator::PoseEstimator(const string & cameraMatrixPath, const string & distCoeffsPath,  
                            const cv::Ptr<cv::aruco::CharucoBoard> charucoBoard,
                            double markerDetectScale, bool debug, double inputImageScaleFactor,
                             const string & caller) :
	ConfigInterface(caller),
    debug_verbose(debug),  scaleFactor(inputImageScaleFactor), imageWidth(-1), imageHeight(-1),
	fiducialModel(FiducialModel{charucoBoard}),
	boardCenter(), markerDetectionScale(markerDetectScale),
    scaledImgMkrDetectionSuccessful(false), poseEstimationSuccessful(false),
    reprojErrorThreshold(1.5), cameraModel(CameraModel<double>{cameraMatrixPath, distCoeffsPath, inputImageScaleFactor, caller})
{
    /*if(! load_camera_intrinsics(cameraMatrixPath, distCoeffsPath))
    {
        this->eHandler->report("Failed in initializing PoseEstimator!", Shared::Error::WARN, this->name);
        std::cerr << "Failed in initializing PoseEstimator!" << std::endl;
        throw;
    }*/

	setInstananeousFOV(1.0);  // 1.3 is a fudge factor because the intrinsics are not correct
    determine_marker_detection_parameters();

}

//#######################################################################################
// The 3rd version is like the 2st version, but allows the caller to provide camera matrix and distCoeffs directly
PoseEstimator::PoseEstimator(const cv::Mat & cameraMatrix, const cv::Mat & distCoeffsVec,
                             const cv::Ptr<cv::aruco::CharucoBoard> charucoBoard,
                             double markerDetectScale, bool debug, double inputImageScaleFactor,
                             const string & caller) :
		ConfigInterface(caller),
        debug_verbose(debug), 
		//cameraMatrix(cameraMatrix), distCoeffs(distCoeffsVec),
		cameraModel(CameraModel<double>{cameraMatrix, distCoeffsVec, inputImageScaleFactor, caller}),
        scaleFactor(inputImageScaleFactor), imageWidth(-1), imageHeight(-1), 
		fiducialModel(FiducialModel{ charucoBoard }),
        boardCenter(), markerDetectionScale(markerDetectScale), scaledImgMkrDetectionSuccessful(false),
        poseEstimationSuccessful(false), reprojErrorThreshold(1.5)
{
	setInstananeousFOV(1.0);  // 1.3 is a fudge factor because the intrinsics are not correct
    determine_marker_detection_parameters();
}

//#######################################################################################
void PoseEstimator::determine_marker_detection_parameters()
{
    // Determine the marker id surrounding the center chessboard corner: centerMarkerId1, centerMarkerId2.
    // for 8 x 12 fiducial, these are originally manually picked as 21 and 26, but will change if the ChAruCo
    // board changes. For example, the following center markers can be identified:
    // Board        centerMarkerId1      centerMarkerId2
    // 8 x 12             21                   26
    // 12 x 8             20                   27
    // 40 x 20           189                  210

    /* now we use the brute-force alternative "algorithm" below.

    // The following code is not fool-proof, but will produce the above marker id's as intended
    // and I think this "algorithm" works when the sizes of the board are even.
    cv::Size boardSize = board->getChessboardSize();
    //std::cout << "boardSize=" << boardSize << std::endl;
    int num_markers_row = boardSize.width / 2;
    int num_markers_col = boardSize.height / 2;
    int total_markers = boardSize.width * num_markers_col;
    int centerMarkerId1 = total_markers / 2 + num_markers_row / 2;
    int centerMarkerId2 = centerMarkerId1 - num_markers_row - 1;

    // Extend the center marker approach to more candidate markers around the center of the charuco board for robustness of board detection
    centerMarkerIds.push_back(centerMarkerId1);
    centerMarkerIds.push_back(centerMarkerId2);
    // Since I don't have time to come up with an "algorithm" that works for all boards, we will hardcode for
    // the default 8 x 12 and 40 x 20 boards. For other user specified boards, we will rely on the center markers
    // identified above for detection.
    if (boardSize.width == 8 && boardSize.height == 12)
    {
        centerMarkerIds.push_back(18);
        centerMarkerIds.push_back(29);
        centerMarkerIds.push_back(22);
        centerMarkerIds.push_back(25);
    }
    else if (boardSize.width == 12 && boardSize.height == 8)
    {
        centerMarkerIds.push_back(21);
        centerMarkerIds.push_back(26);
        centerMarkerIds.push_back(15);
        centerMarkerIds.push_back(32);
    }
    else if (boardSize.width == 40 && boardSize.height == 20)
    {
        //centerMarkerIds.push_back(18);
        //centerMarkerIds.push_back(18);
        //centerMarkerIds.push_back(18);
        //centerMarkerIds.push_back(18);
    }
    */

	auto board{this->fiducialModel.getBoard()};
    cv::Size boardSize = board->getChessboardSize();

    // Brute-force but effective alterative:
    // 1. Find the center of the board in physical dimensions:
    boardCenter = cv::Point3f(boardSize.width, boardSize.height, 0.0f) * board->getSquareLength() / 2.0f;

    // 2. loop over all chessboard corners, and find the one closes to the board center, keep is as centerCorner
    float minDist = (boardSize.width + boardSize.height) *board->getSquareLength(); // a "large" positive number
    centerCornerId = -1;
    for(size_t id=0; id<board->chessboardCorners.size(); id++)
    {
        float d = cv::norm((boardCenter - board->chessboardCorners[id]));
        if( minDist > d)
        {
            minDist = d;   centerCornerId = id;
        }
    }
    centerCorner=board->chessboardCorners[centerCornerId];

    strbuf.str(""); strbuf.clear();
    strbuf << "PoseEstimator: found center chessboard corner, id = "<< centerCornerId;
    this->eHandler->report(strbuf.str(), Shared::Error::DEBUG, this->getName()); 
    if (debug_verbose)
    {
        std::cout << strbuf.str() << std::endl;
    }

    // 3. Loop over all markers and calculate their distances to centerCorner, sort marker ids based on distance,
    //   and keep the id's of the 6 (or 8) with shortest distances ...
    std::vector<float> markerDist;
    for(size_t id = 0; id < board->ids.size(); id++)
    {
        // get the 4 corners of marker at id
        const std::vector<cv::Point3f> & mkCorners = board->objPoints[id];
        // find the avg of 4 marker corners to get the center of the marker
        cv::Point3f markerCenter(0.0f, 0.0f, 0.0f);
        for(auto pt : mkCorners)
        {  markerCenter += pt;  }
        markerCenter = markerCenter / (float)mkCorners.size();
        float d = cv::norm(markerCenter-centerCorner);
        markerDist.push_back(d);
    }
    // Make a copy of board->ids, then sort ids based on markerDist:
    std::vector<int> markerIds = board->ids;
    std::stable_sort(markerIds.begin(), markerIds.end(),
                [&markerDist](int i1, int i2) {return markerDist[i1] < markerDist[i2];});
    // Now copy the first 6 markerIds to centerMarkerIds:
    centerMarkerIds.clear();
    for(size_t i = 0; i < 6; i++)
        centerMarkerIds.push_back(markerIds[i]);

    std::stringstream markerIdStr{};
    for(size_t i=0; i < centerMarkerIds.size(); i++)
    {    markerIdStr << centerMarkerIds[i] << ", "; }
    this->eHandler->report("PoseEstimator: Selected center markers (id): [" + markerIdStr.str() + "]", Shared::Error::DEBUG, this->getName());
    if(debug_verbose)
    {
        std::cout << "PoseEstimator: Selected center markers (id): \n"
                        << "[" << markerIdStr.str() << "]" << std::endl;
    }

    //Initialize marker detection threshold based total number of markers in the board: 50% of markers in the board
    minNumMarkersThreshold = board->ids.size() / 2;
    strbuf.str(""); strbuf.clear();
    strbuf << "PoseEstimator: minNumMarkersThreshold = " << minNumMarkersThreshold;
    this->eHandler->report(strbuf.str(), Shared::Error::DEBUG, this->getName()); 
    if(debug_verbose)
       {
       std::cout << strbuf.str() << std::endl;
    }

    // Here come the real parameters for marker detection. The default values aren't ideal for our environment due to
    // image size/resolution (relative to the the marker size), and image brightness.
    markerDetectParameters = cv::aruco::DetectorParameters::create();
    // The following parameters seem to work better for dimmer/darker & slightly out-of-focus small(4inx6in) charuco board
    markerDetectParameters->adaptiveThreshWinSizeMax = 28; // default is 23
    //parameters->adaptiveThreshWinSizeMin = 3;//default
    markerDetectParameters->adaptiveThreshWinSizeStep = 6; //default is 10
    markerDetectParameters->minMarkerPerimeterRate = 0.01; // 8x12 board markers has a perimeter rate of ~0.04 (Ximea) & 0.024 (Imperx) (default 0.03)
    markerDetectParameters->maxMarkerPerimeterRate = 0.5; //default = 4.0
    markerDetectParameters->adaptiveThreshConstant = 7; //default 7, lower it to get more detections
    markerDetectParameters->polygonalApproxAccuracyRate = 0.09; // default 0.03, raise it to get more detections

    // If larger than the threshold, declare failure in pose estimation (estimate_pose_charuco())
    //reprojErrorThreshold = 1.5; // moved contructors
    // this threshold is somewhat arbitrary, the purpose is to ensure that only reliable pose estimation is returned.
    // Anecdotally, Ximea image reprojection errors are usually < 1.0 (0.2~0.5), while Imperx < 0.1 (0.07~0.1)
    // for the 4 in x 6 in charuco board fiducial at 5 meters.
}

void PoseEstimator::set_board_detection_threshold(int min_markers)
{
    minNumMarkersThreshold = min_markers;
    strbuf.clear();
	strbuf.str("");
    strbuf << "PoseEstimator: overriding default minNumMarkersThreshold = "
                 << minNumMarkersThreshold ;
    this->eHandler->report(strbuf.str(), Shared::Error::DEBUG, this->getName());
    if(debug_verbose)
    {
       std::cout << strbuf.str() << std::endl;
    }

}

void PoseEstimator::setInstananeousFOV(const double & fudgeFactor) {
	double focal_length = get_focal_length() * fudgeFactor; // 1.3 is a fudge factor because the intrinsics are not correct

	// Instantaneous field-of-view, the FOV of each pixel (at the ceter of the image)
	this->ifov = 1.0 / focal_length / M_PI * 180.0; //in degrees
}


//#######################################################################################
//bool PoseEstimator::load_camera_intrinsics(const string & cameraMatrixPath, const string & distCoeffsPath)
//{
//    // Load the intrinsics & distCoeffs from .npy files, a 3x3 and a 1x5 matrix respectively
//    vector<int>shape;
//    vector<double> data;
//    //load camera matrix
//    try
//    {
//        strbuf.str("");strbuf.clear();
//        strbuf  << "Loading camera matrix from: " << cameraMatrixPath;
//        this->eHandler->report(strbuf.str(), Shared::Error::INFO, this->name);
//        if (debug_verbose)
//        {
//            std::cout<< strbuf.str() << std::endl;
//        }
//        aoba::LoadArrayFromNumpy(cameraMatrixPath, shape, data);
//    }
//    catch(exception & e)
//    {
//        strbuf.str(""); strbuf.clear();
//        strbuf  <<"Error while trying to load camera intrinsics: "<< e.what() ;
//        std::cout << strbuf.str() << std::endl;
//        this->eHandler->report(strbuf.str(), Shared::Error::WARN, this->name);
//        return false;
//    }
//    if (shape[0]!=shape[1] || shape[0] !=3)
//    {
//        std::cerr << "Invalid intrinsics matrix dimensions, expecting 3 x 3." << std::endl;
//        this->eHandler->report("Invalid intrinsics matrix dimensions, expecting 3 x 3.", Shared::Error::WARN, this->name);
//        return false;
//    }
//    //std::cout << "Ximea intrinsics: " << shape[0] << ", "<< shape[1] << std::endl;
//
//	
//    cameraMatrix = cv::Mat(shape, CV_64F, data.data()).clone();
//    if (scaleFactor != 1.0)
//    {
//        cameraMatrix.at<double>(0, 0) /= scaleFactor; // focal length in X
//        cameraMatrix.at<double>(1, 1) /= scaleFactor; // focal length in Y
//        cameraMatrix.at<double>(0, 2) /= scaleFactor; // principle point in X
//        cameraMatrix.at<double>(1, 2) /= scaleFactor; // principle point in Y
//    }
//    strbuf.str(""); strbuf.clear();
//    strbuf << cameraMatrix;
//    this->eHandler->report(strbuf.str(), Shared::Error::DEBUG, this->name);
//    if(debug_verbose)
//    {
//        std::cout << strbuf.str() << std::endl;
//    }
//    // Load distCoeffs
//    try
//    {
//        if (debug_verbose)
//        {
//            cout << "Loading camera distCoeffs from: " << distCoeffsPath << endl;
//        }
//        this->eHandler->report("Loading camera distCoeffs from: " + distCoeffsPath, Shared::Error::INFO, this->name);
//        aoba::LoadArrayFromNumpy(distCoeffsPath, shape, data); // there is only 1 row of data (a vector)
//    }
//    catch(exception & e)
//    {
//        strbuf.str(""); strbuf.clear();
//        strbuf  <<"Error while trying to load camera distCoeffs: "<< e.what() ;
//        std::cout << strbuf.str() << std::endl;
//        this->eHandler->report(strbuf.str(), Shared::Error::WARN, this->name);
//        return false;
//    }
//    if (shape[0] !=1 || shape[1] != 5)
//    {
//        std::cerr  << "Invalid distCoeffs matrix dimensions, expecting 1 x 5."<< std::endl;
//        this->eHandler->report("Invalid distCoeffs matrix dimensions, expecting 1 x 5.", Shared::Error::WARN, this->name);
//        return false;
//    }
//
//    //std::cout << "Ximea distCoeffs: " << shape[0] << ", "<< shape[1] << std::endl;
//    distCoeffs = cv::Mat(shape, CV_64F, data.data()).clone();
//
//    strbuf.str(""); strbuf.clear();
//    strbuf << distCoeffs ;
//    this->eHandler->report(strbuf.str(), Shared::Error::DEBUG, this->name);
//    if(debug_verbose)
//    {
//        std::cout << strbuf.str() << std::endl;
//    }
//
//    double focal_length = get_focal_length()*1.0; // 1.3 is a fudge factor because the intrinsics are not correct
//    
//    // Instantaneous field-of-view, the FOV of each pixel (at the ceter of the image)
//    ifov = 1.0/focal_length/M_PI*180.0; //in degrees
//
//    return true;
//}

//#######################################################################################
// Return average focal length
double PoseEstimator::get_focal_length()
{
	auto cameraMatrix{this->cameraModel.cvMatrix.cameraMatrix};
    return (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2.0;
}

//#######################################################################################
// Call this function to get rvec and tvec after estimate_pose_charucoboard() and returns "true".
bool PoseEstimator::get_rvec_tvec_charucoboard(cv::Mat & rvec_out, cv::Mat & tvec_out)
{
    if(poseEstimationSuccessful)
    {
       rvec_out = rvec;  tvec_out = tvec;
    }
    return poseEstimationSuccessful;
}

//#######################################################################################
// Call this function to get extrinsics 4x4 matrix after estimate_pose_charucoboard() returns "true".
bool PoseEstimator::get_extrinsics4x4_charucoboard(cv::Mat & extrinsics4x4)
{
    if(poseEstimationSuccessful)
    {
       extrinsics4x4 = compose_extrinsics(rvec, tvec);
    }
    return poseEstimationSuccessful;
}

//#######################################################################################
// Call this function to get chessboard corners & ids after estimate_pose_charucoboard() returns "true".
bool PoseEstimator::get_corner_uv_id_charucoboard(vector<cv::Point2f> & corners, vector<int> & ids)
{
    if(poseEstimationSuccessful)
    {
       corners = chCorners;  ids = chIds;
    }
    return poseEstimationSuccessful;
}

// Call this function  to get the image coords. of the center chessboard corner after
// estimate_pose_charucoboard() returns "true".
cv::Point2f PoseEstimator::get_center_corner_uv_charucoboard()
{
    cv::Point2f c(-1.0f, -1.0f);
    if(poseEstimationSuccessful)
    {
		auto cameraMatrix{ this->cameraModel.cvMatrix.cameraMatrix };
		auto distCoeffs{this->cameraModel.cvMatrix.distortionCoeffs};
        // Was the center corner detected? If so, return it.
        std::vector<int>::iterator ci = find(chIds.begin(), chIds.end(), centerCornerId);
        if(ci!=chIds.end())
        {  c = chCorners[ci-chIds.begin()]; }
        else
        // if not detected, project the 3d center corner point to (u,v) and return it
        {
            std::vector<cv::Point2f> imgpoints{};
            std::vector<cv::Point3f> objpoints{centerCorner};
            cv::projectPoints(objpoints, rvec, tvec, cameraMatrix, distCoeffs, imgpoints);
            c = imgpoints[0];
        }
    }
    return c;
}
//#######################################################################################
// Return estimated board pose in world coordinates in "result", for the given pan & tilt 
// of the dynamic extrinsics model
bool PoseEstimatorWorld::estimate_pose_charucoboard(const cv::Mat &frame, double pan, double tilt, SixDOF *result)
{
    // For a given image and pan-tilt pair, compute the 6 DOF of a fiducial in the image and return it.

    int charuco_sq_found = -1;
    double repro_err = 0.0;
    cv::Mat ximea_extrinsics, combined_extrinsics;
    if(PoseEstimator::estimate_pose_charucoboard(frame, &repro_err))
    {
        // The function 'estimate_pose_charucoboard' returns 'true' if a board was detected
        // output are placed in members rvec & tvec, and repro_err
        
        //ximea_extrinsics = compose_extrinsics(rvec, tvec);
        this->get_extrinsics4x4_charucoboard(ximea_extrinsics);
        
        strbuf.str(""); strbuf.clear();;
        strbuf << "pan, tilt = " << pan << ", " << tilt << " (" << deg2rad(pan) << ", " << deg2rad(tilt) << ")" << std::endl;
        strbuf << "Fiducial pose (extrinsics, camera frame) = " << std::endl;
        strbuf << ximea_extrinsics ;
        this->eHandler->report(strbuf.str(), Shared::Error::DEBUG, this->getName()); 
        if (debug_verbose)
        {
            //std::cout << "Warning: using PanTiltModel.applyInverse() ..." << std::endl;
            std::cout << strbuf.str() << std::endl;
        }
        combined_extrinsics = ximea_extrinsics;
        pt_model.applyInverse(combined_extrinsics, deg2rad(pan), deg2rad(tilt));

        strbuf.str(""); strbuf.clear();;
        strbuf  << "Fiducial pose (extrinsics, world frame) = " << std::endl;
        strbuf << combined_extrinsics ;
        this->eHandler->report(strbuf.str(), Shared::Error::DEBUG, this->getName()); 
        if (debug_verbose)
        {
            std::cout<< strbuf.str()  << std::endl;
        }

        // Populate the 6 DOF object with the results from rvec and tvec
        if (result != nullptr)
        {
            result->set_pose(combined_extrinsics);                  // 4x4 to sixDOF
            result->pan = pan;                                      // provided pan value of the PT unit
            result->tilt = tilt;                                    // provided tilt value of the PT unit

            result->num_aruco_markers_found = static_cast<int>(mkIds.size());
            result->num_interpolated_corners_found = static_cast<int>(chCorners.size());
            result->reprojection_error = repro_err;
        }
    }
    else
    {
        // If a ChArUco board is not detected (i.e., 'estimate_pose_charucoboard_ximea' returns 'false'), then there isn't any point 
        // in going through the other processing steps. Just return quickly without doing any processing and grab the next frame.
        result->num_aruco_markers_found = charuco_sq_found;
        return false;
    }

    return true;
}

// Return estimated pan & tilt in degrees in ptInOut; on input, ptInOut holds initial guess of the pan * tilt
bool PoseEstimatorWorld::estimate_needed_pantilt(const SixDOF & boardPose, cv::Vec2d & ptInOut) {

    const int max_iterations{100};
    auto sstream{ std::stringstream{} };

    auto ptRads{ cv::Mat{cv::Vec2d{deg2rad(ptInOut[0]), deg2rad(ptInOut[1])}} };

    sstream << "InitGuess PT ( " << ptInOut[0] << ", " << ptInOut[1] << ") Rad ( " << ptRads.at<double>(0) << ", " << ptRads.at<double>(1) << ")";
	this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->getName()); 
	sstream.str("");

    // p3d is the 3D coordinate of the charuco board center in world coordinate frame
    auto p3d = cv::Mat{ boardPose.extrinsics * (cv::Mat{cv::Vec4d(this->boardCenter.x, this->boardCenter.y, this->boardCenter.z, 1.0) } ) };

	// LMSolver
	auto cameraMatrix{ this->cameraModel.cvMatrix.cameraMatrix };
	auto solver{ cv::LMSolver::create(
            cv::makePtr<HandOffSolver>(p3d, this->pt_model, cameraMatrix, cv::Size{this->imageWidth, this->imageHeight }, this->getName()), max_iterations) }; //prev: this->loggerID, now: this->getName()
	auto iterations{solver->run(ptRads)};

    auto estPanTilt{ cv::Vec2d{rad2deg(ptRads.at<double>(0)), rad2deg(ptRads.at<double>(1))} };

    sstream << "Estimated PT: ( " << estPanTilt << ")" << ", Iterations: " << iterations ;
	this->eHandler->report(sstream.str(), Shared::Error::Severity::DEBUG, this->getName()); 
	sstream.str("");

    ptInOut = estPanTilt;

    return true;
}

// Dummy implementation, written for compatibility w/ PoseEstimatorWorld counter-part above.
bool PoseEstimator::estimate_needed_pantilt(const SixDOF &, cv::Vec2d & )
{
    return false;
}
//#######################################################################################
// Out-dated, no longer used as of rev171434
// Estimate the pan & tilt needed to move fiducial center to align with "goal"
cv::Point2f PoseEstimator::estimate_needed_pantilt(const cv::Point2f & goal, const cv::Point2f & fiducialCenter)
{
    if (debug_verbose)
    {
        std::cout << "Calc needed pan/tilt, current fiducial center coordinates: " << fiducialCenter << std::endl;
        std::cout << "Calc needed pan/tilt, goal coordinates:                    " << goal << std::endl;
    }

    cv::Point2f offCenterPixels = goal - fiducialCenter;
    if (debug_verbose) std::cout << "  Fiducial center is off the goal by: " << offCenterPixels << " (pixels)" << std::endl;
    if (debug_verbose) std::cout << "  Delta-pan/tilt estimated based on IFOV (deg) = " << ifov << std::endl;
    return offCenterPixels * ifov;
}

// Overloaded function to be compatible with PoseEstimatorWorld()
bool PoseEstimator::estimate_pose_charucoboard(const cv::Mat &frame, double, double, SixDOF *result)
{
    double reprojection_error;
    bool ret_val = estimate_pose_charucoboard(frame, &reprojection_error);
    if(ret_val && result!=nullptr)
    {
        cv::Mat extrinsics;
        this->get_extrinsics4x4_charucoboard(extrinsics); //convert member rvec & tvec to 4x4
        result->set_pose(extrinsics); // 4x4 to sixDOF
        // Filling in the remaining fields of sixDOF
        //result->pan = nan("");
        //result->tilt = nan("");
        result->reprojection_error = reprojection_error;
        result->num_aruco_markers_found = static_cast<int>(mkIds.size());
        result->num_interpolated_corners_found = static_cast<int>(chCorners.size());
    }
    return ret_val;
}
//#######################################################################################
// img:  is a full-res input image or whatever the "scaleFactor" (in init()) expects of the input image is
// estimates the pose of the charuco board in camera frame (local pose) and save rvec & tvec in member variable
// retrieve them with get_local_pose_charucoboard()
// reprojection_error is optional
bool PoseEstimator::estimate_pose_charucoboard(const cv::Mat &img, double *reprojection_error)
{
	auto board{this->fiducialModel.getBoard()};
	auto cameraMatrix{ this->cameraModel.cvMatrix.cameraMatrix };
	auto distCoeffs{ this->cameraModel.cvMatrix.distortionCoeffs };
    //  img = cv::blur(img, (3, 3));        // skipping this for now since we downsample

    if (imageWidth == -1 || imageHeight == -1)
    {
        //initialize on first use:
        imageWidth = img.cols;
        imageHeight = img.rows;
    }
    else {
        assert(imageWidth == img.cols && imageHeight == img.rows);
    }

    poseEstimationSuccessful = false;
    bool markerDetectionSuccessful = false;

    auto  start_time = chrono::high_resolution_clock::now();
    vector<vector<cv::Point2f> > markersFullRes;
    if(scaledImgMkrDetectionSuccessful) // already done in detect_aruco_markers()
    {
        if (debug_verbose)
            cout << "Estimating pose: re-using previous marker detection results ... " << endl;
        this->eHandler->report("Estimating pose: re-using previous marker detection results ... ", Shared::Error::Severity::DEBUG, this->getName()); 

        //vector<int> ids = mkIds; // this is a member now
        //if(charuco_sq_found != nullptr)
        //    *charuco_sq_found = (int)mkIds.size();

        // Instead of running marker detection on the full resolution image, we scale the markers detected at reduced resolution,
        // and use the scaled markers for corner interpolation to speed up the processing
        for( auto marker : markers )
        {
            double factor = 1.0/markerDetectionScale;
            
            markersFullRes.push_back(vector<cv::Point2f>());
            for (auto pt : marker) // pt is of cv::Point2f
            {
                markersFullRes.back().push_back(pt * factor);
            }
        }
        
    }
    else // Marker detection were not done previously from detect_aruco_markers(), or it has failed; we do it here on the full res input image
    {
        if (debug_verbose)
            cout << "Estimating pose: first perform marker detection ... " << endl;
        this->eHandler->report("Estimating pose: first perform marker detection ... ", Shared::Error::Severity::DEBUG, this->getName()); 

        //vector<int> ids = mkIds;
        cv::aruco::detectMarkers(img, board->dictionary, markersFullRes, mkIds);
        //if(charuco_sq_found != nullptr)
        //    *charuco_sq_found = (int)mkIds.size();
    }
    
    markerDetectionSuccessful = (! mkIds.empty()) &&  (mkIds.size() >= minNumMarkersThreshold);

    // Draw marker in the input image for debugging, probably should never do that
    //if(debug_verbose) cv::aruco::drawDetectedMarkers(img, markersFullRes, mkIds);
    
    double processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start_time).count();

    strbuf.str(""); strbuf.clear();;
    strbuf << "Estimating pose: number of Aruco markers detected: " << mkIds.size() << " (" << processing_time <<" ms)";
    this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->getName()); 
    if (debug_verbose)
    {
        cout << strbuf.str() << endl;
    }

        
    if (markerDetectionSuccessful)
    {
        // chorners and chids are returned from 'interpolateCornersCharuco': chorners = ChArUco corners; chids = ChArUco ids
        // These have been moved to protected members
        //vector<int> chIds;
        //vector<cv::Point2f> chCorners;
        cv::aruco::interpolateCornersCharuco(markersFullRes, mkIds, img, board, chCorners, chIds /*, intrinsics, distCoeffs*/);
        
        strbuf.str(""); strbuf.clear();;
        strbuf << "Estimating pose: number of interpolated chessboard corners: " << chIds.size() ;
        this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->getName()); 
        if (debug_verbose)
        {
            std::cout << strbuf.str() << std::endl;

            // Is the center chessboard corner id=38 among the ones detected? (for 8 x 12 board only)
            //if ( find(chIds.begin(),chIds.end(),38) != chIds.end() )
            //{
            //    std::cout << "Estimating pose: center chessboard corner id=38 is detected" << std::endl;
            //}

        }
        
        // These lines used to be in get_pose(chCorners, chIds, extrinsics_4x4, reprojection_error);
        
        // Use interpolated chessboard corners to estimate the pose, which are much more accurate
        poseEstimationSuccessful = cv::aruco::estimatePoseCharucoBoard(chCorners, chIds, board, cameraMatrix, distCoeffs, rvec, tvec);
        
        if( poseEstimationSuccessful  )
        {
            if(debug_verbose) dumpRvecTvec(rvec, tvec);
            double reproj_error;
            //if ( reprojection_error != nullptr )
            {
                reproj_error = get_reprojection_error(chCorners, chIds);
                strbuf.str(""); strbuf.clear();;
                strbuf << "Reprojection error (RMS): " << reproj_error;
                this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->getName()); 
                if (debug_verbose)
                {
                    std::cout << strbuf.str() << std::endl;
                }
            }
            if (reprojection_error != nullptr)
                *reprojection_error = reproj_error;
            if (reproj_error > this->reprojErrorThreshold)
            {
                poseEstimationSuccessful = false;

                strbuf.str(""); strbuf.clear();;
                strbuf << "Reprojection error ("<< reproj_error <<") exceeded error threshold: " << this->reprojErrorThreshold;
                this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->name);
                if (debug_verbose)
                {
                    std::cout << strbuf.str() << std::endl;
                }
            }
        }
        if (!poseEstimationSuccessful && debug_verbose)
        {
            cout <<"Estimating pose: aruco::estimatePoseCharucoBoard() failed. " << endl;
            this->eHandler->report("Estimating pose: aruco::estimatePoseCharucoBoard() failed.", Shared::Error::Severity::WARN, this->getName()); // this->loggerID);
        }
        processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start_time).count();

        strbuf.str(""); strbuf.clear();;
        strbuf << "Total marker detection + pose estimation time: "<< processing_time << " ms";
        this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->getName()); 
        if (debug_verbose)
        {
            std::cout << strbuf.str() << std::endl;
        }
        // Draw XYZ axes for the board pose 
        //if(debug_verbose)
        //    cv::aruco::drawAxis(img, intrinsics, distCoeffs, rvec, tvec, 0.02);
        //cv::drawFrameAxes(img, intrinsics, distCoeffs, rvec_tmp, tvec_tmp, 0.05, 5);
    }
    else
    {
        return false;  // 'reprojection_error' will not be set in this case
    }
    return poseEstimationSuccessful;
}


//#######################################################################################
// This function implements marker detection with search:
// 1. divide the image into non-overlapping tiles (e.g. 3 x 3)
// 2. perform marker detection in each tile using a scale-down version (for speed)
// 3. collect the min & max of the tiles coordinates that contains >0 markers
// 4. redo marker detection in the region of original image between min & max coordinates
// This function is an alternate to running on the entire (scaled) image directly, originally
// implemented in PoseEstimator::detect_aruco_markers().
bool detect_aruco_markers_with_search(const cv::Mat &img,
                                     cv::Ptr<cv::aruco::Dictionary> aruco_dict,
                                     vector<vector<cv::Point2f> > & my_markers,
                                     vector<int>  & my_mkIds,
                                     cv::Ptr<cv::aruco::DetectorParameters> markerDetectParameters=nullptr,
                                     bool debug=false)
{
    int width_div = 4;
    int height_div = 4;
    int tile_width = img.cols / width_div;
    int tile_height = img.rows / height_div;

    int marker_tile_row_min=INT_MAX;
    int marker_tile_row_max=-1;
    int marker_tile_col_min=INT_MAX;
    int marker_tile_col_max=-1;
    for (int row=0; row < height_div; row ++)
    {
        int row_start = row * tile_height;
        for (int col=0; col < width_div; col ++)
        {
            int col_start = col * tile_width;
            cv::Rect roi(cv::Point2i(col_start, row_start), cv::Size(tile_width, tile_height));
            if(debug)
            {
                std::cout << "Tile "<<row <<", "<<col << ", roi="<< roi<< std::endl;
            }
            //cv::resize(img(roi), tile, cv::Size(), markerDetectionScale, markerDetectionScale, cv::INTER_LANCZOS4);
            cv::aruco::detectMarkers(img(roi), aruco_dict, my_markers, my_mkIds, markerDetectParameters);
            if(my_mkIds.size()>0)
            { // update the region that covers the area with >0 markers
                if(debug)
                    std::cout<<"# detected markers: "<< my_mkIds.size() << std::endl;
                if (marker_tile_row_min>row_start) marker_tile_row_min = row_start;
                if (marker_tile_row_max < row_start + tile_height) marker_tile_row_max = row_start + tile_height;
                if (marker_tile_col_min > col_start) marker_tile_col_min = col_start;
                if (marker_tile_col_max < col_start + tile_width) marker_tile_col_max = col_start + tile_width;
            }
        }
    }

    if (marker_tile_col_min < marker_tile_col_max && marker_tile_row_min < marker_tile_row_max)
    {
        cv::Rect roi(cv::Point2i(marker_tile_col_min, marker_tile_row_min), cv::Point2i(marker_tile_col_max-1, marker_tile_row_max-1));
        if(debug) std::cout <<"Re-detecting markers in ROI:"<< roi << std::endl;
        cv::Mat region;
        //cv::resize(img(roi), region, cv::Size(), markerDetectionScale, markerDetectionScale, cv::INTER_LANCZOS4);
        cv::aruco::detectMarkers(img(roi), aruco_dict, my_markers, my_mkIds, markerDetectParameters);
        if(my_mkIds.size()>0)
        {
            //shift the marker coordinates by (marker_tile_col_min, marker_tile_row_min)
            cv::Point2f offset(marker_tile_col_min, marker_tile_row_min);
            if(debug) std::cout << "Marker location offset:" << offset << std::endl;
            for(size_t idx=0; idx<my_markers.size(); idx++)
            {
               vector<cv::Point2f> & marker = my_markers[idx];
               for (size_t i=0; i< 4; i++) // iterate over corners of the marker & add offset
               {
                   marker[i] += offset;
               }
            }
            return true;
        }
        else
        {
            return false;
        }
    }
    else {
        return false;
    }
}

//#######################################################################################
// Return true if # of detected marker exceeds threshold; Also returned on success is the
// location of the center chessboard corner.
// Description:
// - scale the image
// - detect aruco markers
// - if the # of markers exceeds the minNumMarkersThreshold, proceed, else return false
// - determine if centerMarkerId1 and centerMarkerId2 are among the detected markers
//   if so, proceed, else return false
// - carry out chessboard corner interpolation given the 2x center markers
//   if successful, 
//     - set the centerCorner (value to be returned) to the interpolated center corner position
//     - set scaledImgMkrDetectionSuccessful to true
//     - return true
//   else proceed to next step (fallback method) (TBD)
//
// Side-effect:
//   when scaledImgMkrDetectionSuccessful is true, a subsequent call of estimate_pose_charucoboard()
//   will reuse the detected markers for the scaled image, and proceed w/ corner interpolation.
//   Otherwise, estimate_pose_charucoboard() will carry out marker detection on the full-resolution image
//   first. This approach is for speeding up tracking where most of the time, marker detection is all we
//   needed to do, and full-res marker detection is too costly.
bool PoseEstimator::detect_aruco_markers(const cv::Mat &img, cv::Point2f & centerCorner)
{
	auto board{ this->fiducialModel.getBoard() };
    if (imageWidth == -1 || imageHeight == -1)
    {
        //initialize on first use:
        imageWidth = img.cols;
        imageHeight = img.rows;
    }
    else {
        assert(imageWidth == img.cols && imageHeight == img.rows);
    }
    strbuf.str(""); strbuf.clear();;
    strbuf << "Aruco marker detection ... Scale: " << this->markerDetectionScale ;
    this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->getName()); 
    if (debug_verbose)
    {
        std::cout << strbuf.str() << std::endl;
    }

    if (markerDetectionScale==1.0)
    {
        markerFrame = img.clone();
    }
    else
    {
        cv::resize(img, markerFrame, cv::Size(), markerDetectionScale, markerDetectionScale, cv::INTER_LANCZOS4);
    }
    //markers and mkIds are members
	vector<int> ids2;
    vector<vector<cv::Point2f> > mks2;
    cv::aruco::detectMarkers(markerFrame, board->dictionary, markers, mkIds, markerDetectParameters);
    //detect_aruco_markers_with_search(markerFrame, board->dictionary, markers, mkIds, markerDetectParameters, debug_verbose);

    if(debug_verbose
            && true)  // debugging feature, remove this to activate
    {
        cv::Mat frame = markerFrame.clone();
        cv::aruco::drawDetectedMarkers(frame, markers, mkIds);
        //markerDisplay.createDisplayFrame(frame);
        VideoObject::imshow("Marker Detection", frame); //alt. simpler API
        //markerDisplay.writeVideoFrame();
    }

    strbuf.str(""); strbuf.clear();;
    strbuf << "Number of marker detected: " << mkIds.size();
    this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->getName()); 
    if (debug_verbose)
    {
        std::cout << strbuf.str() << std::endl;
    }

    scaledImgMkrDetectionSuccessful = static_cast<int>(mkIds.size()) >= minNumMarkersThreshold;
    // The following code assumes we have a 8 x 12 ChArUco board, and we find the center corner
    // by interpolating based on Marker #21 & 26
    if (scaledImgMkrDetectionSuccessful)
    {
        size_t index=-1;
        for(index = 0; index < (mkIds.size()); index++)
        {
            if (mkIds[index] != centerMarkerIds.at(0) && mkIds[index] != centerMarkerIds.at(1)) // 21 or 26 are closest markers to the center of our board w/ 48 markers
                continue;
            strbuf.str(""); strbuf.clear();;
            strbuf << "Detected center marker, id: " <<  mkIds[index];
            this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->getName()); 
            if (debug_verbose)
            {
                std::cout << strbuf.str() << std::endl;
            }
            ids2.push_back(mkIds.at(index));
            mks2.push_back(markers.at(index));
        }

        if(ids2.size()>1) //both 21 and 26 are detected, we interpolate chessboard corner between the 2 markers
        {
            vector<int> chIds;
            vector<cv::Point2f> chCorners;
            cv::aruco::interpolateCornersCharuco(mks2, ids2, markerFrame, board, chCorners, chIds);
            if (chIds.size()>0)
            {
                strbuf.str(""); strbuf.clear();;
                strbuf << "Center corner interpolation successful: id=" << chIds[0] << ", " << chCorners[0] ;
                this->eHandler->report(strbuf.str(), Shared::Error::Severity::DEBUG, this->getName()); 
                if (debug_verbose)
                {
                    std::cout << strbuf.str() << std::endl;
                }
                centerCorner = chCorners[0]/markerDetectionScale; //return in the scale of the original image
            }
            else
            {
                strbuf.str(""); strbuf.clear();;
                strbuf << "Center corner interpolation failed";
                this->eHandler->report(strbuf.str(), Shared::Error::Severity::WARN, this->getName()); 
                if (debug_verbose)
                {
                    cout << strbuf.str()<< endl;
                }
                scaledImgMkrDetectionSuccessful = false;
            }
        }
        else
            scaledImgMkrDetectionSuccessful = false;

        // fallback method if the above steps fail
        if (!scaledImgMkrDetectionSuccessful)
        {
            // Now that we have failed: a) detecting both 1st and 2nd center markers, or b) failed to get interpolated
            // center chessboard corners from the 2 markers, we move to the fallback: find any detected marker in
            // centerMarkerIds, and use its coordinates as the anchor and return it as centerCorner.
            vector<cv::Point2f> centerMarker{};
            bool centerMarkerDetected{false};
            for(auto i : centerMarkerIds )
            {
                // Is marker id i in (the detected) mkIds?
                vector<int>::iterator mkIdx = find(mkIds.begin(), mkIds.end(), i);
                if (mkIdx != mkIds.end()) //found
                {
                    size_t idx = mkIdx - mkIds.begin();
                    centerMarker = markers[idx];
                    centerMarkerDetected = true;

                    strbuf.str(""); strbuf.clear();;
                    strbuf << "Fallback center marker id="<< i << " is detected, mkIds[i]="<< mkIds[idx];
                    this->eHandler->report(strbuf.str(), Shared::Error::Severity::WARN, this->getName()); 
                    if(debug_verbose)
                    {
                        cout << strbuf.str() << endl;
                    }
                    break;
                }
            }
            // If one of the markers in centerMarkerIds is detected, we use it as the anchor and return the average position
            // of its 4 corners as the return value for centerCorner.
            if(centerMarkerDetected)
            {
                centerCorner = cv::Point2f(0.0f, 0.0f);
                for(auto c : centerMarker)
                {
                    centerCorner += c;
                }
                centerCorner /= (4.0f * markerDetectionScale);
                scaledImgMkrDetectionSuccessful = true;
            }
        }

        if(!scaledImgMkrDetectionSuccessful)
        {
            centerCorner = cv::Point2f(-1.0, -1.0); // mark it as no marker 21 or 26 was found
        }
    }
    return scaledImgMkrDetectionSuccessful;
}

//#######################################################################################
double PoseEstimator::get_reprojection_error(const vector<cv::Point2f> &chCorners,  const vector<int> &chIds)
{
	auto board{ this->fiducialModel.getBoard() };
	auto cameraMatrix{ this->cameraModel.cvMatrix.cameraMatrix };
	auto distCoeffs{ this->cameraModel.cvMatrix.distortionCoeffs };
    double reprojection_error;

    // We use the chessboard corners for reprojection error. 
    // First get the object points of the corners from the board that corresponds to the detected chCorners

    vector<cv::Point2f> imgpoints;
    vector<cv::Point3f> objpoints;
    for (size_t i=0; i<chCorners.size(); i++)
    {
        objpoints.push_back(board->chessboardCorners[chIds[i]]); // pick the ones we have detected corners for in chCorners
    }
    // now project the object points to the image to get the imgpoints
    cv::projectPoints(objpoints, rvec, tvec, cameraMatrix, distCoeffs, imgpoints);
    // Compute the RMS of the image points of the corners and reprojected points under the rvec & tvec
    // The following equation comes from OpenCV 4.1.2, modules/calib3d/src/solvepnp.cpp
    reprojection_error = cv::norm(chCorners, imgpoints, cv::NORM_L2) / sqrt(2*chCorners.size());
    
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
    
    return reprojection_error;
}

int PoseEstimator::observedMarkerCount() const {
	return static_cast<int>(this->mkIds.size());
}
