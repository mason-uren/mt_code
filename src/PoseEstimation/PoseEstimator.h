// PoseEstimator.h : Header file for determining the 6DOF pose of a fiducial in a single frame of video
//

#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <exception>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Numpy
#include "CppNpy/numpy.h"

// HRL
#include <Shared/Presets.h>
#include <Shared/ModelsConfig.h>

// FIXME - this is coming from DE, not Models
#include "Model/PanTiltModel.h"

// Models
#include "Board/FiducialModel.h"
#include "Camera/CameraModel.h"

// Utilities
#include "OpenCVHelpers/Video/VideoObject.h"
#include "OpenCVHelpers/Transformations/PoseTransformations.h"
#include "ThreadInterface/ConfigInterface.h"

#include "SixDOF.h"
#include "HandOffSolver/HandOffSolver.h"

//#######################################################################################
class PoseEstimator: public ConfigInterface
{
public:
    PoseEstimator(const CameraModel<double> & cameraModel, const FiducialModel & fiducialModel,
                  const bool debug = false, const double & markerDetectionScale = 0.25, const double & inputImageScaleFactor = 1.0,
                  const std::string & caller = Presets::Logger::DEFAULT_ID);
    PoseEstimator(const std::string & cameraMatrixPath, const std::string & distCoeffsPath, 
                  bool altCharucoBoard=false, bool debug=false, 
                  double markerDetectScale=0.25, double inputImageScaleFactor=1.0,
                  const std::string & caller = Presets::Logger::DEFAULT_ID);
    PoseEstimator(const std::string & cameraMatrixPath, const std::string & distCoeffsPath,
                  const cv::Ptr<cv::aruco::CharucoBoard> charucoBoard,
                  double markerDetectScale=0.25, bool Debug=false, double inputImageScaleFactor=1.0,
                  const std::string & caller = Presets::Logger::DEFAULT_ID);
    PoseEstimator(const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs,
                  const cv::Ptr<cv::aruco::CharucoBoard> charucoBoard,
                  double markerDetectScale=0.25, bool Debug=false, double inputImageScaleFactor=1.0,
                  const std::string & caller = Presets::Logger::DEFAULT_ID);
    virtual ~PoseEstimator() = default;

	// Copy constructor/assignment
	PoseEstimator(const PoseEstimator & obj) noexcept :
		ConfigInterface(obj.name),
		fiducialModel(obj.fiducialModel),
		cameraModel(obj.cameraModel)
	//	//,
	//	//rvec(obj.rvec),
	//	//tvec(obj.tvec),
	//	//markerFrame(obj.markerFrame),
	//	//markerDetectParameters(obj.markerDetectParameters),
	//	//imageHeight(obj.imageHeight),
	//	//imageWidth(obj.imageWidth),
	//	//ifov(obj.ifov),
	//	//scaleFactor(obj.scaleFactor),
	//	//scaledImgMkrDetectionSuccessful(obj.scaledImgMkrDetectionSuccessful),
	//	//poseEstimationSuccessful(obj.poseEstimationSuccessful),
	//	//reprojErrorThreshold(obj.reprojErrorThreshold),
	//	//centerMarkerIds(obj.centerMarkerIds),
	//	//centerCornerId(obj.centerCornerId),
	//	//centerCorner(obj.centerCorner),
	//	//debug_verbose(obj.debug_verbose),
	//	//boardCenter(obj.boardCenter),
	//	//markerDetectionScale(obj.markerDetectionScale),
	//	//mkIds(obj.mkIds),
	//	//markers(obj.markers),
	//	//minNumMarkersThreshold(obj.minNumMarkersThreshold),
	//	//chIds(obj.chIds),
	//	//chCorners(obj.chCorners)
	{}
	PoseEstimator & operator=(const PoseEstimator & obj) noexcept {
		fiducialModel = obj.fiducialModel;
		cameraModel = obj.cameraModel;
	//    rvec = obj.rvec;
	//    tvec = obj.tvec;
	//    markerFrame = obj.markerFrame;
	//    markerDetectParameters = obj.markerDetectParameters;
	//	imageHeight = obj.imageHeight;
	//	imageWidth = obj.imageWidth;
	//	ifov = obj.ifov;
	//	scaleFactor = obj.scaleFactor;
	//	scaledImgMkrDetectionSuccessful = obj.scaledImgMkrDetectionSuccessful;
	//	poseEstimationSuccessful = obj.poseEstimationSuccessful;
	//	reprojErrorThreshold = obj.reprojErrorThreshold;
	//	centerMarkerIds = obj.centerMarkerIds;
	//	centerCornerId = obj.centerCornerId;
	//	centerCorner = obj.centerCorner;
	//	debug_verbose = obj.debug_verbose;
	//	boardCenter = obj.boardCenter;
	//	markerDetectionScale = obj.markerDetectionScale;
	//	markers = obj.markers;
	//	mkIds = obj.mkIds;
	//	minNumMarkersThreshold = obj.minNumMarkersThreshold;
	//	chIds = obj.chIds;
	//	chCorners = obj.chCorners;
        return *this;
	}

public:
    bool   init(const std::string & modelJSON, const std::string & cameraMatrixPath, const std::string & distCoeffsPath);
    bool   detect_aruco_markers(const cv::Mat &img, cv::Point2f & markerCorner);
    bool   estimate_pose_charucoboard(const cv::Mat &img, double *reprojection_error=nullptr);
    virtual bool   estimate_pose_charucoboard(const cv::Mat &frame, double pan, double tilt, SixDOF *result=nullptr);
    bool   get_rvec_tvec_charucoboard(cv::Mat & rvec, cv::Mat & tvec);
    bool   get_extrinsics4x4_charucoboard(cv::Mat & extrinsics4x4);
    bool   get_corner_uv_id_charucoboard(std::vector<cv::Point2f> & corners, std::vector<int> & ids);
    cv::Point2f get_center_corner_uv_charucoboard();
    void   set_board_detection_threshold(int min_markers);
    cv::Point2f   estimate_needed_pantilt(const cv::Point2f & goal, const cv::Point2f & fiducialCenter);
    virtual bool  estimate_needed_pantilt(const SixDOF & boardPose, cv::Vec2d & ptInitGuess);
    int observedMarkerCount() const;
    
// Operations
protected:
    double          get_reprojection_error(const std::vector<cv::Point2f> &chorners, const std::vector<int> &chids);
    double          get_focal_length();
    //bool            load_camera_intrinsics(const std::string & cameraMatrixPath, const std::string & distCoeffsPath);
    void            determine_marker_detection_parameters();
	void			setInstananeousFOV(const double & fudgeFactor = 1); // 1.3 is a fudge factor because the intrinsics are not correct

private:
    // Storage for the pose when estimate_pose_charucoboard() is called
    cv::Mat         rvec, tvec;

// Attributes
protected:

	bool            debug_verbose;

    //const float     squareLength = 0.012F;      // These values are for the 4"x6" fiducial that we use for 6DOF estimation
    //const float     markerLength = 0.009F;       // located in "Charuco_Specific/CharucoBoards.py"
    //const int       charucoX = 8;
    //const int       charucoY = 12;
    std::vector<int> centerMarkerIds;
    int             centerCornerId;
    cv::Point3f     centerCorner;
    double          reprojErrorThreshold;

    //cv::Mat         cameraMatrix;
    //cv::Mat         distCoeffs;
    double          scaleFactor;
    double          ifov;  // instantaneous FOV (of an image pixel)
    int             imageWidth, imageHeight;

    cv::Point3f     boardCenter;

    double markerDetectionScale;
    cv::Mat         markerFrame;
    std::vector<int>     mkIds;
	std::vector<std::vector<cv::Point2f> > markers;
    bool            scaledImgMkrDetectionSuccessful, poseEstimationSuccessful;
    int             minNumMarkersThreshold;
    cv::Ptr<cv::aruco::DetectorParameters> markerDetectParameters;
    
	std::vector<int> chIds;
	std::vector<cv::Point2f> chCorners;

	FiducialModel fiducialModel{};
	CameraModel<double> cameraModel;

	std::stringstream  strbuf;  // for tmp string output
};

class PoseEstimatorWorld : public PoseEstimator 
{
public:
    PoseEstimatorWorld(const std::string & cadModelJsonPathname, const CameraModel<double> & cameraModel, const FiducialModel & fiducialModel,
                       bool debug = false, double markerDetectScale=0.25, double inputImageScaleFactor=1.0,
                       const std::string & caller = Presets::Logger::DEFAULT_ID);
    PoseEstimatorWorld(const std::string & cadModelJsonPathname, const std::string & cameraMatrixPath, const std::string & distCoeffsPath,
                       bool altCharucoBoard=false, bool debug=false, double markerDetectScale=0.25, double inputImageScaleFactor=1.0,
					   const std::string & caller = Presets::Logger::DEFAULT_ID);
	~PoseEstimatorWorld() override = default;

	// Copy constructor/assignment
	//PoseEstimatorWorld(const PoseEstimatorWorld & obj) :
	//	pt_model(obj.pt_model)
	//{}
	//Pose

/*
    PoseEstimatorWorld(const string & cadModelJsonPathname, const string & cameraMatrixPath, const string & distCoeffsPath,  
                  const cv::Ptr<cv::aruco::CharucoBoard> charucoBoard,
                  double markerDetectScale=0.25F, bool Debug=true, double inputImageScaleFactor=1.0F);
    PoseEstimatorWorld(const string & cadModelJsonPathname, const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs, 
                  const cv::Ptr<cv::aruco::CharucoBoard> charucoBoard,
                  double markerDetectScale=0.25F, bool Debug=true, double inputImageScaleFactor=1.0F);
*/
    virtual bool     estimate_pose_charucoboard(const cv::Mat &frame, double pan, double tilt, SixDOF *result=nullptr) override;
    virtual bool     estimate_needed_pantilt(const SixDOF & sharedPose, cv::Vec2d & ptInitGuess) override;

protected:
	// Variables
	Model::PanTilt<> pt_model;

private:
	// Functions
    bool            load_cad_model(const std::string &  modelFilename);
};
