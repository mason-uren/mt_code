#ifndef BOEINGMETROLOGYLIB_Opfor2d23d_H
#define BOEINGMETROLOGYLIB_Opfor2d23d_H

#include "BoeingMetrologyLib_API.h"
#include <string>
//#include "cv.h"
//#include "TypeDefs.h"
//#include "Calibration/MultiCameraExtrinsicData.h"
//#include "Calibration/MultiCameraIntrinsicData.h"
//#include "Calibration/Observation/MultiCameraObservation.h"
//#include "Calibration/Observation/MultiPoseObservations.h"


namespace BoeingMetrology
{
    void BOEINGMETROLOGYLIB_API Opfor2d23d(std::string ioFolder, std::string imgFile, std::string plyFile);

	//cv::Point3d ProjectMutualObservations(std::vector<CAMERA_NAME> cameras,
	//	Calibration::MultiCameraExtrinsicData extrinsics,
	//	Calibration::MultiCameraIntrinsicData intrinsics,
	//	std::vector<cv::Point2f> mutualObservations);

	//cv::Vec2d UndistortPixel(cv::Point2f srcPixel, Calibration::IntrinsicData intrinsics);
	//cv::Vec3d GetPixelRay(cv::Vec2d undistortedPixel, 
	//	Calibration::IntrinsicData intrinsics,
	//	Calibration::ExtrinsicData extrinsics);


	//typedef int CAMERA_ID;
	//typedef int MARKER_ID;
	//typedef int OBSERVATION_INDEX;
	//class BOEINGMETROLOGYLIB_API FeatureBasedPointCloud
	//{
	//public:
	//	std::vector<cv::Point3d> mWorldPoints;
	//	std::vector<CAMERA_NAME> mCameras;
	//	Calibration::MultiCameraExtrinsicData mCameraExtrinsics;
	//	Calibration::MultiCameraIntrinsicData mCameraIntrinsics;

	//	//here CAMERA_ID will refer to the ID of the camera in the mCameras vector
	//	//maps from markerIds in each of the shared cameras to the set of cameras which observe it
	//	//and the index of the marker in their respective marker vector
	//	std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>> mMutualObservations;

	//	Calibration::MultiCameraObservation mObservations;
	//	std::map<MARKER_ID, cv::Point3d*> mMarkerIdToPointMap;
	//	bool mHasObservationData;

	//	FeatureBasedPointCloud();
	//	FeatureBasedPointCloud(Calibration::MultiCameraObservation observations,
	//		Calibration::MultiCameraExtrinsicData extrinsics, 
	//		Calibration::MultiCameraIntrinsicData intrinsics);
	//	
	//};
}

#endif