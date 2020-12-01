#pragma once

#include "BoeingMetrologyLib_API.h"
#include "cv.h"
#include "TypeDefs.h"
#include "Calibration/MultiCameraExtrinsicData.h"
#include "Calibration/MultiCameraIntrinsicData.h"
#include "Calibration/Observation/MultiCameraObservation.h"
#include "Calibration/Observation/MultiPoseObservations.h"



namespace BoeingMetrology
{
	namespace Features
	{
		cv::Point3d BOEINGMETROLOGYLIB_API ProjectMutualObservations(std::vector<CAMERA_NAME> cameras,
			Calibration::MultiCameraExtrinsicData extrinsics,
			Calibration::MultiCameraIntrinsicData intrinsics,
			std::vector<cv::Point2f> mutualObservations,
			bool &isValid);

		cv::Vec3d BOEINGMETROLOGYLIB_API GetPixelRay(cv::Vec2d undistortedPixel,
			Calibration::IntrinsicData intrinsics,
			Calibration::ExtrinsicData extrinsics);


		cv::Point2f BOEINGMETROLOGYLIB_API reprojectPoint(float *Rt, float *k, float fx, float fy, float cx, float cy, cv::Point3d p);

		enum ErrorType
		{
			PlaneFittingL2Error = 0,
			PlaneFittingMaxError = 1,
			P2PL2Error = 2,
			P2PMaxError = 3,
			ReprojL2Error = 4,
			ReprojMaxError = 5,
			ErrorTypeCount = 6
		};

		typedef int CAMERA_ID;
		typedef int MARKER_ID;
		typedef int OBSERVATION_INDEX;
		class BOEINGMETROLOGYLIB_API FeatureBasedPointCloud
		{
		public:
			POSE_NAME sourcePose;
			std::vector<cv::Point3d> mWorldPoints;
			std::vector<CAMERA_NAME> mCameras;
			Calibration::MultiCameraExtrinsicData mCameraExtrinsics;
			Calibration::MultiCameraIntrinsicData mCameraIntrinsics;

			//here CAMERA_ID will refer to the ID of the camera in the mCameras vector
			//maps from markerIds in each of the shared cameras to the set of cameras which observe it
			//and the index of the marker in their respective marker vector
			std::map<MARKER_ID, std::vector<std::pair<CAMERA_ID, OBSERVATION_INDEX>>> mMutualObservations;

			Calibration::MultiCameraObservation mObservations;
			std::map<MARKER_ID, cv::Point3d> mMarkerIdToPointMap;
			std::map<MARKER_ID, cv::Point3d> mMarkerIdToControlPoints;
			bool mHasObservationData;

			std::vector<double> mErrorMeasures;
			std::map<MARKER_ID, double> mPlaneErrors;
			std::map<MARKER_ID, double> mP2PErrors;
			std::map<CAMERA_NAME, std::vector<double>> mReprojErrors;
			int mMaxPlaneErrorId, mMaxP2PErrorId, mMaxReprojErrorId;


			FeatureBasedPointCloud();

			FeatureBasedPointCloud(Calibration::Observation::MultiCameraObservation observations,
				Calibration::MultiCameraExtrinsicData extrinsics,
				Calibration::MultiCameraIntrinsicData intrinsics,
				POSE_NAME pose = "");

			FeatureBasedPointCloud(std::vector<CAMERA_NAME> allCameras,
				std::vector<cv::Size> allCameraSizes,
				std::set<CAMERA_ID> camerasPresent,
				std::vector<std::vector<CAMERA_ID>> cameraIds,
				std::vector<std::vector<cv::Point2f>> imagePoints,
				std::vector<MARKER_ID> markerIds,
				std::vector<cv::Point3d> worldPoints,
				std::vector<cv::Point3d> controlPoints,
				Calibration::MultiCameraExtrinsicData extrinsics,
				Calibration::MultiCameraIntrinsicData intrinsics,
				POSE_NAME pose = "");

			void generateCloud(Calibration::Observation::MultiCameraObservation observations,
				Calibration::MultiCameraExtrinsicData extrinsics,
				Calibration::MultiCameraIntrinsicData intrinsics,
				POSE_NAME pose = "");

			void Initialize();

			void computePlaneFitting();

			void computeP2PErrors();

			void computeReprojectionErrors(std::vector<std::vector<cv::Point2f>> &reprojectionPoints, std::vector<std::vector<MARKER_ID>> &reprojectedObservations);

			void computeErrors();

			void getP2PErrors(double &maxError, int &maxErrorId, double &l2Error);

			void getPlaneErrors(double &maxError, int &maxErrorId, double &l2Error);

			void getReprojErrors(double &maxError, int &maxErrorId, double &l2Error);

			void getError(ErrorType errorType, double &maxError, int &maxErrorId, double &L2Error);

			void drawPoseErrors(ErrorType errorType, std::pair<POSE_NAME, std::vector<std::pair<CAMERA_NAME, cv::Mat>>> *poseErrorVis);

			double getCameraDist();

			void WriteToPly(std::string fullFileName);


			/*FeatureBasedPointCloud(Calibration::MultiPoseObservations poseObservations,
			Calibration::MultiCameraIntrinsicData intrinsics,
			Calibration::MultiCameraExtrinsicData extrinsics);*/


		};

		struct BOEINGMETROLOGYLIB_API FeatureCloudSorter
		{
			ErrorType toCompare;
			FeatureCloudSorter(ErrorType errorType);
			bool operator()(FeatureBasedPointCloud a, FeatureBasedPointCloud b);
		};

		class BOEINGMETROLOGYLIB_API MultiPosePointCloud
		{
		public:
			//std::map<POSE_NAME, FeatureBasedPointCloud> mClouds;
			std::vector<FeatureBasedPointCloud> mClouds;
			std::map<POSE_NAME, int> cloudMap;
			std::vector<CAMERA_NAME> mCameras;
			std::map<CAMERA_NAME, int> mCameraIndexMap;
			std::vector<cv::Point3d> allWorldPoints;
			Calibration::MultiPoseObservations mPoseObservations;

			MultiPosePointCloud(Calibration::MultiPoseObservations poseObservations,
				Calibration::MultiCameraIntrinsicData intrinsics,
				Calibration::MultiCameraExtrinsicData extrinsics);

			MultiPosePointCloud();

			void addCloud(FeatureBasedPointCloud cloud);

			std::vector<POSE_NAME> sortCloudsByParam(ErrorType param);

			void drawTopNPoses(ErrorType errorType, int nPoses,
				std::vector<std::pair<POSE_NAME, std::vector<std::pair<CAMERA_NAME, cv::Mat>>>> &posesVis);

			void writeAllToPlyFile(std::string fullFileName);

			void writeParamsToFile(std::string csvFile);
		};
	}
}