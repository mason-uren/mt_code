#pragma once
#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "IntrinsicData.h"
#include <map>
#include <set>
#include "Observation/MultiPoseObservations.h"
#include "CalibrationObject/ArucoBoard.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
        class BOEINGMETROLOGYLIB_API MultiCameraIntrinsicData : public Boeing::Interface::Serializer
		{
		public:
			std::map<CAMERA_NAME, IntrinsicData> cameraData;

			// Populate this object from the input json.
			// Throws exception on failure.
			void JsonDeserialize(const Json::Value &jsonNode) override;

			// Populate the Json node from this object's data members.
			// Throws exception on failure.
			void JsonSerialize(Json::Value &jsonNode) const override;

			MultiCameraIntrinsicData() {};

			// Compute intrinsics for a set of observations.  Optional initial intrinsics can be provided.
			// Optional pose filter can be provided. 
			MultiCameraIntrinsicData(Observation::MultiPoseObservations & multiPoseObservations,
                const int & flags, const double & reprojThreshold, const int & minimumPointsPerView, ArucoBoard & arucoBoard, 
                const MultiCameraIntrinsicData * initialMultiCameraIntrinsicData = nullptr,
				const std::map<POSE_NAME, bool> * cameraPoseFilter = nullptr);

            // Initialize with some maps
            MultiCameraIntrinsicData(const std::map<CAMERA_NAME, cv::Mat> & cameraMatrixMap, const std::map<CAMERA_NAME, cv::Mat> & distortionCoeffsMap);

			//Iterate over a pose observations and compute undistorted image locations for each observation
			std::map<POSE_NAME, MultiCameraObservation> GetUndistortedObservations(Observation::MultiPoseObservations observations);

            // Generate observation data by loading images from disk, loading ChAruco board descriptions from disk, 
            // performing detection and finally filtering results based on reprojection error output by 3d pose estimation
            // using the provided intrinsics.  
            static void GenerateObservationData(const std::string &baseFolderDirty, const int numRequiredDetectedMarkers,
                const MultiCameraIntrinsicData & multiCameraIntrinsicData, std::set<CAMERA_NAME> & failedCameras,
                AbstractObjectBase *detector /*= NULL*/, const int & maxReprojThreshold /*= 0*/);
		};
	}
}