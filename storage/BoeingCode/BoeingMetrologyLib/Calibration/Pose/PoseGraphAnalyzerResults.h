#pragma once
#include "cv.h"
#include <vector>
#include <string>
#include <map>
#include "json/value.h"
#include "Calibration/MultiCameraIntrinsicData.h"
#include "Calibration/MultiCameraExtrinsicData.h"
#include "Calibration/Observation/CameraObservation.h"
#include "Calibration/Observation/MultiPoseObservations.h"
#include "Common/Interface/Serializer.h"
#include "PoseGraphEdgeVertex.h"
#include "SinglePoseGraph.h"

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		namespace Pose
		{
            class BOEINGMETROLOGYLIB_API  PoseGraphAnalyzerResults : public Boeing::Interface::Serializer
			{
			public:
				PoseGraphAnalyzerResults()
				{
				}

                // Accept/reject observations within a pose
                std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> cameraOffsetObsFilter;

				// Accept/reject result per pose
				std::map<POSE_NAME, bool> filterByPoseTolerance;

				// Accept/reject result per camera pair and pose
				std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, bool>> filterBytolerance;

				// Residual distance between nominal and observed
				// Per pose and camera pair
				std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> comparisonDistanceInformation;

				// Absolute distance between camera pairs per pose
				std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> absoluteDistancePerPoseInformation;

                // Absolute distance between camera pairs per camera pair
                std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, double>> absoluteDistancePerCameraPairInformation;

				// 3d position vector between camera pairs per pose
				std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, cv::Point3d>> positionVectorInformation;

				double comparisonTolerance = 0.0;

				// Populate this object from the input stream.
				// Throws exception on failure.
				void JsonDeserialize(const Json::Value &jsonNode) override;

				// Populate the output Json from this object's data members.
				// Throws exception on failure.
				void JsonSerialize(Json::Value &jsonNode) const override;

				// Populate the output Json from this object's data members.
				// Throws exception on failure.
				void SerializePerPose(std::ostream &strm) const;

                // Populate the output Json from this object's data members.
                // Throws exception on failure.
                void SerializePerCameraPair(std::ostream &strm) const;

				// Create summary results and send to stream.
				// Throws exception on failure.
				void SerializeSummary(std::ostream &strm) const;

				// Populate a json from this object's data members and write it to file.
				// Throws exception on failure.
				void SerializeFile(const std::string &fileName) const override;

				// Populate this object from a json file.
				// Throws exception on failure
				void DeserializeFile(const std::string &fileName) override;

			};
		}
	}
}
