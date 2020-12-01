#ifndef BOEINGMETROLOGYLIB_MULTICAMERAOBSERVATION_H
#define BOEINGMETROLOGYLIB_MULTICAMERAOBSERVATION_H

#include <string>
#include <map>
#include "Calibration/CameraCalibrationUtilities.h"
#include "json/value.h"
#include "Calibration/CalibrationObject/AbstractObjectBase.h"
#include "Common/Interface/Serializer.h"
#include "CameraObservation.h"
#include "TypeDefs.h"
#include "Utilities/Utilities.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		using namespace CalibrationObject;
		namespace Observation
		{


            class BOEINGMETROLOGYLIB_API MultiCameraObservation : public Boeing::Interface::Serializer
			{
			public:
				void FilteredObservationsByCamera(MultiCameraObservation &obj, const CAMERA_REGEX_STRING &cameraRegex);

				std::map<CAMERA_NAME, CameraObservation> cameraObservation;
				std::string poseIdentifier;
				int poseId;

				void AddCameraPose(const CAMERA_NAME &cameraName, const CameraObservation &cameraObservationPose);
				CameraObservation & GetCameraPose(const CAMERA_NAME &cameraName);
                CameraObservation GetCameraPoseCopy(const CAMERA_NAME &cameraName) const;

				bool contains(const std::string &cameraName) const;

				void RetrieveCameraObservationPoints(const std::string &cameraName, std::vector<std::vector<int>> &cameraMarkerIds,
					std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
					std::vector<std::vector<cv::Point2f>> &cameraImagePoints,
					cv::Size &imageSize,
					std::vector<std::vector<int>> *cameraPoseIndex = NULL);

                void RetrieveCameraObservationPoints(const std::string &cameraName, std::vector<std::vector<int>> &cameraMarkerIds,
                    std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
                    std::vector<std::vector<cv::Point2f>> &cameraImagePoints,
                    cv::Size &imageSize,
                    std::vector<std::pair<std::vector<int>, std::string>> *cameraPoseIndex = NULL);

				std::map<MARKER_IDENTIFIER, std::map<CAMERA_NAME, int>>  retrieveSharedMarkers();
				//Informational
				std::string folderNameOfObservation;
				void JsonSerialize(Json::Value &jsonNode) const override;
                void JsonDeserialize(const Json::Value &jsonValue) override;
			};
		}
	}
}
#endif
