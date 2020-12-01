#ifndef BOEINGMETROLOGYLIB_MULTIPOSEOBSERVATIONS_H
#define BOEINGMETROLOGYLIB_MULTIPOSEOBSERVATIONS_H

#include <string>
#include <cv.h>
#include <map>
#include <set>
#include "json/value.h"
#include <fstream>
#include "Calibration/CalibrationObject/ArucoBoard.h"
#include "MultiCameraObservation.h"
#include "Scanning/Observation/MultiLensStateCameraObservations.h"
#include "Common/Interface/Serializer.h"
#include "Calibration/IntrinsicData.h"
#include <map>

namespace BoeingMetrology
{
    namespace Calibration
    {
        using namespace CalibrationObject;
        namespace Observation
        {
            // Contains MultiCameraObservations spanning multiple poses
            class BOEINGMETROLOGYLIB_API MultiPoseObservations : public Boeing::Interface::Serializer
            {
            private:
                void SerializeStream(std::ostream &strm) const;
                void DeserializeStream(std::istream &strm);

            public:
                // Informational
                std::string folderNameOfObservations;

                std::map<POSE_NAME, MultiCameraObservation> multiCameraPose;
                std::map<CAMERA_NAME, int> cameraObservationCount;
                std::map<CAMERA_NAME, cv::Size> cameraImageSize;
                // Pose number extraction
                std::map<POSE_NAME, int> poseMappingToInt;

                // Get list of camera names
                std::vector<std::string> GetCameraNames() const;

                void FilteredObservationsByCamera(MultiPoseObservations &obj, const CAMERA_REGEX_STRING &cameraRegexString);

                void FilterCameraObservationByCamera(const POSE_NAME &poseName,
                    const CAMERA_NAME &cameraName, CameraObservation &cameraPose, const CAMERA_REGEX_STRING &cameraRegexString);

                void FilterMultiCameraObservationByCamera(const POSE_NAME &poseName,
                    MultiCameraObservation &cameraPose, const CAMERA_REGEX_STRING &cameraRegexString);

                void FilteredObservationsByDistanceTolerance(MultiPoseObservations &obj,
                    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparisonDistanceInformation,
                    const double tolerance);

                bool FilterCameraObservationByDistanceTolerance(const POSE_NAME &poseName,
                    const CAMERA_NAME &cameraName, CameraObservation &cameraPose,
                    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparisonDistanceInformation,
                    const double tolerance);

                void FilterMultiCameraObservationByDistanceTolerance(const POSE_NAME &poseName,
                    MultiCameraObservation &cameraPose,
                    std::map<POSE_NAME, std::map < CAMERA_NAME_PAIR, double >> &comparisonDistanceInformation,
                    const double tolerance);

                // Add observations for a camera
                void AddObservations(const std::string & cameraName, const std::map<POSE_NAME, CameraObservation> & observations);

                // Load data from a folder structure.  Missing detection results will be ignored.
                void ReadObservationData(const std::string &baseFolder, const int numRequiredDetectedMarkers, 
                    Scanning::Observation::MultiLensStateCameraObservations & multiLensStateCameraObservations,
                    std::set<CAMERA_NAME> & failedCameras,
                    AbstractObjectBase *detector = NULL, std::map<CAMERA_NAME, IntrinsicData> intrinsics = std::map<CAMERA_NAME, IntrinsicData>());

                // Retrieve the observations for a single camera
                void RetrieveCameraObservationPoints(const std::string &cameraName, std::vector<ObservationPoints<float>> & observationPoints);

                // Retrieve the observations for a single camera into a friendly format for opencv calibrateCamera
                void RetrieveCameraObservationPoints(const std::string &cameraName, std::vector<std::vector<int>> &cameraMarkerIds,
                    std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
                    std::vector<std::vector<cv::Point2f>> &cameraImagePoints,
                    std::vector<std::vector<int>> *cameraPoseIndex = nullptr, const std::map<POSE_NAME, bool> * cameraPoseFilter = nullptr);

                // Retrieve the observations and associated image filepaths for a single camera into a friendly format for opencv calibrateCamera
                void RetrieveCameraObservationPointsandFiles(const std::string &cameraName,
                    std::vector<std::vector<int>> &cameraMarkerIds, std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
                    std::vector<std::vector<cv::Point2f>> &cameraImagePoints, std::vector<std::string> &poseImageFiles,
                    std::vector<std::vector<int>> *cameraPoseIndex = nullptr, const std::map<POSE_NAME, bool> * cameraPoseFilter = nullptr) const;

                // Retrieve the observations for a single camera indexed by pose name
                void RetrieveCameraObservationPointsandPoseNames(const std::string &cameraName, std::map<POSE_NAME, std::pair<std::vector<cv::Point3f>, std::vector<cv::Point2f>>> & observationsPerPose);

                // Retrieve the observations and associated pose names for a single camera into a friendly format for opencv calibrateCamera.  
                // Optionally, filter entire poses or observations for a pose. 
                void RetrieveCameraObservationPointsandPoseNames(const std::string &cameraName,
                    std::vector<std::vector<int>> &cameraMarkerIds, std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
                    std::vector<std::vector<cv::Point2f>> &cameraImagePoints, std::vector<std::string> &poseNames,
                    std::vector<std::vector<int>> *cameraPoseIndex = nullptr, const std::map<POSE_NAME, bool> * cameraPoseFilter = nullptr, 
                    const std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> * cameraObsFilter = nullptr) const;

                // Retrieve the mutual observations for a pair of cameras into a friendly format for opencv stereo calibrate
                void RetrieveMutualObservationPoints(const std::vector<std::string> & camNames, std::vector<std::vector<cv::Point3f>> & cameraObjectPoints,
                    std::map<std::string, std::vector<std::vector<cv::Point2f>>> & cameraImagePoints, std::vector<std::string> & poseNames);

                // For an individual camera, filter observations by reprojection error
                // cameraObjectPoints and cameraImagePoints are modified in place
                static void FilterObservationsBasedOnReproj(std::vector<std::vector<cv::Point3f>> & cameraObjectPoints,
                    std::vector<std::vector<cv::Point2f>> & cameraImagePoints, const cv::Size & imageSize,
                    cv::Matx33d cameraMatrix, cv::Mat distortionCoeffs, const int & flags, const double & reprojThreshold,
                    const int & minimumPointsPerView, std::vector<std::vector<int>> *poseIds = nullptr, std::vector<std::string> * poseNames = nullptr, 
                    std::vector<std::vector<int>> * charucoIds = nullptr);

                void JsonSerialize(Json::Value &jsonNode) const override;

                void JsonDeserialize(const Json::Value &jsonNode) override;

                void SerializeFile(std::string fileName) const;
                void DeserializeFile(std::string fileName);
            };
        }
    }
}
#endif
