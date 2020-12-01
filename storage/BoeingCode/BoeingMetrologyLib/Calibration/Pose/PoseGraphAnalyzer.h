#ifndef BOEINGMETROLOGYLIB_POSEGRAPHANALYZER_H
#define BOEINGMETROLOGYLIB_POSEGRAPHANALYZER_H

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
#include "PoseGraphAnalyzerResults.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        namespace Pose
        {
            class BOEINGMETROLOGYLIB_API PoseGraphAnalyzer : public Boeing::Interface::Serializer
            {
            public:

                std::map<POSE_NAME, SinglePoseGraph> poseGraph;
                std::map<POSE_NAME, MultiCameraExtrinsicData> poseExtrinsicData;
                std::map<POSE_NAME, std::map<CAMERA_NAME, ObservationPoints<float>>> poseObservationPoints;

                PoseGraphAnalyzerResults analysisFilterResults;

                PoseGraphAnalyzer()
                {
                }

                // Populate poseGraph and poseExtrinsicData
                void populatePoseGraph(MultiPoseObservations & observations, MultiCameraIntrinsicData & multiCameraIntrinsicData, const double & reprojThreshold, 
                    const int & minimumPointsPerView, const bool& optimizeEachPose = true);

                // For each pose, and each camera pair that observed that pose, compute the relative difference
                // between nominal camera pair distance and the observed camera pair distance
                std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> CompareExtrinsicData(const Interface::ICameraPairDistances *comparisonExtrinsics);

                // For each pose, and each camera pair that observed that pose, compute the distance between cameras
                void ComputeCameraOffsets(std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> & offsetsPerPose, 
                    std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, double>> & offsetPerCameraPair);

                // For each pose, and each camera pair that observed that pose, compare back-projected 3D observations
                void CompareCameraPairBackProjected3dPoints(const MultiCameraIntrinsicData & multiCameraIntrinsicData,
                    std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, std::map<MARKER_IDENTIFIER, std::pair<cv::Point3d, cv::Point3d>>>> & backprojected3dPointPairs);

                // For each pose, and each camera pair that observed that pose, compute the 3d position vector between cameras
                std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, cv::Point3d>> ComputeRelativeCameraPositionVectors();

                // Obtain results for pose on the whole
                std::map<POSE_NAME, bool> filterByPoseTolerance(double tolerance, std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparison);

                // Obtain results for pose by filtering
                std::map<POSE_NAME, bool> filterByPoseDeviation(double deviationTolerance, double absoluteTolerance, std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparison);

                // Obtain results for each pose and camera pair
                std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, bool>> filterBytolerance(double tolerance, std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparison);

                //Obtain results for each pose and camera pair by deviation
                std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, bool>> filterByDeviation(double deviationTolerance, double absoluteTolerance, std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparison);

                // Determine observations that should be filtered because they shared a pose with an outlier camera offset
                void filterObsByCameraPairOffset(const std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, double>> & offsetsPerCameraPair, 
                    const size_t & minNumObs, const double & stdDevThresh, std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> & filter);

                // Populate this object from the input stream.
                // Throws exception on failure.
                void JsonDeserialize(const Json::Value &jsonNode) override;

                // Populate the output Json from this object's data members.
                // Throws exception on failure.
                void JsonSerialize(Json::Value &jsonNode) const override;

            };
        }
    }
}
#endif
