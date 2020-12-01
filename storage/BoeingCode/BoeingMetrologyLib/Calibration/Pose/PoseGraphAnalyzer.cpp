#include "PoseGraphAnalyzer.h"
#include "TypeDefs.h"
#include "Interface/ICameraPairDistances.h"
#include <map>

using namespace BoeingMetrology::Calibration::Observation;

namespace
{
    const double cOutlierDistance = 100.0;

    void ComputeDistributionStats(std::vector<double> &distribution, double &mean, double &deviation, double &median, bool &isSkewed, double &jenkCutoff)
    {

        std::sort(distribution.begin(), distribution.end());
        isSkewed = false;
        mean = 0.0;
        deviation = 0.0;
        median = 0.0;
        jenkCutoff = 0.0;

        for (double val : distribution)
        {
            mean += val;
        }
        mean /= distribution.size();

        double varianceSum = 0.0;
        for (double val : distribution)
        {
            varianceSum += (val - mean)*(val - mean);
        }
        deviation = varianceSum / (distribution.size() - 1);

        median = distribution[distribution.size() / 2];

        isSkewed = std::abs(mean - median) > deviation;

        if (isSkewed)
        {
            double maxJenk = 0.0;
            size_t maxJenkI = distribution.size() * 3 / 4;
            for (size_t i = maxJenkI; i < (int)distribution.size() - 1; ++i)
            {
                double jenk = distribution[i + 1] - distribution[i];
                if (jenk > maxJenk)
                {
                    maxJenkI = i;
                    maxJenk = jenk;
                }
            }
            jenkCutoff = distribution[maxJenkI];
        }
    }
}


void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::JsonSerialize(Json::Value &jsonNode) const
{
    throw std::runtime_error("PoseGraphAnalyzer::JsonSerialize : NOT YET IMPLEMENTED");
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::populatePoseGraph(MultiPoseObservations & observations,
    MultiCameraIntrinsicData & multiCameraIntrinsicData, const double & reprojThreshold, const int & minimumPointsPerView, const bool& optimizeEachPose)
{
    // Iterating through poses
    for (auto &obs : observations.multiCameraPose)
    {
        if (obs.second.cameraObservation.size() > 1)
        {
            // At least two cameras observed this pose.  Add it to the pose graph.
            POSE_NAME poseName = obs.first;

            try
            {
                // Store the marker observations
                for (const auto & camera : obs.second.cameraObservation)
                {
                    this->poseObservationPoints[poseName].insert({ camera.first, (ObservationPoints<float>)camera.second });
                }

                auto &multiObs = obs.second;
                poseGraph[poseName]._verbose = false;
                poseGraph[poseName].initialPoseEstimator(multiObs, multiCameraIntrinsicData, reprojThreshold, minimumPointsPerView);
                poseGraph[poseName].initialize(false);
                std::vector<std::string> lockedCameraNames; // ignored here
                std::map<CAMERA_NAME, double> perCamera2dErrors, perCamera3dErrors; //ignored here
                if (optimizeEachPose)
                {
                    poseGraph[poseName].optimizeExtrinsics(lockedCameraNames, perCamera2dErrors, perCamera3dErrors);
                }
                else
                {
                    poseGraph[poseName].initializeExtrinParam(lockedCameraNames);
                    poseGraph[poseName].populatervectvecFromExtrinParam();
                }
                poseGraph[poseName].packageMultiCameraExtrinsicData(poseExtrinsicData[poseName]);
            }
            catch (std::exception & e)
            {
                throw std::runtime_error("PoseGraphAnalyzer::populatePoseGraph extrinsic calculation failed for pose " + poseName + ".  " + e.what());
            }
        }
    }
    std::cout << "PoseGraphAnalyzer::populatePoseGraph(): " << poseGraph.size() << " poses added" << std::endl;
}

std::map<BoeingMetrology::POSE_NAME, std::map<BoeingMetrology::CAMERA_NAME_PAIR, double>> BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::CompareExtrinsicData(const Interface::ICameraPairDistances *comparisonExtrinsics)
{
    //Compare the distances
    std::map<CAMERA_NAME_PAIR, double> comparisonDistanceInformation =
        comparisonExtrinsics->getCameraPairDistance();

    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> comparison;
    for (auto &obs : poseExtrinsicData)
    {
        POSE_NAME poseName = obs.first;
        auto &extrinsicData = obs.second;

        std::map<CAMERA_NAME_PAIR, double> poseDistanceInformation;
        try
        {
            //Compare the distances
            poseDistanceInformation = extrinsicData.getCameraPairDistance();
            //extrinsicData.ComputeMagnitudeOfDistanceBetweenAllCameraPairs(poseDistanceInformation);
        }
        catch (...)
        {
            std::string temp = "PoseGraphAnalyzer::CompareExtrinsicData : call to get all camera pair distances failed";
            std::cout << temp << std::endl;
        }

        for (auto &distances : poseDistanceInformation)
        {
            const CAMERA_NAME_PAIR &cameraPair = distances.first;
            if (comparisonDistanceInformation.count(cameraPair) != 0)
            {
                comparison[poseName][cameraPair] = cv::abs(distances.second - comparisonDistanceInformation[cameraPair]);
            }
        }
    }
    analysisFilterResults.comparisonDistanceInformation = comparison;
    return comparison;
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::ComputeCameraOffsets(std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> & offsetsPerPose,
    std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, double>> & offsetPerCameraPair)
{
    // Loop through poses
    for (auto &obs : poseExtrinsicData)
    {
        POSE_NAME poseName = obs.first;
        auto &extrinsicData = obs.second;

        // Get the distance between all camera pairs that observed this pose
        std::map<CAMERA_NAME_PAIR, double> poseDistanceInformation;
        extrinsicData.ComputeMagnitudeOfDistanceBetweenAllCameraPairs(poseDistanceInformation);

        // Loop through camera pairs
        for (auto &distances : poseDistanceInformation)
        {
            // Store the results per pose
            const CAMERA_NAME_PAIR &cameraPair = distances.first;
            offsetsPerPose[poseName][cameraPair] = distances.second;

            // Store the results per camera pair
            std::pair<POSE_NAME, double> pose = { poseName, distances.second };
            offsetPerCameraPair[cameraPair].insert(pose);
        }
    }
    analysisFilterResults.absoluteDistancePerPoseInformation = offsetsPerPose;
    analysisFilterResults.absoluteDistancePerCameraPairInformation = offsetPerCameraPair;
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::CompareCameraPairBackProjected3dPoints(const MultiCameraIntrinsicData & multiCameraIntrinsicData, 
    std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, std::map<MARKER_IDENTIFIER, std::pair<cv::Point3d, cv::Point3d>>>> & backprojected3dPointPairs)
{
    // Loop through poses
    for (const auto & pose : this->poseObservationPoints)
    {
        // Get all the cameras that observed this pose
        POSE_NAME poseName = pose.first;
        auto names = poseExtrinsicData[poseName].GetCameraNames();

        // For each camera, compute the back-projected 3D points in that camera's coordinate system after rectifying the observations
        std::map<CAMERA_NAME, std::map<MARKER_IDENTIFIER, cv::Point3d>> corners3d;
        std::map<CAMERA_NAME, cv::Mat> boardToCameraRot, boardToCameraTrans;
        for (const auto & camera : pose.second)
            camera.second.BackprojectTo3d(multiCameraIntrinsicData.cameraData.at(camera.first), corners3d[camera.first], boardToCameraRot[camera.first], boardToCameraTrans[camera.first], true);

        // Loop through all pairs
        for (int i = 0; i < (int)names.size() - 1; i++)
        {
            for (int j = i + 1; j < (int)names.size(); j++)
            {
                // Define the camera pair
                std::pair<std::string, std::string> namepair = { *std::next(names.begin(), i), *std::next(names.begin(), j) };

                // Loop through markers for camera 1
                for (const auto & marker1 : corners3d[namepair.first])
                {
                    // Find this marker for camera 2
                    if (corners3d[namepair.second].count(marker1.first) > 0)
                    {
                        cv::Point3d marker2pt = corners3d[namepair.second][marker1.first];
                        cv::Mat marker2 = cv::Mat(3, 1, CV_64F);
                        marker2.at<double>(0, 0) = marker2pt.x;
                        marker2.at<double>(1, 0) = marker2pt.y;
                        marker2.at<double>(2, 0) = marker2pt.z;

                        // Transform this point from camera 2's frame to camera 1's frame
                        cv::Mat marker2InBoardFrame = boardToCameraRot[namepair.second].t() * (marker2 - boardToCameraTrans[namepair.second]);
                        cv::Mat marker2InCamera1Frame = boardToCameraRot[namepair.first] * marker2InBoardFrame + boardToCameraTrans[namepair.first];

                        // The pair of 3d points in camera 1's frame
                        std::pair<MARKER_IDENTIFIER, std::pair<cv::Point3d, cv::Point3d>> newpair = { marker1.first, { marker1.second, cv::Point3d(marker2InCamera1Frame.at<double>(0, 0), marker2InCamera1Frame.at<double>(1, 0), marker2InCamera1Frame.at<double>(2, 0)) } };

                        // Insert the new pair into the map of map of map
                        if (backprojected3dPointPairs.count(namepair) == 0)
                        {
                            // Create a new map indexed by pose
                            std::map<POSE_NAME, std::map<MARKER_IDENTIFIER, std::pair<cv::Point3d, cv::Point3d>>> posemap;
                            posemap[poseName].insert(newpair);
                            backprojected3dPointPairs[namepair] = posemap;
                        }
                        else if (backprojected3dPointPairs[namepair].count(poseName) == 0)
                        {
                            // Create a new map indexed by marker id
                            std::map<MARKER_IDENTIFIER, std::pair<cv::Point3d, cv::Point3d>> idmap;
                            idmap.insert(newpair);
                            backprojected3dPointPairs[namepair][poseName] = idmap;
                        }
                        else
                            backprojected3dPointPairs[namepair][poseName].insert(newpair);


                        //std::cout << namepair.first << ", " << namepair.second << ", " << marker1.second.x << ", " << marker1.second.y << ", " << marker1.second.z << ", " 
                        //    << marker2InCamera1Frame.at<double>(0, 0) << ", " << marker2InCamera1Frame.at<double>(1, 0) << ", " << marker2InCamera1Frame.at<double>(2, 0) << std::endl;
                    }
                } // End loop through markers
            }
        }
    } // End loop through poses
}

std::map<BoeingMetrology::POSE_NAME, std::map<BoeingMetrology::CAMERA_NAME_PAIR, cv::Point3d>> BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::ComputeRelativeCameraPositionVectors()
{
    // Declare output
    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, cv::Point3d>> positionVector;

    // Loop through poses
    for (auto &obs : poseExtrinsicData)
    {
        POSE_NAME poseName = obs.first;
        auto &extrinsicData = obs.second;

        // Get the distance between all camera pairs that observed this pose
        std::map<CAMERA_NAME_PAIR, cv::Point3d> posePositionVectorInformation;
        extrinsicData.ComputePositionVectorBetweenAllCameraPairs(posePositionVectorInformation);

        for (auto &distances : posePositionVectorInformation)
        {
            const CAMERA_NAME_PAIR &cameraPair = distances.first;
            positionVector[poseName][cameraPair] = distances.second;
        }
    }
    analysisFilterResults.positionVectorInformation = positionVector;
    return positionVector;
}

std::map<BoeingMetrology::POSE_NAME, bool> BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::filterByPoseTolerance(double tolerance, std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparison)
{
    std::map<POSE_NAME, bool> filter;
    for (auto &poseInfo : comparison)
    {
        POSE_NAME poseName = poseInfo.first;
        filter[poseName] = false;
        for (auto &distances : poseInfo.second)
        {
            const CAMERA_NAME_PAIR &cameraPair = distances.first;
            double distance = comparison[poseName][cameraPair];
            if (distance > tolerance)
            {
                filter[poseName] = true;
            }
        }
    }
    analysisFilterResults.filterByPoseTolerance = filter;
    analysisFilterResults.comparisonTolerance = tolerance;
    return filter;
}

std::map<BoeingMetrology::POSE_NAME, bool> BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::filterByPoseDeviation(double deviationTolerance, double absoluteTolerance, 
    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparison)
{
    std::map<POSE_NAME, bool> filter;
    std::vector<double> distanceDistribution = {};
    for (auto &poseInfo : comparison)
    {
        POSE_NAME poseName = poseInfo.first;
        filter[poseName] = false;
        double maxPoseDistance = 0.0;

        //throw out any distances greater than the outlier distance before analyzing remaining poses
        for (auto &distances : poseInfo.second)
        {
            const CAMERA_NAME_PAIR &camPair = distances.first;
            double distance = comparison[poseName][camPair];
            if (distance > cOutlierDistance || distance > absoluteTolerance)
            {
                filter[poseName] = true;
            }
            maxPoseDistance = std::max(maxPoseDistance, distance);
        }

        if (!filter[poseName])
        {
            distanceDistribution.push_back(maxPoseDistance);
        }
    }

    double avgDist = 0.0, stdDev = 0.0, medianDist, jenkCutoff;
    bool isSkewed = true;

    while (isSkewed)
    {
        ComputeDistributionStats(distanceDistribution, avgDist, stdDev, medianDist, isSkewed, jenkCutoff);
        if (isSkewed)
        {
            std::vector<double> tempDistribution = {};
            for (double distance : distanceDistribution)
            {
                if (distance <= jenkCutoff)
                {
                    tempDistribution.push_back(distance);
                }
            }
            distanceDistribution = tempDistribution;
        }
    }


    double upperCutoff = avgDist + deviationTolerance * stdDev;

    for (auto &poseInfo : comparison)
    {
        POSE_NAME poseName = poseInfo.first;
        filter[poseName] = false;

        for (auto &distances : poseInfo.second)
        {
            const CAMERA_NAME_PAIR &camPair = distances.first;
            double distance = comparison[poseName][camPair];
            if (distance > upperCutoff)
            {
                filter[poseName] = true;
            }

        }
    }

    analysisFilterResults.filterByPoseTolerance = filter;
    analysisFilterResults.comparisonTolerance = upperCutoff;
    return filter;
}

std::map<BoeingMetrology::POSE_NAME, std::map<BoeingMetrology::CAMERA_NAME_PAIR, bool>> BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::filterBytolerance(double tolerance, std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparison)
{
    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, bool>> filter;
    for (auto &poseInfo : comparison)
    {
        POSE_NAME poseName = poseInfo.first;

        for (auto &distances : poseInfo.second)
        {
            const CAMERA_NAME_PAIR &cameraPair = distances.first;
            double distance = comparison[poseName][cameraPair];
            filter[poseName][cameraPair] = (distance > tolerance);
        }
    }

    analysisFilterResults.filterBytolerance = filter;

    return filter;
}

std::map<BoeingMetrology::POSE_NAME, std::map<BoeingMetrology::CAMERA_NAME_PAIR, bool>> BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::filterByDeviation(double deviationTolerance, double absoluteTolerance, std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparison)
{
    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, bool>> filter;
    std::vector<double> distanceDistribution = {};
    for (auto &poseInfo : comparison)
    {
        POSE_NAME poseName = poseInfo.first;

        //throw out any distances greater than the outlier distance before analyzing remaining poses
        for (auto &distances : poseInfo.second)
        {
            const CAMERA_NAME_PAIR &camPair = distances.first;
            filter[poseName][camPair] = true;
            double distance = comparison[poseName][camPair];
            if (distance < cOutlierDistance && distance < absoluteTolerance)
            {
                filter[poseName][camPair] = false;
                distanceDistribution.push_back(distance);
            }

        }
    }

    double avgDist = 0.0, stdDev = 0.0, medianDist, jenkCutoff;
    bool isSkewed = true;

    while (isSkewed)
    {
        ComputeDistributionStats(distanceDistribution, avgDist, stdDev, medianDist, isSkewed, jenkCutoff);
        if (isSkewed)
        {
            std::vector<double> tempDistribution = {};
            for (double distance : distanceDistribution)
            {
                if (distance <= jenkCutoff)
                {
                    tempDistribution.push_back(distance);
                }
            }
            distanceDistribution = tempDistribution;
        }
    }


    double upperCutoff = avgDist + deviationTolerance * stdDev;

    for (auto &poseInfo : comparison)
    {
        POSE_NAME poseName = poseInfo.first;

        for (auto &distances : poseInfo.second)
        {
            const CAMERA_NAME_PAIR &camPair = distances.first;
            filter[poseName][camPair] = false;
            double distance = comparison[poseName][camPair];
            if (distance > upperCutoff)
            {
                filter[poseName][camPair] = true;
            }

        }
    }

    analysisFilterResults.filterBytolerance = filter;

    return filter;
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::filterObsByCameraPairOffset(const std::map<CAMERA_NAME_PAIR, std::map<POSE_NAME, double>> & offsetsPerCameraPair, 
    const size_t & minNumObs, const double & stdDevThresh, std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> & filter)
{
    // Loop through camera pairs
    for (const auto & campair : offsetsPerCameraPair)
    {
        // Values per pair
        std::vector<double> offsets;

        // Compute mean and std across poses for this camera pair
        for (const auto & pose : campair.second)
        {
            // Exclude already filtered camera-pose combos
            auto poseFilterIter = filter.find(pose.first);
            if (poseFilterIter != filter.end())
            {
                auto& poseFilter = poseFilterIter->second;

                // Create array of camera names to loop
                decltype(campair.first.first) cameraNames[2] =
                {
                    campair.first.first,
                    campair.first.second
                };

                // Loop over the camera names and see if they're filtered
                bool filtered = false;
                for (auto& cameraName : cameraNames)
                {
                    auto filteredCameraIter = poseFilter.find(cameraName);
                    filtered |= (filteredCameraIter != poseFilter.end() && filteredCameraIter->second);
                }
                if (filtered)
                {
                    continue;
                }
            }

            bool zeroOffset = (std::abs(pose.second) < 0.01);
            if (zeroOffset)
            {
                continue;
            }

            offsets.push_back(pose.second);
        }

        size_t numObs = offsets.size();

        double avgDist = 0.0, stdDev = 0.0;

        if (numObs >= minNumObs)
        {
            CameraCalibrationUtilities::ComputeDistributionStats(offsets, avgDist, stdDev);
            std::cout << "Camera Pair Offset Statistics: " << "camera 1"          << ", "  <<      "camera 2"     << ", " << "mean"  << ", " << "stdDev" << std::endl;
            std::cout << "Camera Pair Offset Statistics: " << campair.first.first << ", " << campair.first.second << ", " << avgDist << ", " << stdDev << std::endl;
        }
        else
        {
            std::cout << "Camera Pair Offset Statistics: " << campair.first.first << ", " << campair.first.second << " too few observations -> " << numObs << std::endl;
        }

        // Loop through poses
        for (const auto & pose : campair.second)
        {
            // Filter decision for cameras
            std::pair<CAMERA_NAME, bool> resultcam1 = { campair.first.first, false };
            std::pair<CAMERA_NAME, bool> resultcam2 = { campair.first.second, false };
            bool zeroOffset = (std::abs(pose.second) < 0.01);
            if (zeroOffset || (numObs >= minNumObs && std::abs(avgDist - pose.second) / stdDev > stdDevThresh))
            {
                resultcam1.second = true;
                resultcam2.second = true;
                if (!zeroOffset)
                    std::cout << "Camera Pair Offset Filter: " << campair.first.first << ", " << campair.first.second << " DISCARDED value = " << pose.second << ", " << pose.first << std::endl;
            }

            // If this camera is already filtered for this pose, we never unfilter it
            if (filter.count(pose.first) == 0)
            {
                filter[pose.first].insert(resultcam1);
                filter[pose.first].insert(resultcam2);
            }
            else
            {
                // Camera 1
                if (filter[pose.first].count(campair.first.first) == 0)
                    filter[pose.first].insert(resultcam1);
                else if (filter[pose.first][resultcam1.first] == false)
                    filter[pose.first][resultcam1.first] = resultcam1.second;

                // Camera 2
                if (filter[pose.first].count(campair.first.second) == 0)
                    filter[pose.first].insert(resultcam2);
                else if (filter[pose.first][resultcam2.first] == false)
                    filter[pose.first][resultcam2.first] = resultcam2.second;
            }
        }
    }
    this->analysisFilterResults.cameraOffsetObsFilter = filter;
}

void BoeingMetrology::Calibration::Pose::PoseGraphAnalyzer::JsonDeserialize(const Json::Value &jsonNode)
{
    throw std::runtime_error("PoseGraphAnalyzer::JsonDeserialize not yet implemented");
}
