#include "MultiPoseGraph.h"
#include <queue>
#include "json/value.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "Calibration/CameraCalibrationUtilities.h"
#include <iostream>
#include <thread>
#include <mutex>

using namespace BoeingMetrology::Calibration::Observation;
void BoeingMetrology::Calibration::Pose::MultiPoseGraph::initialPoseEstimator(MultiPoseObservations & observations, 
    MultiCameraIntrinsicData & multiCameraIntrinsicData, const double & reprojThreshold, const int & minimumPointsPerView, 
    const std::map<POSE_NAME, bool> * cameraPoseFilter /*= nullptr */, const std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> * cameraObsFilter  /*= nullptr */)
{
    this->poseDataPath = observations.folderNameOfObservations;
    std::recursive_mutex _dataLock;

    _nCamera = (int)observations.cameraObservationCount.size();

    if (_nCamera < 2)
    {
        std::cout << "MultiPoseGraph::initialPoseEstimator ERROR: Must have at least 2 cameras" << std::endl;
        return;
    }

    // Clear existing data
    this->Clear();


    /*
    Multi-threaded: Get 3D estimates of each pose
    */

    std::vector<std::thread*> threads(_nCamera);
    int threadCount = 0;
    std::vector<std::string> failedCameras;

    int flag = CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_FOCAL_LENGTH +
        CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_K3;

    int cameraIdx = 0;
    for (const auto & camera : observations.cameraObservationCount)
    {
        std::string cameraName = camera.first;
        _vertexList.push_back(vertex(cameraName));
        if (cameraIdx > 0)
            this->newVertices.push_back(cameraIdx);
        cameraIdx++;
    }

    // Loop through cameras
    cameraIdx = 0;
    for (const auto & camera : observations.cameraObservationCount)
    {
        std::string cameraName = camera.first;
        std::vector<std::vector<cv::Point3f>> cameraObjectPointsInitial;
        std::vector<std::vector<cv::Point2f>> cameraImagePointsInitial;
        std::vector<std::vector<int>> charucoIdsInitial;
        std::vector<std::vector<int>> poseIdsInitial;
        std::vector<std::string> poseNamesInitial;

        observations.RetrieveCameraObservationPointsandPoseNames(cameraName, charucoIdsInitial,
            cameraObjectPointsInitial, cameraImagePointsInitial, poseNamesInitial, &poseIdsInitial, cameraPoseFilter, cameraObsFilter);
        cv::Size imgSize = observations.cameraImageSize.at(cameraName);

        // Launch thread
        threads[threadCount++] = new std::thread(
            [this, cameraName, cameraObjectPointsInitial, cameraImagePointsInitial, imgSize, poseIdsInitial, poseNamesInitial,
            &failedCameras, &multiCameraIntrinsicData, &_dataLock, flag, reprojThreshold, minimumPointsPerView, charucoIdsInitial]()->bool
        {
            try
            {
                populateInitialPoseGraph(cameraName, imgSize, multiCameraIntrinsicData, _dataLock,
                    cameraObjectPointsInitial, cameraImagePointsInitial, poseIdsInitial, poseNamesInitial, charucoIdsInitial, failedCameras, flag, reprojThreshold, minimumPointsPerView);
                return true;
            }
            catch (std::exception & e)
            {
                std::cout << e.what() << std::endl;
                return false;
            }
        }); // End lambda per camera
        cameraIdx++;
    } // End loop through cameras

    // Wait for threads to finish
    std::for_each(threads.begin(), threads.end(), [](std::thread* x){ x->join(); });

    // Free threads
    for (unsigned iter = 0; iter < threads.size(); iter++) delete threads[iter];

    if (failedCameras.size() != 0)
    {
        std::cout << "ERROR: MultiPoseGraph::initialPoseEstimator(): The following cameras failed initialization" << std::endl;
        for (const auto & cam : failedCameras)
            std::cout << cam << " " << std::endl;
        return;
    }
}

void BoeingMetrology::Calibration::Pose::MultiPoseGraph::addMultiPoseObservations(const MultiPoseObservations & observations, 
    MultiCameraIntrinsicData & multiCameraIntrinsicData, const double & reprojThreshold, const int & minimumPointsPerView, const int & flag, 
    const std::map<POSE_NAME, bool> * cameraPoseFilter /*= nullptr */, const std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> * cameraObsFilter /*= nullptr */)
{
    // Get the list of new cameras
    std::vector<std::string> newNames;
    for (const auto & camera : observations.cameraObservationCount)
    {
        CAMERA_NAME cameraName = camera.first;
        if (!this->CameraExists(cameraName))
            newNames.push_back(cameraName);
    }
    
    // Initialize some variables
    std::recursive_mutex dataLock;
    std::vector<std::thread*> threads(newNames.size());
    int threadCount = 0;

    // Loop through new cameras
    for (const auto & cameraName : newNames)
    {
        // Retrieve all observations for this new camera
        std::vector<std::vector<cv::Point3f>> cameraObjectPointsInitial;
        std::vector<std::vector<cv::Point2f>> cameraImagePointsInitial;
        std::vector<std::vector<int>> charucoIdsInitial;
        std::vector<std::vector<int>> poseIdsInitial;
        std::vector<std::string> poseNamesInitial;
        observations.RetrieveCameraObservationPointsandPoseNames(cameraName, charucoIdsInitial,
            cameraObjectPointsInitial, cameraImagePointsInitial, poseNamesInitial, &poseIdsInitial, cameraPoseFilter, cameraObsFilter);
        cv::Size imgSize = observations.cameraImageSize.at(cameraName);

        // Launch thread
        threads[threadCount++] = new std::thread(
            [this, cameraName, cameraObjectPointsInitial, cameraImagePointsInitial, imgSize, poseIdsInitial, poseNamesInitial,
            &multiCameraIntrinsicData, &dataLock, &newNames, reprojThreshold, minimumPointsPerView, flag, charucoIdsInitial]()->bool
        {
            try
            {
                if (cameraImagePointsInitial.size() == 0)
                    throw std::runtime_error("All poses were filtered out!");

                cv::Mat cameraMatrix = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
                cv::Mat distortionCoeffs = cv::Mat::zeros(cv::Size(5, 1), CV_64F);
                {
                    // Store results in map indexed by camera name
                    std::lock_guard<std::recursive_mutex> locker(dataLock);

                    if (multiCameraIntrinsicData.cameraData.find(cameraName) == multiCameraIntrinsicData.cameraData.end())
                    {
                        std::cout << "addMultiPoseObservations: Failed to find intrinsics for " + cameraName << std::endl;
                        return false;
                    }

                    // Deep copy temporary data, since cv::calibrateCamera will modify the intrinsics
                    cameraMatrix = cv::Mat(multiCameraIntrinsicData.cameraData.at(cameraName).cameraMatrix).clone();
                    distortionCoeffs = multiCameraIntrinsicData.cameraData.at(cameraName).distortionCoeffs.clone();
                }

                // Filter observations by reprojection error
                std::vector<std::vector<cv::Point3f>> cameraObjectPoints = cameraObjectPointsInitial;
                std::vector<std::vector<cv::Point2f>> cameraImagePoints = cameraImagePointsInitial;
                std::vector<std::vector<int>> poseIds = poseIdsInitial;
                std::vector<std::string> poseNames = poseNamesInitial;
                std::vector<std::vector<int>> charucoIds = charucoIdsInitial;
                MultiPoseObservations::FilterObservationsBasedOnReproj(cameraObjectPoints, cameraImagePoints, imgSize,
                    cameraMatrix, distortionCoeffs, flag, reprojThreshold, minimumPointsPerView, &poseIds, &poseNames, &charucoIds);

                if (cameraObjectPoints.size() == 0)
                    throw std::runtime_error("All poses were filtered out!");

                // Deep copy intrinsics yet again
                {
                    // Store results in map indexed by camera name
                    std::lock_guard<std::recursive_mutex> locker(dataLock);

                    // Deep copy temporary data, since cv::calibrateCamera will modify the intrinsics
                    cameraMatrix = cv::Mat(multiCameraIntrinsicData.cameraData.at(cameraName).cameraMatrix).clone();
                    distortionCoeffs = multiCameraIntrinsicData.cameraData.at(cameraName).distortionCoeffs.clone();
                }

                // Perform pose estimation on remaining poses
                std::vector<cv::Mat> rotMat(poseIds.size()), transMat(poseIds.size());
                cv::calibrateCamera(cameraObjectPoints, cameraImagePoints, imgSize,
                    cameraMatrix, distortionCoeffs, rotMat, transMat, flag, _criteria);

                int cameraVertexIdx = -1;
                if (poseNames.size() > 0)
                {
                    // Add this camera to the pose graph
                    std::lock_guard<std::recursive_mutex> locker(dataLock);
                    this->AllocateNewCamera(cameraName, multiCameraIntrinsicData.cameraData.at(cameraName), cameraVertexIdx);
                }

                // Add vertices and edges for these poses
                this->AddPoseVertices(poseIds, rotMat, transMat, poseNames, cameraName, dataLock, cameraObjectPoints, cameraImagePoints, charucoIds);
                std::cout << "addMultiPoseObservations added " << poseIds.size() << " poses for camera " << cameraName << std::endl;

                return true;
            }
            catch (cv::Exception  & e)
            {
                std::cout << "addMultiPoseObservations failed for " << cameraName << e.what() << std::endl;
                return false;
            }
            catch (std::exception  & e)
            {
                std::cout << "addMultiPoseObservations failed for " << cameraName << e.what() << std::endl;
                return false;
            }
        }); // End lambda per camera
    } // End loop through cameras

    // Wait for threads to finish
    std::for_each(threads.begin(), threads.end(), [](std::thread* x){ x->join(); });

    // Free threads
    for (unsigned iter = 0; iter < threads.size(); iter++) delete threads[iter];
}
