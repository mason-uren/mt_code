#include "SinglePoseGraph.h"
#include <queue>
#include "json/value.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "Calibration/CameraCalibrationUtilities.h"
#include <iostream>
#include <thread>
#include <mutex>
#include "Calibration/Observation/MultiCameraObservation.h"

using namespace BoeingMetrology::Calibration::Observation;

void BoeingMetrology::Calibration::Pose::SinglePoseGraph::initialPoseEstimator(MultiCameraObservation & observations, 
    MultiCameraIntrinsicData & multiCameraIntrinsicData, const double & reprojThreshold, const int & minimumPointsPerView)
{
    std::recursive_mutex _dataLock;

    this->poseDataPath = observations.folderNameOfObservation;

    _nCamera = (int)observations.cameraObservation.size();

    if (_nCamera < 2)
    {
        std::cout << "SinglePoseGraph::initialPoseEstimator ERROR: Must have at least 2 cameras" << std::endl;
        return;
    }

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
    for (const auto & camera : observations.cameraObservation)
    {
        std::string cameraName = camera.first;
        _vertexList.push_back(vertex(cameraName));
        if (cameraIdx > 0)
            this->newVertices.push_back(cameraIdx);
        cameraIdx++;
    }

    // Loop through cameras
    cameraIdx = 0;
    for (const auto & camera : observations.cameraObservation)
    {
        std::string cameraName = camera.first;
        std::vector<std::vector<cv::Point3f>> cameraObjectPointsInitial;
        std::vector<std::vector<cv::Point2f>> cameraImagePointsInitial;
        std::vector<std::vector<int>> charucoIdsInitial;
        std::vector<std::pair<std::vector<int>, std::string>> poseIdsInitialPair;

        cv::Size imgSize;
        observations.RetrieveCameraObservationPoints(cameraName, charucoIdsInitial,
            cameraObjectPointsInitial, cameraImagePointsInitial, imgSize, &poseIdsInitialPair);

        std::vector<std::vector<int>> poseIdsInitial;
        std::vector<std::string> poseNamesInitial;
        for (const auto & p : poseIdsInitialPair)
        {
            // Extract the pose name
            std::string dir, fname;
            CameraCalibrationUtilities::FileParts(p.second, dir, fname);
            poseNamesInitial.push_back(dir);

            // Store the duplicate pose IDs
            std::vector<int> idsThisPose;
            for (const auto & id : p.first)
                idsThisPose.push_back(id);
            poseIdsInitial.push_back(idsThisPose);
        }

        // Launch thread
        threads[threadCount++] = new std::thread(
            [this, cameraIdx, cameraName, cameraObjectPointsInitial, cameraImagePointsInitial, imgSize, poseIdsInitial, poseNamesInitial, charucoIdsInitial,
            &failedCameras, &multiCameraIntrinsicData, &_dataLock, flag, reprojThreshold, minimumPointsPerView]()
        {
            populateInitialPoseGraph(cameraName, imgSize, multiCameraIntrinsicData, _dataLock,
                cameraObjectPointsInitial, cameraImagePointsInitial, poseIdsInitial, poseNamesInitial, charucoIdsInitial, failedCameras, flag, reprojThreshold, minimumPointsPerView, false);
        }); // End lambda per camera

        cameraIdx++;
    } // End loop through cameras

    // Wait for threads to finish
    std::for_each(threads.begin(), threads.end(), [](std::thread* x){ x->join(); });

    // Free threads
    for (unsigned iter = 0; iter < threads.size(); iter++) delete threads[iter];

    if (failedCameras.size() != 0)
    {
        //std::cout << "SinglePoseGraph::initialPoseEstimator(): The following cameras failed initialization" << std::endl;
        //for (const auto & cam : failedCameras)
        //    std::cout << cam << " " << std::endl;
        return;
    }
}


