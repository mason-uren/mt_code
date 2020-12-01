#include "MultiPoseObservations.h"
#include "Calibration/CameraCalibrationUtilities.h"
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <string>
#include "Calibration/CalibrationObject/AbstractObjectBase.h"
#include "Utilities/Utilities.h"
#include <sys/stat.h>
#include "TypeDefs.h"
#include "Calibration/CameraCalibrationUtilities.h"
#include <thread>
#include <mutex>

#include <map>
#include <cstdlib>
#include <cstdio>
#if defined(_WIN32)
#include <direct.h>
#else
#include <dirent.h>
#endif

using namespace BoeingMetrology::Calibration;

namespace BoeingMetrology
{
namespace Calibration
{
namespace Observation
{

void MultiPoseObservations::AddObservations(const std::string & cameraName, const std::map<POSE_NAME, CameraObservation> & observations)
{
    // Loop through poses
    int poseCount = 0;
    for (const auto & pose : observations)
    {
        POSE_NAME poseName = pose.first;

        this->cameraObservationCount[cameraName]++;

        this->multiCameraPose[poseName].AddCameraPose(cameraName, pose.second);

        if (this->poseMappingToInt.find(poseName) == this->poseMappingToInt.end())
            this->poseMappingToInt[poseName] = poseCount++;

        this->multiCameraPose[poseName].poseIdentifier = poseName;
        this->multiCameraPose[poseName].poseId = this->poseMappingToInt[poseName];

        //this size gets set many times more than needed - oh well...
        this->cameraImageSize[cameraName] = pose.second.imageSize;
        poseCount++;
    }
}

void MultiPoseObservations::ReadObservationData(const std::string &baseFolder, const int numRequiredDetectedMarkers,
    Scanning::Observation::MultiLensStateCameraObservations & multiLensStateCameraObservations, std::set<CAMERA_NAME> & failedCameras,
    AbstractObjectBase *detector /*= NULL*/, std::map<CAMERA_NAME, IntrinsicData> intrinsicMap)
{
    // Clean up and store pose dir name
    CameraCalibrationUtilities::CleanFileName(baseFolder, this->folderNameOfObservations);
    multiLensStateCameraObservations.poseDir = this->folderNameOfObservations;

    // Determine all the pose directory info
    std::map<std::string, std::vector<std::tuple<std::string, std::string, std::string>>> cameraFiles;
    std::vector<std::string> poseDirs;

    //Loops through PNG files in the directory structure
    BoeingMetrology::CameraCalibrationUtilities::getPoseDirectoriesAndFiles(this->folderNameOfObservations, poseDirs, cameraFiles);

    // We will launch a thread for each camera
    std::vector<std::thread*> threads;
    std::recursive_mutex dataLock;
    int poseCount = 0;

    // Loop over cameras
    for (const auto &cameraKVP : cameraFiles)
    {
        // Launch a thread for this camera
        threads.push_back(new std::thread([this, cameraKVP, &multiLensStateCameraObservations, &detector, numRequiredDetectedMarkers, &poseCount, &failedCameras, &dataLock, &intrinsicMap]()
        {
            try
            {
                std::string cameraName = cameraKVP.first;

                // Pose angles for this camera
                Calibration::Observation::ObservationAngles observationAngles;

                // Coverage results across poses
                cv::Mat accumulatedCoverageMask;

                // Loop through poses for this camera             
                for (const auto &pose : cameraKVP.second)
                {
                    // Info for this pose
                    std::string cameraFile = std::get<0>(pose);
                    std::string poseName = std::get<1>(pose);
                    std::string charucoJsonFileName = std::get<2>(pose);

                    struct stat buffer;
                    bool detectionJsonExists = stat(charucoJsonFileName.c_str(), &buffer) == 0;

                    CameraObservation camPose;
                    //Read the data from the JSON
                    if (detectionJsonExists)
                    {
                        std::ifstream fReader(charucoJsonFileName, std::ifstream::binary);
                        
                        camPose.Deserialize(fReader);
                        camPose.fileNameOfObservation = cameraFile;

                        //check a subset of the control points and ensure they match with the detector if it exists
                        if (detector != NULL)
                        {
                            if (ArucoBoard *srcBoard = dynamic_cast<ArucoBoard*>(detector))
                            {
                                std::vector<cv::Point3f> boardControls = srcBoard->GetControls();
                                for (int i = 0; i < camPose.observedPoints.size(); i += 5)
                                {
                                    int marker = camPose.markerIdentifier[i];

                                    cv::Point3f observationControl = camPose.controlPoints[i];

                                    cv::Point3f boardControl = boardControls[marker];

                                    float xdif = abs(observationControl.x - boardControl.x);
                                    float ydif = abs(observationControl.y - boardControl.y);


                                    if (xdif + ydif > 0.001)
                                    {
                                        std::cout << "Observed controls in pose file do NOT match the control points produced by the provided arucoboard" << std::endl;
                                        std::cout << "Check that the observation point JSONS match expected and that the provided arucoboard format is correct" << std::endl;
                                        throw std::runtime_error("observation and board mismatch");
                                    }

                                }
                            }
                        }

                        
                    }
                    else
                    {
                        std::cout << "MultiPoseObservations::ReadObservationData: Detection results missing for " << charucoJsonFileName << std::endl;

                        if (detector != NULL)
                        {
                            if (ArucoBoard *srcBoard = dynamic_cast<ArucoBoard*>(detector))
                            {
                                cv::Mat camImage = cv::imread(cameraFile);

                                camPose.fileNameOfObservation = cameraFile;
                                camPose.cameraName = cameraName;
                                camPose.imageSize = camImage.size();

                                cv::Mat targetMask, detections;
                                bool targetDetected = false;
                                srcBoard->DetectorAdaptiveWindowing(camImage, numRequiredDetectedMarkers, targetDetected, cameraName, camPose, targetMask, detections);

                                if (!targetDetected)
                                {
                                    throw std::runtime_error("Target not detected in image");
                                }

                                if (intrinsicMap.count(cameraName) != 0)
                                {
                                    double rx, ry, rz;
                                    cv::Mat t;
                                    std::map<MARKER_IDENTIFIER, cv::Point2f> outliers;
                                    camPose.Estimate3dPoseEuler(intrinsicMap[cameraName], rx, ry, rz, t, outliers, true);
                                    camPose.SerializeFile(charucoJsonFileName);
                                    std::string detectionFile = cameraFile.substr(0, cameraFile.size() - 4) + "markerCorners.jpeg";
                                    cv::imwrite(detectionFile, detections);
                                    poseCount++;
                                }
                                else
                                {
                                    throw std::runtime_error("Intrinsics missing for detection");
                                }
                            }
                        }


                    }
                        
                    //If the number of points collected is positive...
                    if (camPose.ObservationCount() > 0)
                    {
                        std::lock_guard<std::recursive_mutex> locker(dataLock);
                        this->cameraObservationCount[cameraName]++;

                        this->multiCameraPose[poseName].AddCameraPose(cameraName, camPose);

                        if (this->poseMappingToInt.find(poseName) == this->poseMappingToInt.end())
                            this->poseMappingToInt[poseName] = poseCount++;

                        this->multiCameraPose[poseName].poseIdentifier = poseName;
                        this->multiCameraPose[poseName].poseId = this->poseMappingToInt[poseName];

                        //this size gets set many times more than needed - oh well...
                        this->cameraImageSize[cameraName] = camPose.imageSize;

                        // Load lens state info if it exists
                        std::string lensStateJsonFilename = CameraCalibrationUtilities::ReplaceSubString(charucoJsonFileName, "ChArUco", "");
                        lensStateJsonFilename = CameraCalibrationUtilities::ReplaceSubString(lensStateJsonFilename, ".json", "_lensState.json");
                        if (stat(lensStateJsonFilename.c_str(), &buffer) == 0)
                        {
                            Scanning::Configuration::LensState lensState;
                            lensState.DeserializeFile(lensStateJsonFilename);
                            multiLensStateCameraObservations.AddObservation(cameraName, lensState, poseName, camPose);
                        }
                        poseCount++;
                    }


                } // End loop over poses
            }
            catch (std::exception e)
            {
                std::lock_guard<std::recursive_mutex> locker(dataLock);
                std::cout << e.what() << std::endl;
                std::cout << "MultiPoseObservations::ReadObservationData: Failed to load pose observations for " << cameraKVP.first << std::endl;
                failedCameras.insert(cameraKVP.first);
            }
        })); // End lambda for this camera
    } // End camera loop

    // Wait for threads to finish
    std::for_each(threads.begin(), threads.end(), [](std::thread* x){ x->join(); });

    // Free threads
    for (unsigned iter = 0; iter < threads.size(); iter++) delete threads[iter];

}//void MultiPoseObservations::ReadObservationData

void MultiPoseObservations::FilterMultiCameraObservationByCamera(const POSE_NAME &poseName,
    MultiCameraObservation &cameraPose, const CAMERA_REGEX_STRING &cameraRegexString)
{
    for (auto &camera : cameraPose.cameraObservation)
    {
        CAMERA_NAME cameraName = camera.first;
        if (cameraRegexString == "" || BoeingMetrology::Utilities::RegexMatch(cameraRegexString, cameraName))
        {
            cameraObservationCount[cameraName]++;
            FilterCameraObservationByCamera(poseName, cameraName, camera.second, cameraRegexString);
        }
    }
}//void MultiPoseObservations::FilterMultiCameraObservationByCamera

void MultiPoseObservations::FilterCameraObservationByCamera(const POSE_NAME &poseName,
    const CAMERA_NAME &cameraName, CameraObservation &cameraPose,
    const CAMERA_REGEX_STRING &cameraRegexString)
{
    CAMERA_NAME name_ = cameraName;
    multiCameraPose[poseName].AddCameraPose(name_, cameraPose);

    //If the number of points collected is positive...
    if (cameraPose.ObservationCount() > 0)
    {
        if (poseMappingToInt.find(poseName) == poseMappingToInt.end())
            poseMappingToInt[poseName] = (int)poseMappingToInt.size();

        multiCameraPose[poseName].poseIdentifier = poseName;
        multiCameraPose[poseName].poseId = poseMappingToInt[poseName];

        //this size gets set many times more than needed - oh well...
        cameraImageSize[cameraName] = cameraPose.imageSize;
    }
}//void MultiPoseObservations::FilterCameraObservationByCamera

void MultiPoseObservations::FilterMultiCameraObservationByDistanceTolerance(const POSE_NAME &poseName,
    MultiCameraObservation &cameraPose,
    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparisonDistanceInformation,
    const double tolerance)
{
    for (auto &camera : cameraPose.cameraObservation)
    {
        CAMERA_NAME cameraName = camera.first;

        if (FilterCameraObservationByDistanceTolerance(poseName, cameraName, cameraPose.GetCameraPose(cameraName), comparisonDistanceInformation, tolerance))
        {
            cameraObservationCount[cameraName]++;
        }
    }
}//void MultiPoseObservations::FilterMultiCameraObservationByDistanceTolerance


bool MultiPoseObservations::FilterCameraObservationByDistanceTolerance(const POSE_NAME &poseName,
    const CAMERA_NAME &cameraName, CameraObservation &cameraPose,
    std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparisonDistanceInformation,
    const double tolerance)
{
    std::map < CAMERA_NAME_PAIR, double > &pairDistance = comparisonDistanceInformation[poseName];
    bool inTolerance = false;
    for (auto &pair : pairDistance)
    {
        CAMERA_NAME_PAIR pairNames = pair.first;
        if (pair.second < tolerance && (pairNames.first == cameraName || pairNames.second == cameraName))
            inTolerance = true;
    }

    if (inTolerance)
    {
        CAMERA_NAME name_ = cameraName;
        multiCameraPose[poseName].AddCameraPose(name_, cameraPose);

        //If the number of points collected is positive...
        if (cameraPose.ObservationCount() > 0)
        {
            if (poseMappingToInt.find(poseName) == poseMappingToInt.end())
                poseMappingToInt[poseName] = (int)poseMappingToInt.size();

            multiCameraPose[poseName].poseIdentifier = poseName;
            multiCameraPose[poseName].poseId = poseMappingToInt[poseName];

            //this size gets set many times more than needed - oh well...
            cameraImageSize[cameraName] = cameraPose.imageSize;
        }
        return true;
    }
    else
        return false;
}//bool MultiPoseObservations::FilterCameraObservationByDistanceTolerance

void MultiPoseObservations::RetrieveCameraObservationPoints(const std::string &cameraName, std::vector<ObservationPoints<float>> & observationPoints)
{
    observationPoints.clear();

    // Loop through poses
    for (auto &multiCamPose : multiCameraPose)
    {
        POSE_NAME poseName = multiCamPose.first;

        if (multiCamPose.second.contains(cameraName))
        {
            // Data for this camera and pose
            observationPoints.push_back(multiCamPose.second.GetCameraPose(cameraName));
        }
    }
}//void MultiPoseObservations::RetrieveCameraObservationPoints

void MultiPoseObservations::RetrieveCameraObservationPoints(const std::string &cameraName,
    std::vector<std::vector<int>> &cameraMarkerIds, std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
    std::vector<std::vector<cv::Point2f>> &cameraImagePoints, std::vector<std::vector<int>> *cameraPoseIndex /*= nullptr*/,
    const std::map<POSE_NAME, bool> * cameraPoseFilter /*= nullptr*/)
{
    // Loop through poses
    for (auto &multiCamPose : multiCameraPose)
    {
        POSE_NAME poseName = multiCamPose.first;

        // Check if this pose has been rejected (true = reject)
        if (cameraPoseFilter == nullptr || (cameraPoseFilter != nullptr && cameraPoseFilter->find(poseName) != cameraPoseFilter->end() && !cameraPoseFilter->at(poseName)))
        {
            if (multiCamPose.second.contains(cameraName))
            {
                CameraObservation &cameraPose = multiCamPose.second.GetCameraPose(cameraName);

                std::vector <ObservationPoints<float>::CONTROL_POINT> controlPoints;
                //Points relative to the image plane (in pixels)
                std::vector <ObservationPoints<float>::OBSERVATION_POINT> observedPoints;
                //Marker identifier to correlate across
                std::vector<int> markerIdentifier;
                cameraPose.GetObservations(markerIdentifier, observedPoints, controlPoints);
                cameraMarkerIds.push_back(markerIdentifier);
                cameraObjectPoints.push_back(controlPoints);
                cameraImagePoints.push_back(observedPoints);

                if (cameraPoseIndex != nullptr)
                {
                    //Generate parallel pose index constant for this pose
                    std::vector <int> poseValues;
                    for (int count = 0; count < controlPoints.size(); ++count)
                    {
                        poseValues.push_back(this->poseMappingToInt[poseName]);
                    }
                    cameraPoseIndex->push_back(poseValues);
                }
            }
        }
    }
}//void MultiPoseObservations::RetrieveCameraObservationPoints

void MultiPoseObservations::RetrieveCameraObservationPointsandFiles(const std::string &cameraName,
    std::vector<std::vector<int>> &cameraMarkerIds, std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
    std::vector<std::vector<cv::Point2f>> &cameraImagePoints, std::vector<std::string>& poseImageFiles,
    std::vector<std::vector<int>> *cameraPoseIndex /*= nullptr*/, const std::map<POSE_NAME, bool> * cameraPoseFilter /*= nullptr*/) const
{
    // Loop through poses
    for (const auto &multiCamPose : multiCameraPose)
    {
        POSE_NAME poseName = multiCamPose.first;

        // Check if this pose has been rejected (true = reject)
        if (cameraPoseFilter == nullptr || (cameraPoseFilter != nullptr && cameraPoseFilter->find(poseName) != cameraPoseFilter->end() && !cameraPoseFilter->at(poseName)))
        {
            if (multiCamPose.second.contains(cameraName))
            {
                CameraObservation cameraPose = multiCamPose.second.GetCameraPoseCopy(cameraName);

                std::vector <ObservationPoints<float>::CONTROL_POINT> controlPoints;
                //Points relative to the image plane (in pixels)
                std::vector <ObservationPoints<float>::OBSERVATION_POINT> observedPoints;
                //Marker identifier to correlate across
                std::vector<int> markerIdentifier;
                std::vector<std::string> pose;

                pose.clear();

                cameraPose.GetObservations(markerIdentifier, observedPoints, controlPoints);
                cameraMarkerIds.push_back(markerIdentifier);
                cameraObjectPoints.push_back(controlPoints);
                cameraImagePoints.push_back(observedPoints);

                poseImageFiles.push_back(cameraPose.fileNameOfObservation);


                if (cameraPoseIndex != nullptr)
                {
                    //Generate parallel pose index constant for this pose
                    std::vector <int> poseValues;
                    for (int count = 0; count < controlPoints.size(); ++count)
                    {
                        poseValues.push_back(this->poseMappingToInt.at(poseName));

                    }
                    cameraPoseIndex->push_back(poseValues);
                }
            }
        }
    }
}//void MultiPoseObservations::RetrieveCameraObservationPointsandFiles

void MultiPoseObservations::RetrieveCameraObservationPointsandPoseNames(const std::string &cameraName,
    std::vector<std::vector<int>> &cameraMarkerIds, std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
    std::vector<std::vector<cv::Point2f>> &cameraImagePoints, std::vector<std::string>& poseNames,
    std::vector<std::vector<int>> *cameraPoseIndex /*= nullptr*/, const std::map<POSE_NAME, bool> * cameraPoseFilter /*= nullptr*/, 
    const std::map<POSE_NAME, std::map<CAMERA_NAME, bool>> * cameraObsFilter  /*= nullptr */) const
{
    // Loop through poses
    for (const auto &multiCamPose : multiCameraPose)
    {
        POSE_NAME poseName = multiCamPose.first;

        // Check if this pose has been rejected (true = reject)
        bool rejectedPose = false;
        if (cameraPoseFilter != nullptr)
        {
            const auto poseit = cameraPoseFilter->find(poseName);
            if (poseit != cameraPoseFilter->end() && cameraPoseFilter->at(poseName))
                rejectedPose = true;
        }
        if (rejectedPose)
        {
            std::cout << "RetrieveCameraObservationPointsandPoseNames: Rejecting " << poseName << std::endl;
            continue;
        }

        if (multiCamPose.second.contains(cameraName))
        {
            // Check if this camera has been rejected for this pose (true = reject)
            bool rejected = false;
            if (cameraObsFilter != nullptr)
            {
                const auto poseit = cameraObsFilter->find(poseName);
                if (poseit != cameraObsFilter->end())
                {
                    const auto camit = poseit->second.find(cameraName);
                    if (camit != poseit->second.end() && camit->second)
                        rejected = true;
                }
            }
            if (rejected)
            {
                std::cout << "RetrieveCameraObservationPointsandPoseNames: Rejecting " << poseName << ", " << cameraName << std::endl;
                continue;
            }

            CameraObservation cameraPose = multiCamPose.second.GetCameraPoseCopy(cameraName);

            std::vector <ObservationPoints<float>::CONTROL_POINT> controlPoints;
            //Points relative to the image plane (in pixels)
            std::vector <ObservationPoints<float>::OBSERVATION_POINT> observedPoints;
            //Marker identifier to correlate across
            std::vector<int> markerIdentifier;
            std::vector<std::string> pose;

            pose.clear();

            cameraPose.GetObservations(markerIdentifier, observedPoints, controlPoints);
            cameraMarkerIds.push_back(markerIdentifier);
            cameraObjectPoints.push_back(controlPoints);
            cameraImagePoints.push_back(observedPoints);

            // Strip the filename
            std::string dir, fname;
            CameraCalibrationUtilities::FileParts(cameraPose.fileNameOfObservation, dir, fname);
            poseNames.push_back(dir);

            if (cameraPoseIndex != nullptr)
            {
                //Generate parallel pose index constant for this pose
                std::vector <int> poseValues;
                for (int count = 0; count < controlPoints.size(); ++count)
                {
                    poseValues.push_back(this->poseMappingToInt.at(poseName));

                }
                cameraPoseIndex->push_back(poseValues);
            }
        }
    }
}//void MultiPoseObservations::RetrieveCameraObservationPointsandFiles

void MultiPoseObservations::RetrieveCameraObservationPointsandPoseNames(const std::string &cameraName, std::map<POSE_NAME, std::pair<std::vector<cv::Point3f>, std::vector<cv::Point2f>>> & observationsPerPose)
{
    // Loop through poses
    for (const auto &multiCamPose : multiCameraPose)
    {
        if (multiCamPose.second.contains(cameraName))
        {
            CameraObservation cameraPose = multiCamPose.second.GetCameraPoseCopy(cameraName);

            std::vector <ObservationPoints<float>::CONTROL_POINT> controlPoints;
            //Points relative to the image plane (in pixels)
            std::vector <ObservationPoints<float>::OBSERVATION_POINT> observedPoints;
            //Marker identifier to correlate across 
            std::vector<int> markerIdentifier;
            cameraPose.GetObservations(markerIdentifier, observedPoints, controlPoints);

            // Strip the filename
            std::string dir, fname;
            CameraCalibrationUtilities::FileParts(cameraPose.fileNameOfObservation, dir, fname);

            observationsPerPose[dir] = { controlPoints, observedPoints };
        }
    }
}

void MultiPoseObservations::RetrieveMutualObservationPoints(const std::vector<std::string> & camNames, std::vector<std::vector<cv::Point3f>> & cameraObjectPoints, 
    std::map<std::string, std::vector<std::vector<cv::Point2f>>> & cameraImagePoints, std::vector<std::string> & poseNames)
{
    if (camNames.size() != 2)
        throw std::runtime_error("RetrieveMutualObservationPoints: camera names should be of length 2");

    cameraObjectPoints.clear();
    cameraImagePoints.clear();

    // Loop through multi-camera pose objects
    for (auto &multiCamPose : multiCameraPose)
    {
        // Proceed if both cameras share this pose
        if (multiCamPose.second.contains(camNames[0]) && multiCamPose.second.contains(camNames[1]))
        {
            std::vector<ObservationPoints<float>::CONTROL_POINT> mutualControlPoints;
            std::map<std::string, std::vector<ObservationPoints<float>::OBSERVATION_POINT>> mutualObservedPoints;

            // Get the marker ids for the first camera
            CameraObservation &cameraPose = multiCamPose.second.GetCameraPose(camNames[0]);
            std::map<std::string, std::vector <ObservationPoints<float>::CONTROL_POINT>> controlPoints;
            std::map<std::string, std::vector <ObservationPoints<float>::OBSERVATION_POINT>> observedPoints;
            std::map<std::string, std::vector<int>> markerIdentifier;
            cameraPose.GetObservations(markerIdentifier[camNames[0]], observedPoints[camNames[0]], controlPoints[camNames[0]]);

            // Get the marker ids for the second camera
            cameraPose = multiCamPose.second.GetCameraPose(camNames[1]);
            cameraPose.GetObservations(markerIdentifier[camNames[1]], observedPoints[camNames[1]], controlPoints[camNames[1]]);

            // Find the mutually observed marker ids and record the data
            for (int cam1idx = 0; cam1idx < markerIdentifier[camNames[0]].size(); cam1idx++)
            {
                auto it = std::find(markerIdentifier[camNames[1]].begin(), markerIdentifier[camNames[1]].end(), markerIdentifier[camNames[0]][cam1idx]);
                if (it != markerIdentifier[camNames[1]].end())
                {
                    // Mutually observed
                    size_t cam2idx = std::distance(markerIdentifier[camNames[1]].begin(), it);
                    mutualControlPoints.push_back(controlPoints[camNames[1]][cam2idx]);
                    mutualObservedPoints[camNames[0]].push_back(observedPoints[camNames[0]][cam1idx]);
                    mutualObservedPoints[camNames[1]].push_back(observedPoints[camNames[1]][cam2idx]);
                }
            }

            if (mutualControlPoints.size() > 3)
            {
                // Get the pose name
                POSE_NAME poseName = multiCamPose.first;

                // We have at least four common markers for this pose. Append to the pose vector.
                poseNames.push_back(poseName);
                cameraObjectPoints.push_back(mutualControlPoints);
                cameraImagePoints[camNames[0]].push_back(mutualObservedPoints[camNames[0]]);
                cameraImagePoints[camNames[1]].push_back(mutualObservedPoints[camNames[1]]);
            }
        }
    }
}//void MultiPoseObservations::RetrieveMutualObservationPoints

std::vector<std::string> MultiPoseObservations::GetCameraNames() const
{
    std::vector<std::string> names;
    for (const auto & namepair : cameraObservationCount)
        names.push_back(namepair.first);
    return names;
}

void MultiPoseObservations::FilteredObservationsByCamera(MultiPoseObservations &obj, const CAMERA_REGEX_STRING &cameraRegexString)
{
    for (auto &observation : multiCameraPose)
    {
        POSE_NAME name = observation.first;
        obj.FilterMultiCameraObservationByCamera(name, observation.second, cameraRegexString);
    }
}//void MultiPoseObservations::FilteredObservationsByCamera

void MultiPoseObservations::FilteredObservationsByDistanceTolerance(MultiPoseObservations &obj, std::map<POSE_NAME, std::map<CAMERA_NAME_PAIR, double>> &comparisonDistanceInformation, const double tolerance)
{
    for (auto &observation : multiCameraPose)
    {
        POSE_NAME name = observation.first;
        obj.FilterMultiCameraObservationByDistanceTolerance(name, observation.second, comparisonDistanceInformation, tolerance);
    }
}//void MultiPoseObservations::FilteredObservationsByDistanceTolerance

void MultiPoseObservations::FilterObservationsBasedOnReproj(std::vector<std::vector<cv::Point3f>> & cameraObjectPoints,
    std::vector<std::vector<cv::Point2f>> & cameraImagePoints, const cv::Size & imageSize, cv::Matx33d cameraMatrix,
    cv::Mat distortionCoeffs, const int & flags, const double & reprojThreshold, const int & minimumPointsPerView,
    std::vector<std::vector<int>> *poseIds, std::vector<std::string> * poseNames, std::vector<std::vector<int>> * charucoIds)
{
	// Get reprojection error per pose (view)
	cv::Mat stdDevIntrinsics, stdDevExtrinsics;
	std::vector<double> perView;
	std::vector<cv::Mat> rVectors, tVectors;
	try
	{
		calibrateCamera(cameraObjectPoints, cameraImagePoints, imageSize,
			cameraMatrix, distortionCoeffs, rVectors, tVectors, stdDevIntrinsics, stdDevExtrinsics, perView, flags);
	}
	catch (cv::Exception & e)
	{
		std::cout << "MultiPoseObservations::FilterObservationsBasedOnReprojError: " << e.what() << std::endl;
		throw e;
	}

    // Erase bad poses
    std::vector<int> badPoseIdx;
    for (int idx = 0; idx < (int)perView.size(); idx++)
    {
        if (cameraImagePoints[idx].size() < minimumPointsPerView)
        {
            std::cout << "Pose " << idx << " error - numImagePoints " << cameraImagePoints[idx].size() << " < mininum " << minimumPointsPerView << std::endl;
            badPoseIdx.push_back(idx);
        }
        else
        {
            if (perView[idx] > reprojThreshold)
            {
                std::cout << "Pose " << idx << " error - reproj error " << perView[idx] << " > treshold " << reprojThreshold << std::endl;
                badPoseIdx.push_back(idx);
            }
        }
    }
    for (int idx = (int)badPoseIdx.size() - 1; idx >= 0; idx--)
    {
        cameraObjectPoints.erase(cameraObjectPoints.begin() + badPoseIdx[idx]);
        cameraImagePoints.erase(cameraImagePoints.begin() + badPoseIdx[idx]);
        if (poseIds != nullptr)
            poseIds->erase(poseIds->begin() + badPoseIdx[idx]);
        if (poseNames != nullptr)
            poseNames->erase(poseNames->begin() + badPoseIdx[idx]);
        if (charucoIds != nullptr)
            charucoIds->erase(charucoIds->begin() + badPoseIdx[idx]);
    }
}//void MultiPoseObservations::FilterObservationsBasedOnReproj

void MultiPoseObservations::JsonSerialize(Json::Value &jsonNode) const
{
    try
    {
        //Generate the metadata with the observations
        Json::Value poseArray;
        for (const auto &multiCP : this->multiCameraPose)
        {
            Json::Value poseNode;
            multiCP.second.JsonSerialize(poseNode["pose"]);
            poseNode["poseName"] = multiCP.first;
            poseArray.append(poseNode);
        }
        jsonNode["poseArray"] = poseArray;
        jsonNode["poseDirectory"] = this->folderNameOfObservations;
    }
    catch (...)
    {
        throw;
    }
}//void MultiPoseObservations::JsonSerialize

void MultiPoseObservations::JsonDeserialize(const Json::Value &jsonNode)
{
    multiCameraPose.clear();
    cameraObservationCount.clear();
    cameraImageSize.clear();
    poseMappingToInt.clear();

    try
    {
        this->folderNameOfObservations = jsonNode["poseDirectory"].asString();
        int count = 0;
        for (const Json::Value & pose : jsonNode["poseArray"])
        {
            MultiCameraObservation multiCP;
            multiCP.JsonDeserialize(pose["pose"]);
            this->multiCameraPose[pose["poseName"].asString()] = multiCP;
            poseMappingToInt[multiCP.poseIdentifier] = count++;

            for (auto &cameraObs : multiCP.cameraObservation)
            {
                CAMERA_NAME name = cameraObs.first;
                cameraObservationCount[name]++;
                cameraImageSize[name] = cameraObs.second.imageSize;
            }
        }
    }
    catch (...)
    {
        throw;
    }
}//void MultiPoseObservations::JsonDeserialize

void MultiPoseObservations::SerializeStream(std::ostream &strm) const
{
    // Camera info
    Json::Value root;
    JsonSerialize(root);
    Json::StyledStreamWriter json_writer;
    json_writer.write(strm, root);
}

void MultiPoseObservations::DeserializeStream(std::istream &strm)
{
    Json::Reader jsonReader;
    Json::Value root;
    //Read through the JSON file and collect the points
    if (jsonReader.parse(strm, root))
    {
        JsonDeserialize(root);
    }
    else
    {
        throw std::runtime_error("MultiPoseObservations::DeserializeStream(): Failed to parse json.");
    }
}

void MultiPoseObservations::SerializeFile(std::string fileName) const
{
    //Write the serialization to the file
    std::ofstream strm(fileName);
    SerializeStream(strm);
    strm.close();
}

void MultiPoseObservations::DeserializeFile(std::string fileName)
{
    //Read the content from the file and de-serialize into the class
    std::ifstream strm(fileName);
    DeserializeStream(strm);
    strm.close();
}


}//namespace Observation
}//namespace Calibration
}//namespace BoeingMetrology

