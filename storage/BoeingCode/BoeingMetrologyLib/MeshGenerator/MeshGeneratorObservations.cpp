#include "MeshGeneratorObservations.h"
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

            void MeshGeneratorObservations::ReadObservationData(const std::string &observationFolder,
                const std::string &illuminatedFolder,
                const std::string &darkFolder,
                Scanning::Observation::MultiLensStateCameraObservations & multiLensStateCameraObservations,
                std::set<CAMERA_NAME> & failedCameras,
                AbstractObjectBase *detector)
            {
                // Clean up and store pose dir name
                CameraCalibrationUtilities::CleanFileName(observationFolder, this->folderNameOfObservations);
                CameraCalibrationUtilities::CleanFileName(illuminatedFolder, this->illuminatedDir);
                CameraCalibrationUtilities::CleanFileName(darkFolder, this->darkDir);

                multiLensStateCameraObservations.poseDir = this->folderNameOfObservations;

                // Determine all the pose directory info
                std::map<std::string, std::vector<std::tuple<std::string, std::string, std::string>>> cameraFiles;
                std::vector<std::string> poseDirs;
                std::map<std::string, std::vector<std::tuple<std::string, std::string, std::string>>> lightFiles;
                std::vector<std::string> lightPoseDirs;
                std::map<std::string, std::vector<std::tuple<std::string, std::string, std::string>>> darkFiles;
                std::vector<std::string> darkPoseDirs;

                //Loops through PNG files in the directory structure
                BoeingMetrology::CameraCalibrationUtilities::getPoseDirectoriesAndFiles(this->folderNameOfObservations, poseDirs, cameraFiles);
                BoeingMetrology::CameraCalibrationUtilities::getPoseDirectoriesAndFiles(this->illuminatedDir, lightPoseDirs, lightFiles);
                BoeingMetrology::CameraCalibrationUtilities::getPoseDirectoriesAndFiles(this->darkDir, darkPoseDirs, darkFiles);

                // We will launch a thread for each camera
                std::vector<std::thread*> threads;
                std::recursive_mutex dataLock;
                int poseCount = 0;

                // Loop over cameras
                for (const auto &cameraKVP : cameraFiles)
                {
                    // Launch a thread for this camera
                    threads.push_back(new std::thread([this, cameraKVP, &multiLensStateCameraObservations, &detector, &poseCount, &failedCameras, &dataLock, &darkFiles, &lightFiles]()
                    {
                        try
                        {
                            std::string cameraName = cameraKVP.first;

                            // Pose angles for this camera
                            Calibration::Observation::ObservationAngles observationAngles;

                            // Coverage results across poses
                            cv::Mat accumulatedCoverageMask;

                            std::vector<std::tuple<std::string, std::string, std::string>> darkCamFiles = darkFiles[cameraName];
                            std::vector<std::tuple<std::string, std::string, std::string>> lightCamFiles = lightFiles[cameraName];

                            for (int i = 0; i < cameraKVP.second.size(); ++i)
                            {
                                std::tuple<std::string, std::string, std::string> pose = cameraKVP.second[i];
                                std::tuple<std::string, std::string, std::string> darkPose = darkCamFiles[i];
                                std::tuple<std::string, std::string, std::string> lightPose = lightCamFiles[i];

                                std::string cameraFile = std::get<0>(pose);
                                std::string poseName = std::get<1>(pose);
                                std::string charucoJsonFileName = std::get<2>(pose);

                                std::string darkCameraFile = std::get<0>(darkPose);
                                std::string darkPoseName = std::get<1>(darkPose);

                                std::string lightCameraFile = std::get<0>(lightPose);
                                std::string lightPoseName = std::get<1>(lightPose);

                                /*if (darkPoseName != poseName || lightPoseName != poseName)
                                {
                                    std::cout << "There is not a 1-1 mapping between observation poses, light poses, and dark poses" << std::endl;
                                    std::cout << "Observation pose " << poseName << " : Dark pose " << darkPoseName << " : Light pose " << lightPoseName << std::endl;
                                    throw std::runtime_error("Improper mapping between poses");
                                }
*/
                                CameraObservation camPose;

                                struct stat buffer;
                                bool detectionJsonExists = stat(charucoJsonFileName.c_str(), &buffer) == 0;
                                //Read the data from the JSON
                                if (detectionJsonExists)
                                {
                                    std::ifstream fReader(charucoJsonFileName, std::ifstream::binary);

                                    camPose.Deserialize(fReader);
                                    camPose.fileNameOfObservation = cameraFile;

                                }
                                else
                                {
                                    std::cout << "MultiPoseObservations::ReadObservationData: Detection results missing for " << charucoJsonFileName << std::endl;

                                    if (detector != NULL)
                                    {
                                        if (ArucoBoard *srcBoard = dynamic_cast<ArucoBoard*>(detector))
                                        {
                                            std::cout << "Camfile " << cameraFile << " lightCameraFile " << lightCameraFile << " darkCameraFile " << darkCameraFile << std::endl;
                                            cv::Mat camImage = cv::imread(cameraFile);
                                            cv::Mat lightImage = cv::imread(lightCameraFile);
                                            cv::Mat darkImage = cv::imread(darkCameraFile);



                                            cv::cvtColor(camImage, camImage, cv::COLOR_BGR2GRAY);
                                            cv::cvtColor(lightImage, lightImage, cv::COLOR_BGR2GRAY);
                                            cv::cvtColor(darkImage, darkImage, cv::COLOR_BGR2GRAY);

                                            cv::Mat lightDif = lightImage - darkImage;

                                            cv::Mat lightDifThresh;

                                            cv::threshold(lightDif, lightDifThresh, 2, 1, CV_THRESH_BINARY);

                                            cv::Scalar difPixels = cv::sum(cv::sum(lightDifThresh));

                                            //if the illuminated image is not significantly different than the unilluminated image
                                            //then ignore the camera observations
                                            if (difPixels[0] < lightDif.cols * lightDif.rows / 10)
                                            {
                                                continue;
                                            }

                                            cv::Mat camDif = cv::Mat::zeros(camImage.size(), CV_8UC1);

                                            for (int y = 0; y < camDif.rows; ++y)
                                            {
                                                for (int x = 0; x < camDif.cols; ++x)
                                                {
                                                    if (camImage.at<uchar>(y, x) > darkImage.at<uchar>(y, x))
                                                    {
                                                        camDif.at<uchar>(y, x) = camImage.at<uchar>(y, x) - darkImage.at<uchar>(y, x);
                                                    }
                                                }
                                            }
                                            camDif.convertTo(camDif, CV_32FC1);
                                            lightDif.convertTo(lightDif, CV_32FC1);

                                            cv::Mat normCamDif = cv::Mat::zeros(camImage.size(), CV_32FC1);

                                            float *normptr = normCamDif.ptr<float>(0), *camptr = camDif.ptr<float>(0), *lightptr = lightDif.ptr<float>(0);
                                            for (int idx = 0; idx < camDif.rows * camDif.cols; ++idx)
                                            {
                                                normptr[idx] = camptr[idx] / lightptr[idx];
                                            }

                                            cv::threshold(normCamDif, normCamDif, 0.75, 1.0, CV_THRESH_BINARY);

                                            /*cv::namedWindow("unmorphed", CV_WINDOW_NORMAL);
                                            cv::imshow("unmorphed", normCamDif);*/


                                            normCamDif.convertTo(camDif, CV_8UC1, 255);


                                            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

                                            cv::morphologyEx(camDif, camDif, cv::MORPH_OPEN, kernel);
                                            cv::morphologyEx(camDif, camDif, cv::MORPH_CLOSE, kernel);

                                            /*cv::namedWindow("camDif", CV_WINDOW_NORMAL);
                                            cv::imshow("camDif", camDif);*/


                                            camPose.fileNameOfObservation = cameraFile;
                                            camPose.cameraName = cameraName;
                                            camPose.imageSize = camImage.size();

                                            std::cout << "detecting observations " << std::endl;

                                            cv::Mat targetMask, detections;
                                            bool targetDetected = false;
                                            srcBoard->DetectorAdaptiveWindowing(camDif, 4, targetDetected, cameraName, camPose, targetMask, detections);


                                            std::cout << "finished mesh detectors" << std::endl;

                                            /*cv::namedWindow("detections", CV_WINDOW_NORMAL);
                                            cv::imshow("detections", detections);

                                            char k = cv::waitKey();*/


                                            /*if (!targetDetected)
                                            {
                                                throw std::runtime_error("Target not detected in image");
                                            }*/

                                            camPose.SerializeFile(charucoJsonFileName);
                                            std::string detectionFile = cameraFile.substr(0, cameraFile.size() - 4) + "markerCorners.jpeg";
                                            cv::imwrite(detectionFile, detections);
                                            poseCount++;

                                            std::cout << "Serialized detections " << std::endl;
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
                                    /*std::string lensStateJsonFilename = CameraCalibrationUtilities::ReplaceSubString(charucoJsonFileName, "ChArUco", "");
                                    lensStateJsonFilename = CameraCalibrationUtilities::ReplaceSubString(lensStateJsonFilename, ".json", "_lensState.json");
                                    if (stat(lensStateJsonFilename.c_str(), &buffer) == 0)
                                    {
                                        Scanning::Configuration::LensState lensState;
                                        lensState.DeserializeFile(lensStateJsonFilename);
                                        multiLensStateCameraObservations.AddObservation(cameraName, lensState, poseName, camPose);
                                    }*/
                                    poseCount++;
                                }

                            }// End loop over poses
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


        }//namespace Observation
    }//namespace Calibration
}//namespace BoeingMetrology

