#ifndef BOEINGMETROLOGYLIB_OBSERVATIONPOINTS_H
#define BOEINGMETROLOGYLIB_OBSERVATIONPOINTS_H

#include <string>
#include <cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <map>
#include <fstream>
#include <string>
#include "Common/Interface/Serializer.h"
#include "json/writer.h"
#include "json/reader.h"
#include "Calibration/IntrinsicData.h"
#include "Common/Utilities/Math/Transformations.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"
#include "Calibration/CameraCalibrationUtilities.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        namespace Observation
        {
            template <class Type = float>
            // Observed calibration points and corresponding known geometry of those points 
            // for a single camera and a single pose
            class ObservationPoints // : public Boeing::Interface::Serializer
            {
            public:
                typedef cv::Point3_<Type> CONTROL_POINT;
                typedef cv::Point_<Type> OBSERVATION_POINT;

                std::string cameraName;
                cv::Size imageSize;

                //Points on the calibration object (relative to the 3d object and not relative to the world or camera frame)
                std::vector <CONTROL_POINT> controlPoints;

                //Points relative to the image plane (in pixels)
                std::vector <OBSERVATION_POINT> observedPoints;

                //Marker identifier to correlate across 
                std::vector<MARKER_IDENTIFIER> markerIdentifier;

                void InitializeObservations(const std::string& cameraName_, cv::Size imageSize_)
                {
                    this->cameraName = cameraName_;
                    this->imageSize = imageSize_;
                    this->controlPoints.clear();
                    this->observedPoints.clear();
                    this->markerIdentifier.clear();
                }

                void AddObservation(const MARKER_IDENTIFIER markId, const OBSERVATION_POINT &observedPoint, const CONTROL_POINT &controlPoint)
                {
                    markerIdentifier.push_back(markId);
                    controlPoints.push_back(controlPoint);
                    observedPoints.push_back(observedPoint);
                }

                void GetObservations(std::vector<MARKER_IDENTIFIER> &markerIdentifier_, std::vector <OBSERVATION_POINT> &observedPoints_, std::vector <CONTROL_POINT> &controlPoints_) const
                {
                    markerIdentifier_.clear();
                    controlPoints_.clear();
                    observedPoints_.clear();
                    markerIdentifier_ = markerIdentifier;
                    controlPoints_ = controlPoints;
                    observedPoints_ = observedPoints;
                }

                int ObservationCount() const
                {
                    //return the number of observations
                    return (int)controlPoints.size();
                }

                // Estimate 3D pose of board.  This is a transform from the calibration board's frame to the camera frame.  
                // t is a translation vector and rx, ry, rz are Euler angles in degrees based on the rotation order X then Y then Z.
                // Optionally, determine reprojection error per observation point.
                double Estimate3dPoseEuler(const IntrinsicData & intrinsicData, double & rx, double & ry, double & rz, cv::Mat & t, 
                    std::map<MARKER_IDENTIFIER, cv::Point2f> & outliers, const bool & computeRms = false) const
                {
                    // r is rotation matrix from board to camera
                    cv::Mat r;
                    double rms = this->Estimate3dPose(intrinsicData, r, t, outliers, computeRms);

                    // Compute Euler angles rx, ry, rz in degrees
                    Boeing::Utilities::Math::Transformations::euler_from_matrix<double>(r, "rxyz", rx, ry, rz);

                    return rms;
                }

                // Estimate 3D pose of board.  This is a transform from the calibration board's frame to the camera frame.  
                // t is a translation vector and r is a rotation vector -- see 
                // https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20Rodrigues(InputArray%20src,%20OutputArray%20dst,%20OutputArray%20jacobian)
                void Estimate3dPoseRodrigues(const IntrinsicData & intrinsicData, cv::Mat & r, cv::Mat & t) const
                {
                    if (this->controlPoints.size() > 4)
                    {
                        // Get translation vector and Rodrigues rotation vector
                        r = cv::Mat(3, 1, CV_64F);
                        t = cv::Mat(3, 1, CV_64F);
                        cv::solvePnP(this->controlPoints, this->observedPoints, intrinsicData.cameraMatrix, intrinsicData.distortionCoeffs, r, t);
                    }
                    else
                    {
                        throw std::runtime_error(
                            std::string("ObservationPoints::Estimate3dPoseRodrigues failed to estimate pose for camera '") + 
                            cameraName + 
                            "' - " +
                            std::to_string(controlPoints.size()) +
                            " control points exists, >4 are required.");
                    }
                }

                // Estimate 3D pose of board.  The output transform will take a point specified in the calibration board's frame as p_b and 
                // convert it into the camera frame via p_camera = r * p_b + t, where p_b and t are column vectors.  
                // In other words this is a transform from the calibration board's frame to the camera frame.  
                double Estimate3dPose(const IntrinsicData & intrinsicData, cv::Mat & r, cv::Mat & t, std::map<MARKER_IDENTIFIER, cv::Point2f> & outliers, const bool & computeRms = false) const
                {
                    double rms = -1.0;

                    if (this->controlPoints.size() > 4)
                    {
                        // Get translation vector and Rodrigues rotation vector
                        cv::Mat rtemp = cv::Mat(3, 1, CV_64F);
                        t = cv::Mat(3, 1, CV_64F);
                        cv::solvePnP(this->controlPoints, this->observedPoints, intrinsicData.cameraMatrix, intrinsicData.distortionCoeffs, rtemp, t);

                        if (computeRms)
                        {
                            // Reprojection error of each corner
                            std::vector<double> reprojectionError;

                            // Reproject control points back to image plane and compare observed image coordinates to reprojected points
                            std::vector<OBSERVATION_POINT> projectPts;
                            cv::projectPoints(this->controlPoints, rtemp, t, intrinsicData.cameraMatrix, intrinsicData.distortionCoeffs, projectPts);
                            for (size_t idx = 0; idx < this->observedPoints.size(); idx++)
                            {
                                reprojectionError.push_back(cv::norm(this->observedPoints[idx] - projectPts[idx]));
                            }

                            double std;
                            CameraCalibrationUtilities::ComputeDistributionStats(reprojectionError, rms, std);
                            size_t minNumMarkerForStats = 20;
                            double stdDevThresh = 3.0;
                            if (reprojectionError.size() > minNumMarkerForStats)
                            {
                                for (size_t idx = 0; idx < reprojectionError.size(); idx++)
                                {
                                    if (std::abs(rms - reprojectionError[idx]) / std > stdDevThresh)
                                    {
                                        outliers[this->markerIdentifier[idx]] = this->observedPoints[idx];
                                        //std::cout << "Marker Id " << this->markerIdentifier[idx] << " OUTLIER " << rms << ", " << std << ", " << reprojectionError[idx] << std::endl;
                                    }
                                }
                            }
                        }

                        // Convert from Rodrigues rotation vector to 3x3 dcm
                        r = cv::Mat(3, 3, CV_64F);
                        cv::Rodrigues(rtemp, r);
                    }
                    else
                    {
                        throw std::runtime_error(
                            std::string("ObservationPoints::Estimate3dPose failed to estimate pose for camera '") +
                            cameraName +
                            "' - " +
                            std::to_string(controlPoints.size()) +
                            " control points exists, >4 are required.");
                    }
                    return rms;
                }

                // Back-project observed corners to 3D based on intrinsics and known calibration board geometry.  Observations should be undistorted 
                // first if they haven't already been.
                void BackprojectTo3d(const IntrinsicData & intrinsicData, std::map<MARKER_IDENTIFIER, cv::Point3d> & corners3d, cv::Mat & boardToCameraRot, 
                    cv::Mat & boardToCameraTrans, const bool & undistort = false) const
                {
                    // Get transform from board to camera
                    boardToCameraRot = cv::Mat(3, 3, CV_64F);
                    boardToCameraTrans = cv::Mat(3, 1, CV_64F);
                    std::map<MARKER_IDENTIFIER, cv::Point2f> outliers;
                    this->Estimate3dPose(intrinsicData, boardToCameraRot, boardToCameraTrans, outliers, false);

                    // Perform undistortion if necessary
                    std::vector<OBSERVATION_POINT> undistortedPts = this->observedPoints;
                    if (undistort)
                        cv::undistortPoints(undistortedPts, undistortedPts, intrinsicData.cameraMatrix, intrinsicData.distortionCoeffs, cv::noArray(), intrinsicData.cameraMatrix);

                    // Loop through chessboard corners
                    for (size_t idx = 0; idx < undistortedPts.size(); idx++)
                    {
                        // Unproject the observed pixel coordinates to 3D
                        MARKER_IDENTIFIER id = this->markerIdentifier[idx];

                        cv::Point3d backprojected = intrinsicData.BackProjectOntoCalibrationBoard(boardToCameraRot, boardToCameraTrans, undistortedPts[idx], false);

                        // Update output
                        corners3d[id] = backprojected;
                    }
                }

                // Overlay the detected pixels onto an image
                void OverlayDetectionsOnImage(cv::Mat & image) const
                {
                    try
                    {
                        int radius = 25;
                        for (const auto & pt : this->observedPoints)
                            cv::circle(image, pt, radius, cv::Scalar(0, 0, 255), 6, cv::LINE_AA);
                    }
                    catch (...)
                    {
                        throw;
                    }
                }

                void JsonSerialize(Json::Value &jsonNode) const
                {
                    // Camera info
                    jsonNode["cameraName"] = cameraName;
                    jsonNode["ImageHeight"] = imageSize.height;
                    jsonNode["ImageWidth"] = imageSize.width;

                    //Generate the metadata with the observations
                    Json::Value cornerPoints = Json::Value(Json::arrayValue);
                    Json::Value actualPoints = Json::Value(Json::arrayValue);
                    for (int i = 0; i < observedPoints.size(); i++)
                    {
                        Json::Value cornerpoint;
                        cornerpoint["Num"] = i;
                        cornerpoint["observed_x"] = observedPoints[i].x; // image coordinates
                        cornerpoint["observed_y"] = observedPoints[i].y; // image coordinates
                        cornerpoint["id"] = markerIdentifier[i];
                        cornerpoint["control_x"] = controlPoints[i].x; // physical coordinates
                        cornerpoint["control_y"] = controlPoints[i].y; // physical coordinates

                        cornerPoints.append(cornerpoint);
                    }

                    jsonNode["observation_points"] = cornerPoints;
                }

                void JsonDeserialize(const Json::Value &jsonValue)
                {
                    try
                    {
                        std::string cName = jsonValue["cameraName"].asString();
                        int iWidth = jsonValue["ImageWidth"].asInt();
                        int iHeight = jsonValue["ImageHeight"].asInt();

                        this->InitializeObservations(cName, cv::Size(iWidth, iHeight));

                        for (const Json::Value & itm : jsonValue["observation_points"])
                        {
                            // int index = itm["Num"].asInt(); // this variable is not used
                            int markerId;
                            if (itm.isMember("Id"))
                                markerId = itm["Id"].asInt();
                            else
                                markerId = itm["id"].asInt();

                            double obsX = itm["observed_x"].asDouble();
                            double obsY = itm["observed_y"].asDouble();
                            double cntlX = itm["control_x"].asDouble();
                            double cntlY = itm["control_y"].asDouble();

                            // Get the actual control points in 3D
                            cv::Point3f controlPoint(static_cast<float>(cntlX), static_cast<float>(cntlY), 0);// = board->chessboardCorners[charucoId];

                            // Get the observed points in image coordinates
                            cv::Point2f observationPoint(static_cast<float>(obsX), static_cast<float>(obsY));
                            this->AddObservation(markerId, observationPoint, controlPoint);
                        }
                    }
                    catch (...)
                    {
                        throw;
                    }
                }

                void SerializeStream(std::ostream &strm) const
                {
                    // Camera info
                    Json::Value root;
                    JsonSerialize(root);
                    Json::StyledStreamWriter json_writer;
                    json_writer.write(strm, root);
                }

                void DeserializeStream(std::istream &strm)
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
                        throw std::runtime_error("ObservationPoints::DeserializeStream(): Failed to parse json.");
                    }
                }

                void SerializeFile(std::string fileName) const
                {
                    //Write the serialization to the file
                    std::ofstream strm(fileName);
                    SerializeStream(strm);
                    strm.close();
                }

                void DeserializeFile(std::string fileName)
                {
                    //Read the content from the file and de-serialize into the class
                    std::ifstream strm(fileName);
                    DeserializeStream(strm);
                    strm.close();
                }
            };
        }
    }
}

#ifdef BOEINGMETROLOGYLIB_EXPORT
template class BOEINGMETROLOGYLIB_API BoeingMetrology::Calibration::Observation::ObservationPoints<float>;
#else
extern template class BOEINGMETROLOGYLIB_API BoeingMetrology::Calibration::Observation::ObservationPoints<float>;
#endif

#endif
