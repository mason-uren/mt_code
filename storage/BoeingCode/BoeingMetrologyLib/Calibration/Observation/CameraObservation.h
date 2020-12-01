#ifndef BOEINGMETROLOGYLIB_CAMERAOBSERVATION_H
#define BOEINGMETROLOGYLIB_CAMERAOBSERVATION_H

#include <string>
#include <cv.h>
#include "ObservationPoints.h"
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "Calibration/IntrinsicData.h"
namespace BoeingMetrology
{
    namespace Calibration
    {
        namespace Observation
        {
            // Observed calibration points and corresponding known geometry of those points 
            // for a single camera and a single pose
            class BOEINGMETROLOGYLIB_API CameraObservation : public ObservationPoints<float>, public Boeing::Interface::Serializer
            {
            private:
                //Transformation of the calibration object
                cv::Matx34d transform;

                void SerializeStream(std::ostream &strm) const;
                void DeserializeStream(std::istream &strm);
            public:

                // Pose folder
                std::string fileNameOfObservation;

                // Reprojection error
                double rms = 0.0;

                // Set the 3x3 rotation matrix.
                // Throws exception on failure.
                void SetRotationMatrix(const cv::Mat & rotMat);

                // Set the translation vector.
                // Throws exception on failure.
                // Translation (position) components are in meters.  
                void SetTranslationVector(const cv::Mat & transMat);

                // Get the 3x3 rotation matrix (CV_64FC1)
                cv::Mat GetRotationMatrix() const;

                // Get the 3x1 translation vector (CV_64FC1)
                // Translation (position) components are in meters.  
                cv::Mat GetTranslationVector() const;

                // Get range to calibration object in meters
                double GetRangeToTarget() const;

                // Estimate 3D pose of board.  This is a transform from the calibration board's frame to the camera frame.  
                // t is a translation vector and rx, ry, rz are Euler angles in degrees based on the rotation order X then Y then Z.
                // Optionally, determine reprojection error per observation point.
                // Here we also sets this->transform
                double Estimate3dPoseEuler(const IntrinsicData & intrinsicData, double & rx, double & ry, double & rz, cv::Mat & t,
                    std::map<MARKER_IDENTIFIER, cv::Point2f> & outliers, const bool & computeRms = false);

				CameraObservation undistortObservations(IntrinsicData intrinsics);

                void SetRotationTranslation(cv::Matx34d transform_);
                void JsonSerialize(Json::Value &jsonNode) const override;
                void JsonDeserialize(const Json::Value &jsonValue) override;
                void SerializeFile(std::string fileName) const;
                void DeserializeFile(std::string fileName);
            };
        }
    }
}

#endif
