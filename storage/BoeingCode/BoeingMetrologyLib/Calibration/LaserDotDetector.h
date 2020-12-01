#ifndef BOEINGMETROLOGYLIB_LASERDOTDETECTOR_H
#define BOEINGMETROLOGYLIB_LASERDOTDETECTOR_H

#include <cv.h>
#include "Calibration/CalibrationObject/ArucoBoard.h"
#include "Common/Interface/Serializer.h"
#include "Calibration/Observation/LaserDotObservationPoints.h"
#include "TypeDefs.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
        // Performs detection of a laser dot on a calibration board image
        class BOEINGMETROLOGYLIB_API  LaserDotDetector : public Boeing::Interface::Serializer
		{
		public:

            // Extract a laser dot with user help
            void detectRedLaserDotFromGrayscaleImageUserInput(const cv::Mat& inputImageDistorted, const cv::Ptr<cv::aruco::CharucoBoard>& board, const std::string & resultsFname,
                const Calibration::IntrinsicData & intrinsicData, Observation::LaserDotObservationPoints& laserDotObservationPoints, const float & intensityThreshold = 220.0f);

            // Extract a list of laser dot detections from a greyscale image after undistorting the image
            void detectRedLaserDotFromGrayscaleImage(const cv::Mat& inputImageDistorted, const cv::Ptr<cv::aruco::CharucoBoard>& board, const std::string & resultsFname,
                const Calibration::IntrinsicData & intrinsicData, Observation::LaserDotObservationPoints& laserDotObservationPoints, const float & intensityThreshold = 220.0f);

            // Extract a list of laser dot detections from a greyscale image without undistorting the image
            void detectRedLaserDotFromGrayscaleImage(const cv::Mat& inputImage, const cv::Ptr<cv::aruco::CharucoBoard>& board, const std::string & resultsFname,
                Observation::LaserDotObservationPoints& laserDotObservationPoints, const float & intensityThreshold = 220.0f);

            // Display image for user selection of detection
            void selectDotCircle(const std::vector<cv::Point2f> & initialDetections, Observation::LaserDotObservationPoints& laserDotObservationPoints);

            // Event callback for processing mouse clicks
            static void  selectDotCircleCallback(int event, int x, int y, int flags, void* userdata);

            // Unproject a pixel coordinate back onto the calibration board in the camera's 3D coordinate frame
            static void BackprojectLaserDotsTo3D(const Observation::LaserDotObservationPoints & laserDotObservationPoints, const IntrinsicData & intrinsicData,
                const std::vector<cv::Point3f> & cameraObjectPoints, const std::vector<cv::Point2f> & cameraImagePoints, std::map<POSE_NAME, std::pair<cv::Point2f, cv::Point3f>> & laserDot3dCoords);

            // Best-fit line to 3D points.  Output is a mean position and directional unit vector. 
            static void BestFit3dLine(const std::vector<cv::Point3d> & pts, std::pair<cv::Point3d, cv::Point3d> & meanDir);

            // Best fit a line to the 3d points and project back to estimate origin of ray given depth values
            static double BestFit3dLineAndEstimateOrigin(const std::pair<std::vector<cv::Point3d>, std::vector<double>> & pts, std::pair<cv::Point3d, cv::Point3d> & posDir);

            virtual void JsonSerialize(Json::Value &jsonNode) const override;
            virtual void JsonDeserialize(const Json::Value &jsonNode) override;

		private:
			void preProcessImageForRedDot(const cv::Mat& inputImage, cv::Mat& outputImage);

			void getHoughCircles(const cv::Mat& inputImage, std::vector<cv::Vec3f>& circles);				

		};

        struct MouseCallbackParams {
            std::vector<cv::Point2f> circlesBoard;
            float circleRadius;
            float scaleX;
            float scaleY;
            int selIndx = -1;
            cv::Point2i pixelCoord;
        };
	}
}

#endif

