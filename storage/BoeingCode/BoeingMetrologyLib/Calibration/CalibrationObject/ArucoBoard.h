#ifndef BOEINGMETROLOGYLIB_ARUCOBOARD_H
#define BOEINGMETROLOGYLIB_ARUCOBOARD_H

#include <string>
#include "AbstractObjectBase.h"
#include <cv.h>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include "json/writer.h"
#include "Calibration/Observation/CameraObservation.h"
#include "Calibration/Observation/ObservationAngles.h"
#include <iostream>

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        using namespace Observation;
		namespace CalibrationObject
		{
			// Encapsulate a cv::aruco::CharucoBoard object
			class BOEINGMETROLOGYLIB_API ArucoBoard : public AbstractObjectBase
			{
				friend class ArucoCirclesBoard;

                // For digital board, the width of the image in pixels
				int imageWidth_;

                // For digital board, the height of the image in pixels
				int imageHeight_;

                // For digital board, the padded white board around the chessboard, specified in pixels
                int borderWidth_;

                // For a digital board, this is the size of a pixel in physical units (meters).  A value >0 indicates digital board with this pixel size.
                double pixelSizeInPhysicalUnits_;

                // For a digital board, this is the derived square size in pixels.  For a physical board, this is assigned -1. 
                int squareSizeInPixels_;

                // The number of chessboard squares horizontally
				int checkerboardSquareX_;

                // The number of chessboard squares vertically
				int checkerboardSquareY_;

                // The width of each chessboard square in physical units (meters).  For a digital board, this is derived.
				float squareLength_;

                // The width of each aruco marker square in physical units (meters).  For a digital board, this is derived. 
				float markerLength_;

                // The unique Aruco marker pattern
                cv::aruco::PREDEFINED_DICTIONARY_NAME markerPattern_;

                // The underlying OpenCV data structure associated with the marker pattern
                cv::Ptr<cv::aruco::Dictionary> arucoDict_;

                void SerializeStream(std::ostream &strm) const;

                void DeserializeStream(std::istream &strm);

			public:

				// Pointer to the opencv representation of the board
				cv::Ptr<cv::aruco::CharucoBoard> board_;

				// Constructor
				ArucoBoard();

                // For a digital board, derive the square size in pixels
                static int CharucoBoardSquareInPixels(const cv::Size outSize, int marginSize,
                    int chessboardSquaresX, int chessboardSquaresY, float squareLength, float markerLength);

				// Determine if we have a valid aruco board
				bool isValid() const;

                // Get squareLength_
                float GetSquareLength() const
                {
                    return this->squareLength_;
                }

                double getPhysicalSquareSizeOfDigitalBoard() const;

				std::vector<cv::Point3f> GetControls();

				// Take in image, perform aruco detection, and output an image mask for the aruco board
                void GenerateArucoBoardMask(const cv::Mat& image, cv::Mat& mask, cv::Mat& overlaidImage) const;

				// Create a virtual image of the board.  Used to project onto a flat surface or display on a TV.  
				void ObjectGenerator(cv::Mat &matOut) override;

                // Compute a window size scale factor based on analysis performed in LIVPLN-600, in which we found
                // that corner refinement works well with default parameters for 640x480 data, but not for 
                // megapixel resolution
                static double ComputeCornerRefinementWinSizePixelScaleFactor(const cv::Size & imgSize);

				// Detect chessboard corners.  A minimum number of corners may be required to call it a detection.
				virtual void Detector(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
					const std::string &cameraName, ObservationPoints<float> &observationPts,
					cv::Mat &targetImageMask, bool doFilter = false, float filterCutoff = 10) override;

                // Detect chessboard corners with adaptive windowing
                void DetectorAdaptiveWindowing(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
                    const std::string &cameraName, ObservationPoints<float> &observationPts,
                    cv::Mat &targetImageMask, cv::Mat & markerCornerOverlay, bool doFilter = false, float filterCutoff = 10) const;

                // Detect chessboard corners, perform pose estimation, and store results.  Returns true on successful detection.
                bool DetectCornersAndEstimatePose(const cv::Mat &inputImage, const std::string &cameraName, const Calibration::IntrinsicData & intrinsicData, 
                    const int & minNumCorners, const double & maxReprojThresh, const std::string & accumulatedImageDir, const std::string & poseDir, 
                    CameraObservation &observationPts, cv::Mat &accumulatedImageMask, cv::Mat & markerCornerOverlay, 
                    Calibration::Observation::ObservationAngles & observationAngles) const;

				virtual void FilterMarkers(ObservationPoints<float> &observationPts, cv::Mat srcImage, cv::Mat &lineImg, float maxResidualPerPoint) const;

                virtual void MarkerSanityCheck(const cv::Mat srcImage, std::vector<cv::Point2f> observations, int sanityWindowSize,
                    bool &badMarkersFound, std::vector<cv::Point2f> &badMarkers, cv::Mat &badMarkerImage) const;
				
                virtual void MarkerSanityCheckHarris(const cv::Mat srcImage, std::vector<cv::Point2f> observations, int sanityWindowSize,
                    bool &badMarkersFound, std::vector<cv::Point2f> &badMarkers, cv::Mat &badMarkerImage, std::vector<cv::Point2f> &refinedMarkers) const;

				virtual void adjustLine(cv::Vec4f &toAdjust, std::vector<cv::Point2f> members, cv::Vec2f fitSlope);

				static void getCharucoBoardExtremePoints(const cv::Mat& image, const cv::Ptr<cv::aruco::CharucoBoard>& board, std::vector<float>& extremePoints);

                // Populate the output Json from this object's data.
                // Throws exception on failure.
                virtual void JsonSerialize(Json::Value &jsonNode) const override;

                // Populate this object from the input stream.
                // Throws exception on failure.
                virtual void JsonDeserialize(const Json::Value &jsonNode) override;

                // Write this class's contents to a json-formatted text file
                virtual void SerializeFile(const std::string &fileName) const override;

                // Read a json-formatted text file into this class
                virtual void DeserializeFile(const std::string &fileName) override;


				void setBoardParameters(
                    int imageWidth, int imageHeight, 
                    int checkerboardSquareX, int checkerboardSquareY,
					int borderWidth, float squareLength, float markerLength,
					cv::aruco::PREDEFINED_DICTIONARY_NAME markerPattern,
                    float pixelSizeInPhysicalUnits = -1 /* >0 indicates digital board */);

			};//class ArucoBoard
		}//namespace CalibrationObject
	}//namespace Calibration
}//namespace BoeingMetrology

#endif
