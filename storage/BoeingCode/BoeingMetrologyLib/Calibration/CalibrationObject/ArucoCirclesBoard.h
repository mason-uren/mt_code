#ifndef BOEINGMETROLOGYLIB_ARUCOCIRCLES_H
#define BOEINGMETROLOGYLIB_ARUCOCIRCLES_H

#include <string>
#include "AbstractObjectBase.h"
#include <cv.h>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include "json/writer.h"
#include "Calibration/Observation/ObservationPoints.h"
#include <iostream>
#include "ArucoBoard.h"

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		using namespace Observation;
		namespace CalibrationObject
		{
			// Encapsulate a cv::aruco::CharucoBoard object
			class BOEINGMETROLOGYLIB_API ArucoCirclesBoard : public AbstractObjectBase
			{
				

				static int ArucoCirclesBoardInPixels(const cv::Size outSize, int marginSize,
					int chessboardSquaresX, int chessboardSquaresY, float squareLength);

			public:
				ArucoBoard arucoFormat_;

				float circleDiam_;

				// Constructor
				ArucoCirclesBoard();

				ArucoCirclesBoard(ArucoBoard srcBoard, float circleDiameter);

				// Determine if we have a valid aruco board
				bool isValid() const;

				// Get squareLength_
				float GetSquareLength() const
				{
					return this->arucoFormat_.GetSquareLength();
				}

				// Create a virtual image of the board.  Used to project onto a flat surface or display on a TV.  
				void ObjectGenerator(cv::Mat &matOut) override;

				// Compute a window size scale factor based on analysis performed in LIVPLN-600, in which we found
				// that corner refinement works well with default parameters for 640x480 data, but not for 
				// megapixel resolution
				//static double ComputeCornerRefinementWinSizePixelScaleFactor(const cv::Size & imgSize);

				// Detect chessboard corners.  A minimum number of corners may be required to call it a detection.
				virtual void Detector(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
					const std::string &cameraName, ObservationPoints<float> &exitingObservationPoints,
					cv::Mat &targetImageMask, bool doFilter = false, float filterCutoff = 10) override;

				// Detect chessboard corners with adaptive windowing
				/*void DetectorAdaptiveWindowing(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
					const std::string &cameraName, ObservationPoints<float> &observationPts,
					cv::Mat &targetImageMask, cv::Mat & markerCornerOverlay, bool doFilter = false, 
					float filterCutoff = 10, bool doRefinement = false) const;*/

				
				void JsonSerialize(Json::Value &jsonNode) const override;


				void JsonDeserialize(const Json::Value &jsonNode) override;
                
			};//class ArucoBoard
		}//namespace CalibrationObject
	}//namespace Calibration
}//namespace BoeingMetrology

#endif
