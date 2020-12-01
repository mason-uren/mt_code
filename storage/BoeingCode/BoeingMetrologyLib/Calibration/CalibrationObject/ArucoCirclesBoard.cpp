#include "ArucoCirclesBoard.h"
#include "Calibration/aruco/charuco.hpp"
#include "Calibration/aruco/aruco.hpp"
#include "Utilities/Utilities.h"

#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "BoeingMetrology/BoeingMetrologyLib/Calibration/CirclesGrid/CirclesGrid.hpp"
#include "Calibration/CirclesGrid/BlobDetector.hpp"

#define DEBUG_BLOB_DETECTOR
namespace BoeingMetrology
{
	namespace Calibration
	{
		using namespace Observation;
		namespace CalibrationObject
		{
			int ArucoCirclesBoard::ArucoCirclesBoardInPixels(const cv::Size outSize, int marginSize, int chessboardSquaresX, int chessboardSquaresY, float squareLength)
			{
				CV_Assert(outSize.area() > 0);
				CV_Assert(marginSize >= 0);

				cv::Mat _img;
				_img.create(outSize, CV_8UC1);
				_img.setTo(255);
				cv::Mat out = _img;
				cv::Mat noMarginsImg =
					out.colRange(marginSize, out.cols - marginSize).rowRange(marginSize, out.rows - marginSize);

				double totalLengthX, totalLengthY;
				totalLengthX = squareLength * chessboardSquaresX;
				totalLengthY = squareLength * chessboardSquaresY;

				// proportional transformation
				double xReduction = totalLengthX / double(noMarginsImg.cols);
				double yReduction = totalLengthY / double(noMarginsImg.rows);

				// determine the zone where the chessboard is placed
				cv::Mat chessboardZoneImg;
				if (xReduction > yReduction) {
					int nRows = int(totalLengthY / xReduction);
					int rowsMargins = (noMarginsImg.rows - nRows) / 2;
					chessboardZoneImg = noMarginsImg.rowRange(rowsMargins, noMarginsImg.rows - rowsMargins);
				}
				else {
					int nCols = int(totalLengthX / yReduction);
					int colsMargins = (noMarginsImg.cols - nCols) / 2;
					chessboardZoneImg = noMarginsImg.colRange(colsMargins, noMarginsImg.cols - colsMargins);
				}

				// determine the margins to draw only the markers
				// take the minimum just to be sure
				double squareSizePixels = cv::min(double(chessboardZoneImg.cols) / double(chessboardSquaresX),
					double(chessboardZoneImg.rows) / double(chessboardSquaresY));

				return (int)squareSizePixels;
			}

			ArucoCirclesBoard::ArucoCirclesBoard() :
				arucoFormat_(),
				circleDiam_(0)
			{}

			ArucoCirclesBoard::ArucoCirclesBoard(ArucoBoard srcBoard, float circleDiameter)
			{
				arucoFormat_ = srcBoard;
				circleDiam_ = circleDiameter;
			}

			bool ArucoCirclesBoard::isValid() const
			{
				if (arucoFormat_.board_.empty() ||
					arucoFormat_.board_->dictionary.empty() ||
					arucoFormat_.board_->dictionary->bytesList.empty())
				{
					return false;
				}
				// other checks may go here.
				return true;
			}

			void ArucoCirclesBoard::ObjectGenerator(cv::Mat &matOut)
			{
				std::cout << "Aruco circles board beginning to print " << std::endl;
                
				// Obtain the number of pixels that make up a square
				auto squareSizeInPixels = arucoFormat_.CharucoBoardSquareInPixels(
                    cv::Size(arucoFormat_.imageWidth_, arucoFormat_.imageHeight_), 
                    arucoFormat_.borderWidth_, arucoFormat_.checkerboardSquareX_, arucoFormat_.checkerboardSquareY_, 
                    arucoFormat_.squareLength_, arucoFormat_.markerLength_);

                // Calculate number of pixels that make up a circle
                auto circleRadiusInPixels = squareSizeInPixels * (circleDiam_ / arucoFormat_.squareLength_);

				//// Board parameters are scaled to physical units
				//double squareSizeInPhysicalUnits = arucoFormat_.squareSizeInPhysicalUnits_, pixelSizeInPhysicalUnits = arucoFormat_.pixelSizeInPhysicalUnits_;
				//bool pixelSizeDerived = false;
				//double circleToSquareRatio = circleDiam_ / arucoFormat_.squareLength_;
				//if (arucoFormat_.pixelSizeInPhysicalUnits_ > 0)
				//	squareSizeInPhysicalUnits = arucoFormat_.chessboardSquareSizeInPixels_ * arucoFormat_.pixelSizeInPhysicalUnits_;
				//else
				//{
				//	pixelSizeInPhysicalUnits = squareSizeInPhysicalUnits / arucoFormat_.chessboardSquareSizeInPixels_;
				//	pixelSizeDerived = true;
				//}

				//double circleSizeInPhysicalUnits = squareSizeInPhysicalUnits * circleToSquareRatio;
				//double circleSizeInPixels = circleSizeInPhysicalUnits / pixelSizeInPhysicalUnits;
				//double circleRadiusInPixels = circleSizeInPixels / 2.0;

				//std::cout << "Circles radius in pixels " << circleRadiusInPixels;

				//double squareSizeInPixels = arucoFormat_.chessboardSquareSizeInPixels_;


                cv::Mat internalMat = cv::Mat::ones(arucoFormat_.checkerboardSquareY_*squareSizeInPixels, arucoFormat_.checkerboardSquareX_*squareSizeInPixels, CV_8UC1) * 255;

				for (int squareX = 0; squareX < arucoFormat_.checkerboardSquareX_ - 1; ++squareX)
				{
					double cornerX = (squareX + 1) * squareSizeInPixels;
					for (int squareY = 0; squareY < arucoFormat_.checkerboardSquareY_ - 1; ++squareY)
					{
						double cornerY = (squareY + 1) * squareSizeInPixels;
						std::cout << "CornerX " << cornerX << " CornerY " << cornerY << std::endl;
						cv::circle(internalMat, cv::Point2d(cornerX, cornerY), (int)(circleRadiusInPixels + 0.5), cv::Scalar(0), -1);
					}
				}

				//matOut = cv::Mat::ones(internalMat.rows + arucoFormat_.borderWidth_*2, internalMat.cols + arucoFormat_.borderWidth_*2, CV_8UC1) * 255;
				
				matOut = cv::Mat::ones(arucoFormat_.imageHeight_, arucoFormat_.imageWidth_, CV_8UC1) * 255;

				int widthDif = matOut.cols - (internalMat.cols + arucoFormat_.borderWidth_ * 2);

				int heightDif = matOut.rows - (internalMat.rows + arucoFormat_.borderWidth_ * 2);

				internalMat.copyTo(matOut(cv::Rect(arucoFormat_.borderWidth_ + widthDif / 2, arucoFormat_.borderWidth_ + heightDif / 2, internalMat.cols, internalMat.rows)));

			}

			void ArucoCirclesBoard::Detector(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
				const std::string &cameraName, BoeingMetrology::Calibration::ObservationPoints<float> &existingObservationPoints,
				cv::Mat &targetImageMask, bool doFilter, float filterCutoff)
			{
				cv2::SimpleBlobDetector::Params blobParams = cv2::SimpleBlobDetector::Params();
				cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

				blobParams.filterByArea = false;
				blobParams.minArea = 300;
				blobParams.maxArea = 20000;
				blobParams.minThreshold = 50;
				blobParams.maxThreshold = 165;
				blobParams.thresholdStep = 10;
				blobParams.minDistBetweenBlobs = 50;
				blobParams.minRepeatability = 5;

				blobParams.filterByArea = true;
				blobParams.minArea = 10000;
				blobParams.maxArea = 1000000;
				
				blobParams.filterByCircularity = true;
				blobParams.minCircularity = 0.1f;

				blobParams.filterByConvexity = true;
				blobParams.minConvexity = 0.87f;

				blobParams.filterByInertia = true;
				blobParams.minInertiaRatio = 0.01f;

				blobParams.filterByAreaGrouping = true;
				blobParams.keepAreaGroupingWithLargestArea = true;

				blobParams.removeEdgeContours = true;

				const cv::Ptr<cv2::SimpleBlobDetector> blobDetector = cv2::SimpleBlobDetector::create(blobParams);

				

				std::vector<Point2f> centers;

				

				CirclesGrid2::findCirclesGrid(inputImage, cv::Size(arucoFormat_.checkerboardSquareX_, arucoFormat_.checkerboardSquareY_), 
					static_cast<int>(existingObservationPoints.observedPoints.size()), centers, cv::CALIB_CB_SYMMETRIC_GRID, blobDetector);

				for(cv::Point2f point : centers)
				{
					std::cout << "Blob detected at " << point << std::endl;
				}

				//this will be in the same order as the observation points, eg. keypoint[0] -> observedPoints[0]
				std::vector<cv::Point2f> matchedKeypoints = {};
				for (int i = 0; i < existingObservationPoints.observedPoints.size(); ++i)
				{
					double minDist = 1000;
					cv::Point2f best = cv::Point2f();
					cv::Point2f imgPoint = existingObservationPoints.observedPoints[i];
					for(cv::Point2f pt : centers)
					{
						double dist = cv::norm(imgPoint - pt);
						if (dist < minDist)
						{
							best = pt;
							minDist = dist;
						}
					}
					matchedKeypoints.push_back(best);
				}
			
				

				for (int i = 0; i < existingObservationPoints.observedPoints.size(); ++i)
				{
					existingObservationPoints.observedPoints[i] = matchedKeypoints[i];
				}

				if (!targetImageMask.empty() && targetImageMask.size == inputImage.size)
				{
					cv::aruco::drawDetectedCornersCharuco(targetImageMask, existingObservationPoints.observedPoints, existingObservationPoints.markerIdentifier, cv::Scalar(0, 0, 255));
				}
				

				detectedTarget = true;
			}

			//void ArucoCirclesBoard::DetectorAdaptiveWindowing(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
			//	const std::string &cameraName, BoeingMetrology::Calibration::ObservationPoints<float> &observationPts,
			//	cv::Mat &targetImageMask, cv::Mat & markerCornerOverlay, bool doFilter, float filterCutoff, bool doRefinement) const
			//{
			//	 //Manually copy into aruco2 board 
			//	cv::Size s = arucoFormat_.board_->getChessboardSize();
			//	float slength = arucoFormat_.board_->getSquareLength();
			//	float mlength = arucoFormat_.board_->getMarkerLength();
			//	cv::Ptr<cv::aruco2::CharucoBoard> aruco2board = cv::aruco2::CharucoBoard::create(s.width, s.height, slength, mlength, arucoFormat_.board_->dictionary);

			//	//double winSizeScaleFactor = ComputeCornerRefinementWinSizePixelScaleFactor(inputImage.size());
			//	double winSizeScaleFactor = 1;
			//	// Detect marker corners
			//	std::vector< int > markerIds;
			//	std::vector< std::vector<cv::Point2f> > markerCorners;
			//	auto params = cv::aruco2::DetectorParameters::create();
			//	params->cornerRefinementMethod = cv::aruco2::CORNER_REFINE_NONE;
			//	if (cameraName.find("Imperx") != std::string::npos)
			//	{
			//		// Perform marker corner refinement with custom settings for Imperx
			//		params->cornerRefinementMethod = cv::aruco2::CORNER_REFINE_SUBPIX;
			//		params->cornerRefinementMaxIterations = 100;
			//		params->cornerRefinementWinSize *= winSizeScaleFactor;
			//	}
			//	cv::aruco2::detectMarkers(inputImage, arucoFormat_.board_->dictionary, markerCorners, markerIds, params);

			//	// Overlay the marker corners on the image
			//	markerCornerOverlay = inputImage.clone();
			//	BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, markerCorners, 20, cv::Scalar(0, 0, 255, 0));
			//	BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, markerCorners, 2, cv::Scalar(0, 0, 255, 0));

			//	detectedTarget = false;

			//	// if at least one marker detected
			//	if (markerIds.size() > 0)
			//	{
			//		// Interpolate to chessboard corners based on marker corners
			//		std::vector<cv::Point2f> charucoCorners;
			//		std::vector<int> charucoIds;
			//		cv::aruco2::interpolateCornersCharuco(markerCorners, markerIds, inputImage, aruco2board, charucoCorners, charucoIds, winSizeScaleFactor);

			//		// Overlay the chessboard corners
			//		BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, charucoCorners, 15, cv::Scalar(255, 0, 0, 0));
			//		BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, charucoCorners, 2, cv::Scalar(0, 0, 255, 0));

			//		if (charucoCorners.size() >= numRequiredDetectedMarkers)
			//		{
			//			observationPts.InitializeObservations(cameraName, inputImage.size());
			//			for (int i = 0; i < charucoCorners.size(); i++)
			//			{
			//				observationPts.AddObservation(charucoIds[i], charucoCorners[i], arucoFormat_.board_->chessboardCorners[charucoIds[i]]);
			//			}

			//			if (observationPts.observedPoints.size() > numRequiredDetectedMarkers)
			//			{
			//				detectedTarget = true;


			//				if (!targetImageMask.empty() && targetImageMask.size == inputImage.size)
			//				{
			//					cv::aruco2::drawDetectedCornersCharuco(targetImageMask, charucoCorners, charucoIds, cv::Scalar(0, 0, 255));
			//				}
			//			}
			//			else
			//			{
			//				std::cout << "Filtering the points caused the number of observations to fall below threshold" << std::endl;
			//			}

			//		}
			//	}
			//}



			void ArucoCirclesBoard::JsonSerialize(Json::Value &jsonNode) const
			{
				arucoFormat_.JsonSerialize(jsonNode);
				jsonNode["circleDiameter"] = circleDiam_;
			}

			void ArucoCirclesBoard::JsonDeserialize(const Json::Value &jsonNode)
			{
				arucoFormat_.JsonDeserialize(jsonNode);
				circleDiam_ = jsonNode["circleDiameter"].asFloat();
			}

		}//namespace CalibrationObject
	}//namespace Calibration
}//namespace BoeingMetrology
