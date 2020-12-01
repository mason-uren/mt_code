#include "ArucoBoard.h"
#include "Calibration/aruco/charuco.hpp"
#include "Calibration/aruco/aruco.hpp"
#include "Utilities/Utilities.h"
#include "highgui.h"
#include <map>

namespace BoeingMetrology
{
namespace Calibration
{
    using namespace Observation;
namespace CalibrationObject
{
    int ArucoBoard::CharucoBoardSquareInPixels(const cv::Size outSize, int marginSize, int chessboardSquaresX, int chessboardSquaresY, float squareLength, float markerLength)
    {
        CV_Assert(outSize.area() > 0);
        CV_Assert(marginSize >= 0);

        if (markerLength >= squareLength)
        {
            std::cout << "Marker length must be < square length" << std::endl;
            return false;
        }

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

    ArucoBoard::ArucoBoard() :
        imageWidth_(0),
        imageHeight_(0),
        checkerboardSquareX_(0),
        checkerboardSquareY_(0),
        borderWidth_(0),
        squareLength_(0),
        markerLength_(0),
        pixelSizeInPhysicalUnits_(-1),
        squareSizeInPixels_(-1),
        markerPattern_(cv::aruco::DICT_4X4_50)
    {

    }

    bool ArucoBoard::isValid() const
    {
        if (board_.empty() ||
            board_->dictionary.empty() ||
            board_->dictionary->bytesList.empty())
        {
            return false;
        }
        // other checks may go here.
        return true;
    }

    void ArucoBoard::GenerateArucoBoardMask(const cv::Mat& image, cv::Mat& mask, cv::Mat& overlaidImage) const
	{
		std::vector<float> extremePoints;
		// get board perimeter
		ArucoBoard::getCharucoBoardExtremePoints(image, this->board_, extremePoints);

        if (extremePoints.size() == 4)
        {
            std::vector<cv::Point2i> extremeCoordinates;

            extremeCoordinates.push_back(cv::Point2i((int)(extremePoints[0] + 0.5), (int)(extremePoints[2]+ 0.5)));
            extremeCoordinates.push_back(cv::Point2i((int)(extremePoints[0] + 0.5), (int)(extremePoints[3]+ 0.5)));
            extremeCoordinates.push_back(cv::Point2i((int)(extremePoints[1] + 0.5), (int)(extremePoints[3]+ 0.5)));
            extremeCoordinates.push_back(cv::Point2i((int)(extremePoints[1] + 0.5), (int)(extremePoints[2]+ 0.5)));

            // Draw a contour around the 4 outermost detected markers, overlaid on the image
            overlaidImage = image.clone();
            std::vector<std::vector<cv::Point2i>> arrayOfArrays;
            arrayOfArrays.push_back(extremeCoordinates);
            cv::drawContours(overlaidImage, arrayOfArrays, 0, cv::Scalar(255, 255, 255), 25, 8);

            // Create the binary mask for thresholding out all but the aruco pixels
            mask = cv::Mat::zeros(image.size(), CV_8UC1);
            cv::drawContours(mask, arrayOfArrays, 0, cv::Scalar(255, 255, 255), CV_FILLED);
        }
        else
            std::cout << "GenerateArucoBoardMask: Failed to detect board" << std::endl;
	}

	void ArucoBoard::getCharucoBoardExtremePoints(const cv::Mat& image, const cv::Ptr<cv::aruco::CharucoBoard>& board, std::vector<float>& extremePoints)
	{
        // Define and initialize vars
		cv::Ptr<cv::aruco::Dictionary> dict = board->dictionary;
		float markerLen = board->getMarkerLength();
		float sqLen = board->getSquareLength();
		float marginToUse = (sqLen - markerLen) / 2.0f;
		int numImageRows = image.rows;
		int numImageCols = image.cols;

        // Identify markers and their corners in the image
		std::vector<int> markerIDs;
		std::vector<std::vector<cv::Point2f>> markerCorners;
		cv::aruco::detectMarkers(image, dict, markerCorners, markerIDs);
		int numOfMarkers = (int)markerIDs.size();

		if (numOfMarkers == 0)
		{
			return;
		}

		int leftMostX = numImageRows - 1; 
        int rightMostX = 0;
		int upperMostY = numImageCols - 1; 
        int bottomMostY = 0;

		std::vector<cv::Point2f> firstMarkerPoints = markerCorners[0];
		float markerWidthPixels = firstMarkerPoints[1].x - firstMarkerPoints[0].x;
		marginToUse *= markerWidthPixels / markerLen;

		for (int i = 0; i < numOfMarkers; i++)
		{
			std::vector<cv::Point2f> marker = markerCorners[i];
			int numCorners = (int)marker.size();
			if (numCorners == 0) continue;
			for (int j = 0; j < numCorners; j++)
			{
				cv::Point2f corner = marker[j];
				if (corner.x < (float)leftMostX)
					leftMostX = (int)corner.x;
				if (corner.x >(float)rightMostX)
					rightMostX = (int)corner.x;
				if (corner.y < (float)upperMostY)
					upperMostY = (int)corner.y;
				if (corner.y >(float)bottomMostY)
					bottomMostY = (int)corner.y;
			}
		}
		extremePoints.clear();
		extremePoints.push_back(leftMostX - marginToUse);
		extremePoints.push_back(rightMostX + marginToUse);
		extremePoints.push_back(upperMostY - marginToUse);
		extremePoints.push_back(bottomMostY + marginToUse);
	}

    /**
     * \return Physical square size for a digital board, or -1 if this board is not digital (ie. has no pixelSizeInPhysicalUnits set)
     */
    double ArucoBoard::getPhysicalSquareSizeOfDigitalBoard() const
    {
        return (pixelSizeInPhysicalUnits_ > 0) ? squareSizeInPixels_ * pixelSizeInPhysicalUnits_ : -1;
    }

    std::vector<cv::Point3f> ArucoBoard::GetControls()
	{
		std::vector<cv::Point3f> result = {};
		double controlY = 0.0;
		for (int squareRow = 0; squareRow < checkerboardSquareY_ - 1; ++squareRow)
		{
			controlY += squareLength_;
			float controlX = 0.0;
			for (int squareCol = 0; squareCol < checkerboardSquareX_-1; ++squareCol)
			{
				controlX += squareLength_;
				result.push_back(cv::Point3f((float)controlX, (float)controlY, 0.0f));
			}
		}
		return result;
	}

    void ArucoBoard::ObjectGenerator(cv::Mat &matOut)
    {
        arucoDict_ = cv::aruco::getPredefinedDictionary(markerPattern_);

        if (pixelSizeInPhysicalUnits_ > 0)
        {
            // Using a digital board

            // 1. Obtain the number of pixels that make up a marker based on squareSizeInPixels_ which is calculated during setBoardParameters() 
            auto markerSizeInPixels = (markerLength_ / squareLength_) * squareSizeInPixels_;

            // 2. Create a board that will display at the correct size
            board_ = cv::aruco::CharucoBoard::create(
                checkerboardSquareX_, checkerboardSquareY_, 
                (float)(squareSizeInPixels_ * pixelSizeInPhysicalUnits_),
                (float)(markerSizeInPixels * pixelSizeInPhysicalUnits_),
                arucoDict_);
        }
        else
        {
            // Using a physical board - simple case.
            board_ = cv::aruco::CharucoBoard::create(checkerboardSquareX_, checkerboardSquareY_, squareLength_, markerLength_, arucoDict_);
        }

        board_->draw(cv::Size(imageWidth_, imageHeight_), matOut, borderWidth_);
    }

    void ArucoBoard::Detector(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
        const std::string &cameraName, BoeingMetrology::Calibration::ObservationPoints<float> &observationPts,
        cv::Mat &targetImageMask, bool doFilter, float filterCutoff)
    {
        std::vector< int > markerIds;
        std::vector< std::vector<cv::Point2f> > markerCorners;
        auto params = cv::aruco::DetectorParameters::create();
        // DetectorParameters::doCornerRefinement no longer exist from OpenCV 3.3
#if CV_VERSION_MAJOR < 3 || CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR <= 2
        params->doCornerRefinement = false;
#endif
        cv::aruco::detectMarkers(inputImage, board_->dictionary, markerCorners, markerIds, params);

        detectedTarget = false;

        // if at least one marker detected
        if (markerIds.size() > 0)
        {
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board_, charucoCorners, charucoIds);
            if (charucoCorners.size() >= numRequiredDetectedMarkers)
            {
                observationPts.InitializeObservations(cameraName, inputImage.size());
                for (int i = 0; i < charucoCorners.size(); i++)
                {
                    observationPts.AddObservation(charucoIds[i], charucoCorners[i], board_->chessboardCorners[charucoIds[i]]);
                }

                if (doFilter)
                {
                    cv::Mat linesImg;

                    FilterMarkers(observationPts, inputImage, linesImg, filterCutoff);

                }

                if (observationPts.observedPoints.size() > numRequiredDetectedMarkers)
                {
                    detectedTarget = true;


                    if (!targetImageMask.empty() && targetImageMask.size == inputImage.size)
                    {
                        cv::aruco::drawDetectedCornersCharuco(targetImageMask, charucoCorners, charucoIds, cv::Scalar(0, 0, 255));
                    }
                }
                else
                {
                    std::cout << "Filtering the points caused the number of observations to fall below threshold" << std::endl;
                }

            }
        }
    }

    double ArucoBoard::ComputeCornerRefinementWinSizePixelScaleFactor(const cv::Size & imgSize)
    {
        // See LIVPLN-600 https://casa-jira.web.boeing.com/browse/LIVPLN-600
        // 640x480 -> scale factor is 1
        // 7360x4912 -> scale factor is 5

        // Linearly interpolate (or extrapolate) based on image width
        double slope = (5.0 - 1.0) / (7360.0 - 640.0);
        double scaleFactor = slope * imgSize.width + 0.619;
        return scaleFactor;
    }

    void ArucoBoard::DetectorAdaptiveWindowing(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
        const std::string &cameraName, BoeingMetrology::Calibration::ObservationPoints<float> &observationPts,
        cv::Mat &targetImageMask, cv::Mat & markerCornerOverlay, bool doFilter, float filterCutoff) const
    {
        // Manually copy into aruco2 board where we are compiling the source code
        cv::Size s = board_->getChessboardSize();
        float slength = board_->getSquareLength();
        float mlength = board_->getMarkerLength();
        cv::Ptr<cv::aruco2::CharucoBoard> aruco2board = cv::aruco2::CharucoBoard::create(s.width, s.height, slength, mlength, board_->dictionary);

        double winSizeScaleFactor = ComputeCornerRefinementWinSizePixelScaleFactor(inputImage.size());
		winSizeScaleFactor = 1.0;

        // We may modify the image prior to the detector
        cv::Mat inputImageSmooth = inputImage; // Soft copy

        // Detect marker corners
        std::vector< int > markerIds;
        std::vector< std::vector<cv::Point2f> > markerCorners;
        auto params = cv::aruco2::DetectorParameters::create();
        params->cornerRefinementMethod = cv::aruco2::CORNER_REFINE_NONE;
		params->minMarkerPerimeterRate = 0.001;
        if (cameraName.find("Imperx") != std::string::npos)
        {
            // Perform marker corner refinement with custom settings for Imperx
            params->cornerRefinementMethod = cv::aruco2::CORNER_REFINE_SUBPIX;
            params->cornerRefinementMaxIterations = 100;
            params->cornerRefinementWinSize = (int)((double)params->cornerRefinementWinSize * winSizeScaleFactor + 0.5);
        }
        else if (cameraName.find("Baumer") != std::string::npos)
        {
            // We are resolving 4k monitor pixels with this sensor, this blurring is to smooth out the grid of the pixels,
            // which was majorly disturbing the marker and corner detector
            cv::GaussianBlur(inputImage, inputImageSmooth, cv::Size(7, 7), 1.5, 1.5);
        }
        cv::aruco2::detectMarkers(inputImageSmooth, board_->dictionary, markerCorners, markerIds, params);

        // Overlay the marker corners on the image
        markerCornerOverlay = inputImageSmooth.clone();
        BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, markerCorners, 13, cv::Scalar(0, 0, 255, 0), 3);
        BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, markerCorners, 1, cv::Scalar(0, 0, 255, 0), 1);

        detectedTarget = false;

        // if at least one marker detected
        if (markerIds.size() > 0)
        {
            // Interpolate to chessboard corners based on marker corners
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco2::interpolateCornersCharuco(markerCorners, markerIds, inputImageSmooth, aruco2board, charucoCorners, charucoIds, winSizeScaleFactor);

            // Overlay the chessboard corners
            BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, charucoCorners, 3, cv::Scalar(255, 0, 0, 0), 1);
            BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, charucoCorners, 1, cv::Scalar(0, 0, 255, 0), 1);

            if (charucoCorners.size() >= numRequiredDetectedMarkers)
            {
                observationPts.InitializeObservations(cameraName, inputImage.size());
                for (int i = 0; i < charucoCorners.size(); i++)
                {
                    observationPts.AddObservation(charucoIds[i], charucoCorners[i], board_->chessboardCorners[charucoIds[i]]);
                }

                if (doFilter)
                {
                    cv::Mat linesImg;

                    FilterMarkers(observationPts, inputImageSmooth, linesImg, filterCutoff);

                }

                if (observationPts.observedPoints.size() > numRequiredDetectedMarkers)
                {
                    detectedTarget = true;


                    if (!targetImageMask.empty() && targetImageMask.size == inputImage.size)
                    {
                        cv::aruco2::drawDetectedCornersCharuco(targetImageMask, charucoCorners, charucoIds, cv::Scalar(0, 0, 255));
                    }
                }
                else
                {
                    std::cout << "Filtering the points caused the number of observations to fall below threshold" << std::endl;
                }

            }
        }
    }


    bool ArucoBoard::DetectCornersAndEstimatePose(const cv::Mat &inputImageOrig, const std::string &cameraName, const Calibration::IntrinsicData & intrinsicData, 
        const int & minNumCorners, const double & maxReprojThresh, const std::string & accumulatedImageDir, const std::string & poseDir, 
        CameraObservation &observationPts, cv::Mat &accumulatedImageMask, cv::Mat & markerCornerOverlay, Calibration::Observation::ObservationAngles & observationAngles) const
    {
        // Validate inputs
        if (inputImageOrig.rows == 0 || inputImageOrig.cols == 0)
            throw std::runtime_error("ArucoBoard::DetectCornersAndEstimatePose: " + cameraName + " image is empty");
        cv::Mat inputImage = inputImageOrig;
        if (inputImageOrig.type() == CV_16UC1)
        {
            inputImage = cv::Mat(inputImageOrig.size(), CV_8UC1);
            cv::normalize(inputImageOrig, inputImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        }

        // Load existing coverage results.  If the file doesn't exist, initialize the image.
        if (accumulatedImageDir != "" && accumulatedImageMask.rows == 0)
        {
            std::string maskImageName = accumulatedImageDir + "/" + cameraName + ".jpeg";
            accumulatedImageMask = cv::imread(maskImageName);
            if (accumulatedImageMask.data == NULL)
                accumulatedImageMask = cv::Mat::zeros(inputImage.rows, inputImage.cols, CV_8UC3);
        }

        // Load cumulative ObservationAngles.  Initialize if there's no file to load.
        try
        {
            std::string fname = accumulatedImageDir + "/" + cameraName + "_observationAngles.json";
            observationAngles = Calibration::Observation::ObservationAngles();
            observationAngles.DeserializeFile(fname);
        }
        catch (...)
        {
            observationAngles = Calibration::Observation::ObservationAngles();
        }

        // Initialize ObservationPoints, which are cleared at each pose
        observationPts.InitializeObservations(cameraName, inputImage.size());

        // Attempt to detect aruco board
        bool detectedTarget = false;
        this->DetectorAdaptiveWindowing(inputImage, minNumCorners, detectedTarget, cameraName, observationPts, accumulatedImageMask, markerCornerOverlay);

        if (detectedTarget)
        {
            // Estimate the board's pose in the camera frame, optionally perform a reprojection error check, and add the rotation angles to the histograms
            try
            {
                cv::Mat t;
                double rxprime = 0, ryprime = 0, rzprime = 0, rx = 0, ry = 0, rz = 0;
                std::map<MARKER_IDENTIFIER, cv::Point2f> outliers;
                //std::cout << poseDir << std::endl;
                if (maxReprojThresh == 0.0) // Pose estimation only
                {
                    observationPts.Estimate3dPoseEuler(intrinsicData, rxprime, ryprime, rzprime, t, outliers);
                }
                else
                {
                    // Perform pose estimation and also compare reprojection error to the provided threshold
                    double rms = observationPts.Estimate3dPoseEuler(intrinsicData, rxprime, ryprime, rzprime, t, outliers, true);

                    if (outliers.size() > 0)
                    {
                        std::cout << "DETECTION OUTLIERS -> " << poseDir + "/" + cameraName + "markerCorners.png" << std::endl;
                        std::vector<cv::Point2f> outlierList;
                        for (const auto & outlier : outliers)
                            outlierList.push_back(outlier.second);
                        BoeingMetrology::Utilities::DrawCircles(markerCornerOverlay, outlierList, 50, cv::Scalar(255, 0, 0, 0));
                    }

                    if (rms > maxReprojThresh)
                    {
                        detectedTarget = false;
                        std::cout << "MultiDetector: " + cameraName + " detection discarded.  Reprojection error = " + std::to_string(rms) << std::endl;
                        return false;
                    }
                }

                // Add observation angles to the histogram
                observationAngles.ModulateObservation(rxprime, ryprime, rzprime, rx, ry, rz);
                observationAngles.AddObservation(rx, ry, rz);

                if (accumulatedImageDir != "")
                {
                    // Write out the cumulative observation angles
                    std::string obsAnglesFname = accumulatedImageDir + "/" + cameraName + "_observationAngles.json";
                    observationAngles.SerializeFile(obsAnglesFname);
                }
            }
            catch (std::exception & e)
            {
                std::cout << "ChArucoBoardMultiDetector: Failed rotation angle estimation for " + cameraName << std::endl << " - " << e.what();
            }

            if (poseDir != "")
            {
                // Write out the serialized ObservationPoints object
                std::string jsonFileName = poseDir + "/ChArUco" + cameraName + ".json";
                std::ofstream jsonStream(jsonFileName);
                observationPts.SerializeFile(jsonFileName);

                // Write out the overlaid detections image
                std::string fname = poseDir + "/" + cameraName + "markerCorners.jpeg";
                cv::imwrite(fname, markerCornerOverlay);
            }

            if (accumulatedImageDir != "")
            {
                // Write out the accumulated coverage image
                std::string maskImageName = accumulatedImageDir + "/" + cameraName + ".jpeg";
                cv::imwrite(maskImageName, accumulatedImageMask);
            }
        } // End detectedTarget check

        // Write the aruco board description as json to disk
        std::string jsonFileName = poseDir + "/CalibrationObject_" + cameraName + ".json";
        this->SerializeFile(jsonFileName);

        return detectedTarget;
    }

    void ArucoBoard::FilterMarkers(BoeingMetrology::Calibration::ObservationPoints<float> &observationPts,
        cv::Mat srcImage, cv::Mat &lineImg, float maxResidualPerPoint) const
    {
        cv::Size chessSize = board_->getChessboardSize();
        int downWidth = chessSize.width - 1;
        int downHeight = chessSize.height - 1;
        std::vector<std::vector<int>> lineMemberIds;
        std::map<int, bool> idPresent = std::map<int, bool>();
        std::map<int, bool> idBelongsToLine = std::map<int, bool>();
        std::map<int, cv::Point2f> idMarkerMap = std::map<int, cv::Point2f>();
        std::map<int, bool> idValid = std::map<int, bool>();
        for (int i = 0; i < downWidth * downHeight; ++i)
        {
            idPresent[i] = false;
        }

        for (int i = 0; i < observationPts.markerIdentifier.size(); ++i)
        {
            int id = observationPts.markerIdentifier[i];
            idPresent[id] = true;
            idMarkerMap[id] = observationPts.observedPoints[i];
            idValid[id] = true;
            idBelongsToLine[id] = false;
        }
        for (int col = 0; col < downWidth; ++col)
        {
            std::vector<int> memberIds = {};
            for (int id = col; id < downHeight*downWidth; id += downWidth)
            {
                if (idPresent[id])
                {
                    memberIds.push_back(id);
                }
            }
            lineMemberIds.push_back(memberIds);
        }
        for (int row = 0; row < downWidth * downHeight; row += downWidth)
        {
            std::vector<int> memberIds = {};
            for (int id = row; id < row + downWidth; ++id)
            {
                if (idPresent[id])
                {
                    memberIds.push_back(id);
                }
            }
            lineMemberIds.push_back(memberIds);
        }

        cv::Vec2f axis1, axis2;
        std::vector<cv::Vec2f> lineSlopes = {};
        for(std::vector<int> lineIds : lineMemberIds)
        {
            std::vector<cv::Point2f> lineMarkers = {};
            for(int id : lineIds)
            {
                lineMarkers.push_back(idMarkerMap[id]);
            }
            if (lineIds.size() < 2)
            {
                continue;
            }
            cv::Vec4f lineEq;
            cv::fitLine(lineMarkers, lineEq, CV_DIST_L2, 0, 0.01, 0.01);
            lineSlopes.push_back(cv::Vec2f(lineEq[0], lineEq[1]));
        }
        axis1 = lineSlopes[0];
        float maxDif = 0;
        for(cv::Vec2f slope : lineSlopes)
        {
            float dif = (float)cv::norm(axis1 - slope);
            if (dif > maxDif)
            {
                maxDif = dif;
                axis2 = slope;
            }
        }
        std::vector<cv::Vec2f> l1s, l2s;
        for(cv::Vec2f slope : lineSlopes)
        {
            float dif1 = (float)cv::norm(slope - axis1);
            float dif2 = (float)cv::norm(slope - axis2);

            if (dif1 < dif2)
            {
                l1s.push_back(slope);
                axis1 = cv::Vec2f();
                for(cv::Vec2f l : l1s)
                {
                    axis1 += l;
                }
                axis1 = axis1 * (1.0 / l1s.size());
            }
            else
            {
                l2s.push_back(slope);
                axis2 = cv::Vec2f();
                for(cv::Vec2f l : l2s)
                {
                    axis2 += l;
                }
                axis2 = axis2 * (1.0 / l2s.size());
            }
        }

        std::vector<cv::Vec4f> lines = {};
        for(std::vector<int> lineIds : lineMemberIds)
        {
            float lineResidual = 10000.0;
            std::vector<int> removedIds = {};
            std::vector<int> remainingIds = {};
            for(int id : lineIds)
            {
                if (idValid[id])
                {
                    remainingIds.push_back(id);
                }
            }

            bool isAxis1 = false;
            bool isAxisDecided = false;
            cv::Vec4f lineEq;
            bool removePoint = true;
            while (removePoint && remainingIds.size() > 1)
            {
                std::vector<cv::Point2f> lineMarkers = {};
                for(int id : remainingIds)
                {
                    lineMarkers.push_back(idMarkerMap[id]);
                }


                cv::Vec2f origVec;
                cv::Vec2f lineSlope;
                if (!isAxisDecided)
                {
                    cv::fitLine(lineMarkers, lineEq, CV_DIST_L2, 0, 0.01, 0.01);
                    origVec = cv::Vec2f(lineEq[2], lineEq[3]);
                    lineSlope = cv::Vec2f(lineEq[0], lineEq[1]);
                    if (cv::norm(lineSlope - axis1) < cv::norm(lineSlope - axis2))
                    {
                        isAxisDecided = true;
                        isAxis1 = true;
                    }
                    else
                    {
                        isAxisDecided = true;
                        isAxis1 = false;
                    }
                }
                else
                {
                    cv::fitLine(lineMarkers, lineEq, CV_DIST_L2, 0, 0.01, 0.01);
                }
                origVec = cv::Vec2f(lineEq[2], lineEq[3]);
                lineSlope = cv::Vec2f(lineEq[0], lineEq[1]);
                /*if (isAxis1)
                {
                adjustLine(lineEq, lineMarkers, axis1);
                }
                else
                {
                adjustLine(lineEq, lineMarkers, axis2);
                }*/
                float maxDist = 0.0;
                int maxDistId = 0;
                lineResidual = 0.0;
                removePoint = false;
                for(int id : remainingIds)
                {
                    cv::Point2f mPoint = idMarkerMap[id];
                    cv::Vec2f mVec = cv::Vec2f(mPoint.x, mPoint.y);
                    float dist = (float)cv::norm((mVec - origVec) - lineSlope*(mVec - origVec).dot(lineSlope));
                    if (dist > maxDist)
                    {
                        maxDist = dist;
                        maxDistId = id;
                    }
                    if (dist > maxResidualPerPoint)
                    {
                        removePoint = true;
                    }

                    lineResidual += dist;
                }

                if (removePoint)
                {
                    std::cout << "removing point from set, markerId : " << maxDistId << " residual : " << maxDist << std::endl;
                    removedIds.push_back(maxDistId);
                    std::vector<int> tempRemainingIds = {};
                    for(int id : remainingIds)
                    {
                        if (id != maxDistId)
                        {
                            tempRemainingIds.push_back(id);
                        }
                    }
                    remainingIds = tempRemainingIds;
                }

            }
            if (remainingIds.size() > 1)
            {
                lines.push_back(lineEq);
                /*std::cout << "the per point residual for this line is " << lineResidual / remainingIds.size() << std::endl;
                std::cout << "The line eq is " << lineEq[0] << " " << lineEq[1] << std::endl;*/
                for(int id : remainingIds)
                {
                    idBelongsToLine[id] = true;
                }

            }

            for(int id : removedIds)
            {
                idValid[id] = false;
            }

        }



        srcImage.copyTo(lineImg);
        if (lineImg.channels() == 1)
        {
            cv::cvtColor(lineImg, lineImg, CV_GRAY2RGB);
        }
        for(cv::Vec4f lineEq : lines)
        {
            float x0 = lineEq[2];
            float y0 = lineEq[3];
            float dx = lineEq[0];
            float dy = lineEq[1];
            float x1 = x0;
            float x2 = x0;
            float y1 = y0;
            float y2 = y0;
            while (x1 > 0 && x1 < lineImg.cols && y1 > 0 && y1 < lineImg.rows)
            {
                x1 += dx;
                y1 += dy;
            }
            while (x2 > 0 && x2 < lineImg.cols && y2 > 0 && y2 < lineImg.rows)
            {
                x2 -= dx;
                y2 -= dy;
            }
            cv::Point p1 = cv::Point((int)(x1 + 0.5), (int)(y1 + 0.5));
            cv::Point p2 = cv::Point((int)(x2 + 0.5), (int)(y2 + 0.5));
            cv::line(lineImg, p1, p2, cv::Scalar(0, 0, 255), 5);
        }
        for(int id : observationPts.markerIdentifier)
        {
            cv::Point2f p = idMarkerMap[id];
            if (idValid[id] && idBelongsToLine[id])
            {
                //std::cout << "Marker " << id << " is still valid " << std::endl;
                cv::circle(lineImg, p, 30, cv::Scalar(0, 255, 0), 10);
            }
            else
            {
                //std::cout << "Marker " << id << " is not valid " << std::endl;
                cv::circle(lineImg, p, 20, cv::Scalar(0, 0, 255), 7);
            }
        }

        ObservationPoints<float> remainingObs;
        remainingObs.InitializeObservations(observationPts.cameraName, observationPts.imageSize);
        for(int id : observationPts.markerIdentifier)
        {
            if (idValid[id] && idBelongsToLine[id])
            {
                remainingObs.AddObservation(id, idMarkerMap[id], board_->chessboardCorners[id]);
            }
        }
        observationPts = remainingObs;

    }

    void ArucoBoard::MarkerSanityCheck(const cv::Mat srcImage, std::vector<cv::Point2f> observations, int sanityWindowSize,
        bool &badMarkersFound, std::vector<cv::Point2f> &badMarkers, cv::Mat &badMarkerImage) const
    {
        
        badMarkersFound = false;
        badMarkers = {};
        srcImage.copyTo(badMarkerImage);

        std::vector<cv::Mat> channels;
        cv::split(srcImage, channels);

        //we could look at iterating over channels, but ignore for now
        cv::Mat toCheck = channels[0];

        int rad = sanityWindowSize / 2;
        double globalMax=0, globalMin=255;
        cv::minMaxLoc(toCheck, &globalMin, &globalMax);
        double globalDif = globalMax - globalMin;
        double requiredLocalDif = 0.5 * globalDif;

        double localMax=0, localMin=255;
        for(cv::Point2f marker : observations)
        {
            int topRow, botRow, leftCol, rightCol;
            topRow = std::max((int)marker.y - rad, 0);
            botRow = std::min((int)marker.y + rad, srcImage.rows - 1);
            leftCol = std::max((int)marker.x - rad, 0);
            rightCol = std::min((int)marker.x + rad, srcImage.cols - 1);

            cv::Mat window = srcImage(cv::Rect(cv::Point(leftCol, topRow), cv::Point(rightCol, botRow)));
            cv::minMaxLoc(window, &localMin, &localMax);
            
            if (localMax - localMin < requiredLocalDif)
            {
                badMarkersFound = true;
                badMarkers.push_back(marker);
            }
        }

        try
        {
            BoeingMetrology::Utilities::DrawCircles(badMarkerImage, badMarkers, 20, cv::Scalar(0, 0, 255));
            BoeingMetrology::Utilities::DrawCircles(badMarkerImage, badMarkers, 2, cv::Scalar(0, 0, 255));
        }
        catch (std::exception & e)
        {
            std::cout << "Exception in ArucoBoard::MarkerSanityCheck - " << e.what();
        }
    }

    void ArucoBoard::MarkerSanityCheckHarris(const cv::Mat srcImage, std::vector<cv::Point2f> observations, int sanityWindowSize,
        bool &badMarkersFound, std::vector<cv::Point2f> &badMarkers, cv::Mat &badMarkerImage, std::vector<cv::Point2f> &refinedMarkers) const
    {
        badMarkersFound = false;
        badMarkers = {};
        srcImage.copyTo(badMarkerImage);

        std::vector<cv::Mat> channels;
        cv::split(srcImage, channels);

        //we could look at iterating over channels, but ignore for now
        cv::Mat gray = cv::Mat::zeros(srcImage.size(), CV_8UC1);
        
        for(const cv::Mat & chan : channels)
        {
            gray = gray + chan / (double)channels.size();
        }

        gray = channels[0];

        int rad = sanityWindowSize / 2;
        std::cout << "Starting harris detection " << std::endl;
        cv::Mat harris = cv::Mat::zeros(gray.size(), CV_32FC1);
        cv::cornerHarris(gray, harris, 2, 3, 0.04, cv::BORDER_DEFAULT);
        cv::Mat scaledHarris;
        harris.copyTo(scaledHarris);

        //cv::Mat scaledHarris = cv::Mat::zeros(gray.size(), CV_32FC1);
        /*cv::normalize(harris, scaledHarris, 1, cv::NORM_MINMAX);
        cv::Mat normalHarris;
        scaledHarris.copyTo(normalHarris);
        
        cv::convertScaleAbs(scaledHarris, normalHarris);
        normalHarris.copyTo(scaledHarris);*/

     /*   cv::namedWindow("harris", CV_WINDOW_NORMAL);
        cv::imshow("harris", scaledHarris);
        cv::waitKey();*/

        std::cout << "Finshed intial harris detection " << std::endl;
        cv::Mat harrisAccumulator = cv::Mat::zeros(scaledHarris.size(), CV_32FC1);

        int neighborhoodSize = 1;
        float *acuptr = harrisAccumulator.ptr<float>(0);
        float *scaleptr = scaledHarris.ptr<float>(0);
        for (int row = neighborhoodSize + 5; row < harrisAccumulator.rows - neighborhoodSize - 5; ++row)
        {
            //std::cout << "Row " << row << std::endl;
            for (int col = neighborhoodSize + 5; col < harrisAccumulator.cols - neighborhoodSize - 5; ++col)
            {
                float sum = 0;
                for (int j = row - neighborhoodSize; j <= row + neighborhoodSize; ++j)
                {
                    for (int i = col - neighborhoodSize; i <= col + neighborhoodSize; ++i)
                    {
                        sum += scaleptr[j * scaledHarris.cols + i];
                    }
                }
                acuptr[row * harrisAccumulator.cols + col] = sum;
            }
        }
        std::cout << "finished harris accumulation" << std::endl;
        

        double globalMax = -std::numeric_limits<double>::max();
        double globalMin = std::numeric_limits<double>::max();

        for (const auto & marker : observations)
        {
            int topRow, botRow, leftCol, rightCol;
            topRow = std::max((int)marker.y - rad, 0);
            botRow = std::min((int)marker.y + rad, srcImage.rows - 1);
            leftCol = std::max((int)marker.x - rad, 0);
            rightCol = std::min((int)marker.x + rad, srcImage.cols - 1);

            cv::Point tl = cv::Point(leftCol, topRow);
            cv::Point br = cv::Point(rightCol, botRow);

            cv::Mat mask = cv::Mat::zeros(harrisAccumulator.size(), CV_8UC1);
            cv::rectangle(mask, cv::Rect(tl, br), cv::Scalar(1), CV_FILLED);

            // Get the local min and max with applied mask, ignore indicies
            double localMax = -std::numeric_limits<double>::max();
            double localMin = std::numeric_limits<double>::max();
            int * minIdx = NULL;
            int  *maxIdx = NULL;
            cv::minMaxIdx(harrisAccumulator, &localMin, &localMax, minIdx, maxIdx, mask);

            if (localMin != std::numeric_limits<double>::max())
                globalMin = std::min(globalMin, localMin);
            if (localMax != -std::numeric_limits<double>::max())
                globalMax = std::max(globalMax, localMax);

        }
        if (globalMin != std::numeric_limits<double>::max() && globalMax != -std::numeric_limits<double>::max())
            std::cout << "found flobal max and mins " << std::endl;
        else
            throw std::runtime_error("MarkerSanityCheckHarris: Failed to find global min/max!");

        double requiredHarrisVal = globalMax * 0.3;

        refinedMarkers = {};
        for (const auto & marker : observations)
        {
            // Increasing x from left to right
            // Increasing y from top to bottom
            int topRow, botRow, leftCol, rightCol;
            topRow = std::max((int)marker.y - rad, 0);
            botRow = std::min((int)marker.y + rad, srcImage.rows - 1);
            leftCol = std::max((int)marker.x - rad, 0);
            rightCol = std::min((int)marker.x + rad, srcImage.cols - 1);

            cv::Point tl = cv::Point(leftCol, topRow);
            cv::Point br = cv::Point(rightCol, botRow);

            cv::Mat mask = cv::Mat::zeros(harrisAccumulator.size(), CV_8UC1);
            cv::rectangle(mask, cv::Rect(tl, br), cv::Scalar(255), CV_FILLED);

            // Confirmed the following:
            // Increasing index from top-left to bottom-right
            // Indices come out row, col order
            double localMax = -std::numeric_limits<double>::max();
            double localMin = std::numeric_limits<double>::max();
            int  minIdx[3] = { -1, -1, -1 };
            int maxIdx[3] = { -1, -1, -1 };
            cv::minMaxIdx(harrisAccumulator, &localMin, &localMax, minIdx, maxIdx, mask);

            if (maxIdx[0] == -1 || maxIdx[1] == -1)
                throw std::runtime_error("MarkerSanityCheckHarris: Failed to find maxIdx");

            // Scale back to original image frame
            int maxX = maxIdx[1] * (scaledHarris.cols / harris.cols);
            int maxY = maxIdx[0] * (scaledHarris.rows / harris.rows);

            // Calculate distance in image frame
            float xDist = std::abs(marker.x - maxX);
            float yDist = std::abs(marker.y - maxY);

            

            if (localMax > requiredHarrisVal)
            {
                refinedMarkers.push_back(cv::Point2f((float)maxX, (float)maxY));
                if (xDist * xDist + yDist * yDist > 2.25)
                {
                    badMarkersFound = true;
                    badMarkers.push_back(marker);
                }
            }
            else
            {
                badMarkersFound = true;
                badMarkers.push_back(marker);
            }
        }

        std::cout << "Finished finding bad and refining markers" << std::endl;


        try
        {
            BoeingMetrology::Utilities::DrawCircles(badMarkerImage, badMarkers, 20, cv::Scalar(0, 0, 255));
            BoeingMetrology::Utilities::DrawCircles(badMarkerImage, badMarkers, 2, cv::Scalar(0, 0, 255));

            BoeingMetrology::Utilities::DrawCircles(badMarkerImage, refinedMarkers, 20, cv::Scalar(0, 255, 0));
            BoeingMetrology::Utilities::DrawCircles(badMarkerImage, refinedMarkers, 2, cv::Scalar(0, 255, 0));
        }
        catch (std::exception & e)
        {
            std::cout << "Exception in ArucoBoard::MarkerSanityCheckHarris - " << e.what();
        }

        std::cout << "Completed harris corner check" << std::endl;
    }

    void ArucoBoard::adjustLine(cv::Vec4f &toAdjust, std::vector<cv::Point2f> members, cv::Vec2f fitSlope)
    {
        cv::Vec2f origVec = cv::Vec2f(toAdjust[2], toAdjust[3]);
        cv::Vec2f perpindicular = cv::Vec2f(-fitSlope[1], fitSlope[0]);
        cv::Vec2f shift = cv::Vec2f();
        for(cv::Point2f p : members)
        {
            cv::Vec2f pVec = cv::Vec2f(p.x, p.y);
            cv::Vec2f offset = (pVec - origVec) - fitSlope*(pVec - origVec).dot(fitSlope);
            shift += offset;
        }
        cv::Vec2f shiftOrig = origVec + shift * (1.0 / members.size());
        toAdjust = cv::Vec4f(fitSlope[0], fitSlope[1], shiftOrig[0], shiftOrig[1]);
    }

    void ArucoBoard::JsonSerialize(Json::Value &jsonNode) const
    {
        jsonNode["imageWidth"] = imageWidth_;
        jsonNode["imageHeight"] = imageHeight_;
        jsonNode["borderWidth"] = borderWidth_;
        jsonNode["pixelSizeInPhysicalUnits"] = pixelSizeInPhysicalUnits_;
        jsonNode["squareSizeInPixels"] = squareSizeInPixels_;
        jsonNode["chessboardWidth"] = checkerboardSquareX_;
        jsonNode["chessboardHeight"] = checkerboardSquareY_;
        jsonNode["squareLength"] = squareLength_;
        jsonNode["markerLength"] = markerLength_;
        jsonNode["markerPattern"] = markerPattern_;

        // Calculated value to include for user inspection (not needed by JsonDeserialize).
        if (pixelSizeInPhysicalUnits_ > 0)
        {
            jsonNode["physicalSquareSizeOfDigitalBoard"] = getPhysicalSquareSizeOfDigitalBoard();
        }
    }

    void ArucoBoard::JsonDeserialize(const Json::Value &jsonNode)
    {
        imageWidth_ = jsonNode["imageWidth"].asInt();
        imageHeight_ = jsonNode["imageHeight"].asInt();
        borderWidth_ = jsonNode["borderWidth"].asInt();
        pixelSizeInPhysicalUnits_ = jsonNode["pixelSizeInPhysicalUnits"].asDouble();
        squareSizeInPixels_ = jsonNode["squareSizeInPixels"].asInt();
        checkerboardSquareX_ = jsonNode["chessboardWidth"].asInt();
        checkerboardSquareY_ = jsonNode["chessboardHeight"].asInt();
        squareLength_ = jsonNode["squareLength"].asFloat();
        markerLength_ = jsonNode["markerLength"].asFloat();
        markerPattern_ = static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(jsonNode["markerPattern"].asInt());

        // Initialize this->board_ and this->arucoDict_
        cv::Mat boardImg;
        this->ObjectGenerator(boardImg);
    }

    void ArucoBoard::setBoardParameters(int imageWidth, int imageHeight,
                                        int checkerboardSquareX, int checkerboardSquareY, 
                                        int borderWidth, float squareLength, float markerLength, 
                                        cv::aruco::PREDEFINED_DICTIONARY_NAME markerPattern,
                                        float pixelSizeInPhysicalUnits /*= -1 (>0 indicates digital board) */)
    {
        imageWidth_ = imageWidth;
        imageHeight_ = imageHeight;
        checkerboardSquareX_ = checkerboardSquareX;
        checkerboardSquareY_ = checkerboardSquareY;
        borderWidth_ = borderWidth;
        squareLength_ = squareLength;
        markerLength_ = markerLength;
        markerPattern_ = markerPattern;
        pixelSizeInPhysicalUnits_ = pixelSizeInPhysicalUnits;
        squareSizeInPixels_ = -1;

        if (pixelSizeInPhysicalUnits_ > 0)
        {
            squareSizeInPixels_ = CharucoBoardSquareInPixels(
                cv::Size(imageWidth_, imageHeight_), borderWidth_,
                checkerboardSquareX_, checkerboardSquareY_, squareLength_, markerLength_);
        }
        else
        {
            squareSizeInPixels_ = -1;
        }
    }

    void ArucoBoard::SerializeStream(std::ostream &strm) const
    {
        // Camera info
        Json::Value root;
        JsonSerialize(root);
        Json::StyledStreamWriter json_writer;
        json_writer.write(strm, root);
    }

    void ArucoBoard::DeserializeStream(std::istream &strm)
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
            throw std::runtime_error("ArucoBoard::DeserializeStream(): Failed to parse json.");
        }
    }

    void ArucoBoard::SerializeFile(const std::string &fileName) const
    {
        //Write the serialization to the file
        std::ofstream strm(fileName);
        SerializeStream(strm);
        strm.close();
    }

    void ArucoBoard::DeserializeFile(const std::string &fileName)
    {
        //Read the content from the file and de-serialize into the class
        std::ifstream strm(fileName);
        DeserializeStream(strm);
        strm.close();
    }

}//namespace CalibrationObject
}//namespace Calibration
}//namespace BoeingMetrology
