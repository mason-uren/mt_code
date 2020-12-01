#ifndef rochade_h
#define rochade_h

#include "opencv2/opencv.hpp"
#include "../BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace ImageProcessing
	{
		cv::Mat BOEINGMETROLOGYLIB_API RochadeResizing(cv::Mat im, double downSamplingThreshold, double& upSampleFac, double& downSampleFac);
		cv::Mat BOEINGMETROLOGYLIB_API RochadeLocalThresholding(cv::Mat edgeImage, double localRelativeThreshold);
		cv::Mat BOEINGMETROLOGYLIB_API Scharr(cv::Mat inputImage);
		cv::Mat BOEINGMETROLOGYLIB_API fillHolesAndNotches(cv::Mat inputMask, int halfSize, int minNrSetPixels, int foregroundLevel = 255);
		cv::Mat BOEINGMETROLOGYLIB_API removeBlobs(cv::Mat maskIm, double percentage);
		cv::Mat BOEINGMETROLOGYLIB_API removeNonloop(cv::Mat skeletonMask);
		cv::Mat BOEINGMETROLOGYLIB_API findSaddlePointCandidates(cv::Mat skeletonMask);
		void BOEINGMETROLOGYLIB_API combineSaddles(cv::Mat edgeMask, cv::Mat saddleCandidates, int saddleCombinationHalfSize, cv::Mat& combinedSaddleMask,
			cv::Mat& saddleLabelImage);
		void BOEINGMETROLOGYLIB_API RochadeRefine(cv::Mat imageInput, std::vector<cv::Point2f> initialcorners, int halfSizePatch,
			std::vector<cv::Point2f>& refinedcorners, std::vector<bool>& refinementSuccessful);
		bool BOEINGMETROLOGYLIB_API verifyCheckerboard(cv::Mat removeNonloopMask, cv::Mat saddleLabelImage, cv::Mat combinedSaddleMask, int ncornerX, int ncornerY,
			double downSampleFac, std::vector<cv::Point2f>& initialcorners);
		void BOEINGMETROLOGYLIB_API RochadeCornerVisualize(cv::Mat im, std::vector<cv::Point2f> corners, cv::Mat& imcorners);
	}
}

#endif