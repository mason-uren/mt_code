#ifndef BOEINGMETROLOGYLIB_UTILITIES_H
#define BOEINGMETROLOGYLIB_UTILITIES_H

#include <string>
#include "json/json.h"
#include "TypeDefs.h"
#include <cv.h>
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    class BOEINGMETROLOGYLIB_API Utilities
	{
	public:

		// Int to string with zero-padding
		static std::string zeroPadNumber(const int & src, const int & dstWidth);

		// YYYYMMDD_HHMMSS
		static std::string getCurrentTimeInSeconds();
		
		static bool RegexMatch(const REGEX_STRING& src, const std::string& dest);

        // Draw circles surrounding points
        static void DrawCircles(cv::Mat & img, const std::vector<std::vector<cv::Point2f>> & pts, const int & radius = 25, const cv::Scalar & color = cv::Scalar(0, 0, 255, 0), int thickness = 5);
        static void DrawCircles(cv::Mat & img, const std::vector<cv::Point2f> & pts, int radius = 25, cv::Scalar color = cv::Scalar(0, 0, 255, 0), int thickness = 5);

        // Compute a histogram of pixels and the corresponding CDF for an 8-bit greyscale image.  hist and cdf are both 1D matrices of type CV_32F.
        static void imageCDF(const cv::Mat & src, const int & numBins, const int & minBin, const int & maxBin, cv::Mat & hist, cv::Mat & cdf, const cv::Mat mask = cv::Mat());
	};
}

#endif
