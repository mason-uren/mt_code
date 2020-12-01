#include <time.h>
#include <iomanip>
#include "Utilities.h"
#include <sstream>
#include <regex>
#include "opencv2/imgproc.hpp"

std::string BoeingMetrology::Utilities::zeroPadNumber(const int & src, const int & dstWidth)
{
	std::ostringstream ss;
	ss << std::setw(dstWidth) << std::setfill('0') << src;
	return ss.str();
}

std::string BoeingMetrology::Utilities::getCurrentTimeInSeconds()
{
	time_t t = time(0);
	struct tm now;
#ifdef _MSC_VER
		localtime_s(&now, &t);
#else
	// https://en.cppreference.com/w/c/chrono/localtime
	now = *localtime(&t);
#endif
	std::string dateStr = std::to_string(now.tm_year + 1900) + zeroPadNumber(now.tm_mon + 1, 2) +
		zeroPadNumber(now.tm_mday, 2) + "_" + zeroPadNumber(now.tm_hour, 2) +
		zeroPadNumber(now.tm_min, 2) + zeroPadNumber(now.tm_sec, 2);

	return dateStr;
}

bool BoeingMetrology::Utilities::RegexMatch(const REGEX_STRING& src, const std::string& dest)
{
	if (std::regex_match(dest, std::regex(src)))
		return true;
	else
		return false;
}

void BoeingMetrology::Utilities::DrawCircles(cv::Mat & img, const std::vector<std::vector<cv::Point2f>> & pts, const int & radius, const cv::Scalar & color, int thickness)
{
    for (const auto & vec : pts)
    {
        for (const auto & pt : vec)
        {
            cv::circle(img, pt, radius, color, thickness);
        }
    }
}

void BoeingMetrology::Utilities::imageCDF(const cv::Mat & src, const int & numBins, const int & minBin, const int & maxBin, cv::Mat & hist, cv::Mat & cdf, const cv::Mat mask /* = cv::Mat() */)
{
    // CV_32F Histogram of a single channel
    float range[] = { (float)minBin, (float)maxBin };
    const float* histRange = { range };
    bool uniform = true;
    bool accumulate = false;
    cv::calcHist(&src, 1, 0, mask, hist, 1, &numBins, &histRange, uniform, accumulate);

    // CDF
    hist.copyTo(cdf);
    for (int i = 1; i <= numBins - 1; i++)
    {
        cdf.at<float>(i) += cdf.at<float>(i - 1);
    }
    cdf /= cdf.at<float>(numBins - 1);
}

void BoeingMetrology::Utilities::DrawCircles(cv::Mat & img, const std::vector<cv::Point2f> & pts, int radius, cv::Scalar color, int thickness)
{
    for (const auto & pt : pts)
        cv::circle(img, pt, radius, color, thickness);
}