#include "ImageProcessing/ImageCharacteristics.h"
#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iomanip>
#include "Utilities/Utilities.h"

#ifdef WIN32
#include <Windows.h>
#endif

double BoeingMetrology::ImageCharacteristics::FocusMeasure::normalizedGraylevelVariance(const cv::Mat& src)
{
	cv::Scalar mu, sigma;
	cv::meanStdDev(src, mu, sigma);

	double focusMeasure = (sigma.val[0] * sigma.val[0]) / mu.val[0];
	return focusMeasure;
}

double BoeingMetrology::ImageCharacteristics::FocusMeasure::bimodalVariance(const cv::Mat & src, std::pair<double, double> & meanStd1, 
    std::pair<double, double> & meanStd2, cv::Mat & threshMask, const cv::Mat mask /* = cv::Mat() */, const double & lowerPerc /*= 0.1 */, const double & upperPerc /* = 0.9 */)
{
    // Validate inputs
    if (src.rows == 0 || src.cols == 0)
        throw std::runtime_error("bimodalVariance: Input image is empty");
    cv::Mat srcGrey = src;
    if (src.type() == CV_8UC3)
    {
        std::cout << "FocusMeasure::bimodalVariance: converting image to greyscale" << std::endl;
        cv::cvtColor(src, srcGrey, cv::COLOR_BGR2GRAY);
    }
    else if (src.type() != CV_8UC1)
        throw std::runtime_error("bimodalVariance: Input image is not 8UC1");

    // Determine percentiles.  Optionally ignore masked pixels.
    cv::Mat hist, cdf;
    Utilities::imageCDF(srcGrey, 256, 0, 255, hist, cdf, mask);
    int lowerPercIdx = -1;
    int upperPercIdx = 255;
    for (int i = 0; i <= 256 - 1; i++){
        if (lowerPercIdx == -1 && cdf.at<float>(i) >= lowerPerc)
            lowerPercIdx = i;
        if (cdf.at<float>(i) >= upperPerc)
        {
            upperPercIdx = i;
            break;
        }
    }

    // Clone the image and truncate statistical outliers
    std::pair<int, int> bounds = { lowerPercIdx, upperPercIdx };
    cv::Mat outliersremoved = srcGrey.clone();
    outliersremoved.setTo(bounds.first + 1, outliersremoved < bounds.first);
    outliersremoved.setTo(bounds.second - 1, outliersremoved > bounds.second);

    // Optionally set background pixels to the midpoint so it's ignored during normalization
    if (mask.rows > 0 && mask.cols > 0)
        outliersremoved.setTo(0.5 * (bounds.first + bounds.second), ~mask);
    
    // Normalize the truncated image so pixels range from 0 to 255
    cv::Mat srcnorm;
    cv::normalize(outliersremoved, srcnorm, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // Optionally set background pixels to 127
    if (mask.rows > 0 && mask.cols > 0)
        outliersremoved.setTo(127, ~mask);

    // Create a mask of pixels greater than 128
    cv::Mat righthandMask;
    cv::threshold(srcnorm, righthandMask, 128, 255, cv::THRESH_BINARY);

    cv::Mat lefthandMask = ~righthandMask;
    if (mask.rows > 0 && mask.cols > 0)
        lefthandMask = ~(righthandMask | ~mask);

    // Mean, std of "left-hand" mode
    cv::Scalar mean1, sd1;
    cv::meanStdDev(srcnorm, mean1, sd1, lefthandMask);

    // Mean, std of "right-hand" mode
    cv::Scalar mean2, sd2;
    cv::meanStdDev(srcnorm, mean2, sd2, righthandMask);

    //std::cout << "mean1, sd1, mean2, sd2" << std::endl;
    //std::cout << mean1 << ", " << sd1 << ", " << mean2 << ", " << sd2 << std::endl;
    meanStd1 = { mean1[0], sd1[0] };
    meanStd2 = { mean2[0], sd2[0] };

    // The output mask informs the user if the high-contrast target has been properly masked and thresholded into black and white.  
    // For a black-and-white chessboard, this mask should be entirely black except the white squares.  
    threshMask = lefthandMask;

    return (0.5 * (sd1[0] + sd2[0]));
}

double BoeingMetrology::ImageCharacteristics::FocusMeasure::tenengrad(const cv::Mat& src, int ksize)
{
	cv::Mat Gx, Gy;
	cv::Sobel(src, Gx, CV_64F, 1, 0, ksize);
	cv::Sobel(src, Gy, CV_64F, 0, 1, ksize);

	cv::Mat FM = Gx.mul(Gx) + Gy.mul(Gy);

	double focusMeasure = cv::mean(FM).val[0];
	return focusMeasure;
}

double BoeingMetrology::ImageCharacteristics::FocusMeasure::varianceOfLaplacian(const cv::Mat& src)
{
	cv::Mat lap;
	cv::Laplacian(src, lap, CV_64F);

	cv::Scalar mu, sigma;
	cv::meanStdDev(lap, mu, sigma);

	double focusMeasure = sigma.val[0] * sigma.val[0];
	return focusMeasure;
}

double BoeingMetrology::ImageCharacteristics::FocusMeasure::modifiedLaplacian(const cv::Mat& src)
{
	cv::Mat M = (cv::Mat_<double>(3, 1) << -1, 2, -1);
	cv::Mat G = cv::getGaussianKernel(3, -1, CV_64F);

	cv::Mat Lx;
	cv::sepFilter2D(src, Lx, CV_64F, M, G);

	cv::Mat Ly;
	cv::sepFilter2D(src, Ly, CV_64F, G, M);

	cv::Mat FM = cv::abs(Lx) + cv::abs(Ly);

	double focusMeasure = cv::mean(FM).val[0];
	return focusMeasure;
}


BoeingMetrology::ImageCharacteristics::PoseDiagnostics::PoseDiagnostics(const double & focusMetricIn,
	const double & hv_ExposureScoreIn, const double & hv_HomogeneityScoreIn, const double & hv_ContrastScoreIn, 
	const double & hv_SizeMarksIn, const std::string & hv_MessageIn, const double & hv_FocusScoreIn, 
	const std::vector<double> & rowIn, const std::vector<double> & colIn, const std::vector<double> & indexIn,
	const std::vector<double> & poseIn, const std::vector<int> & poseIndexIn)
{
	this->focusMetric = focusMetricIn;
	this->hv_ExposureScore = hv_ExposureScoreIn;
	this->hv_HomogeneityScore = hv_HomogeneityScoreIn;
	this->hv_ContrastScore = hv_ContrastScoreIn;
	this->hv_SizeMarks = hv_SizeMarksIn;
	this->hv_Message = hv_MessageIn;
	this->hv_FocusScore = hv_FocusScoreIn;
	this->row = rowIn;
	this->col = colIn;
	this->index = indexIn;
	this->pose = poseIn;
	this->poseIndex = poseIndexIn;
}

bool BoeingMetrology::ImageCharacteristics::PoseDiagnostics::Deserialize(const std::string & jsonFname)
{
	// Read the json
	Json::Value root;
	Json::Reader reader;
	std::ifstream fReader(jsonFname, std::ifstream::binary);
	if (reader.parse(fReader, root))
	{
		try
		{
			hv_Message = root["hv_Message"].asString();
			if (hv_Message.size() > 0)
			{
				focusMetric = root["focusMetric"].asDouble();
				hv_ExposureScore = root["hv_ExposureScore"].asDouble();
				hv_HomogeneityScore = root["hv_HomogeneityScore"].asDouble();
				hv_ContrastScore = root["hv_ContrastScore"].asDouble();
				hv_SizeMarks = root["hv_SizeMarks"].asDouble();
				hv_FocusScore = root["hv_FocusScore"].asDouble();
				Json::Value observationPoints = root["observationPoints"];
				for (auto & pt : observationPoints)
				{
					row.push_back(pt["row"].asDouble());
					col.push_back(pt["col"].asDouble());
					index.push_back(pt["index"].asDouble());
				}
				Json::Value posePoints = root["posePoints"];
				for (auto & pt : posePoints)
				{
					pose.push_back(pt["pose"].asDouble());
					poseIndex.push_back(pt["poseIndex"].asInt());
				}
				return true;
			}
			else
				return false;
		}
		catch (...)
		{
			return false;
		}
	}
	else
		return false;
}

bool BoeingMetrology::ImageCharacteristics::PoseDiagnostics::SerializeAndWrite(const std::string & jsonFname, const Json::Value & objpoints_root)
{

	Json::Value diagInfo;
	diagInfo["object3DPoints"] = objpoints_root;
	diagInfo["focusMetric"] = this->focusMetric;
	diagInfo["hv_ExposureScore"] = this->hv_ExposureScore;
	diagInfo["hv_HomogeneityScore"] = this->hv_HomogeneityScore;
	diagInfo["hv_ContrastScore"] = this->hv_ContrastScore;
	diagInfo["hv_SizeMarks"] = this->hv_SizeMarks;
	diagInfo["hv_Message"] = this->hv_Message;
	diagInfo["hv_FocusScore"] = this->hv_FocusScore;

	Json::Value pointList;
	for (int i = 0; i < row.size(); ++i)
	{
		Json::Value calibrationObs;
		calibrationObs["row"] = row[i];
		calibrationObs["col"] = col[i];
		calibrationObs["index"] = index[i];
		pointList.append(calibrationObs);
	}
	diagInfo["observationPoints"] = pointList;

	Json::Value poseList;
	for (int i = 0; i < pose.size(); ++i)
	{
		Json::Value calibrationObs;

		calibrationObs["pose"] = pose[i];
		calibrationObs["poseIndex"] = poseIndex[i];
		poseList.append(calibrationObs);
	}
	diagInfo["posePoints"] = poseList;


	Json::StyledWriter styledWriter;
	std::ofstream writer(jsonFname, std::ifstream::binary);
	writer << styledWriter.write(diagInfo);
	writer.close();

	return true;
}

void BoeingMetrology::ImageCharacteristics::PoseDiagnosticsGroup::ReportFeatureExtrema()
{
	// Loop through sensors
	std::cout << "Pose diagnostics extrema " << std::endl;
	for (auto sensor : this->data)
	{
		// Sensor name
		std::cout << sensor.first << std::endl;

		// Sort by focus measure
		std::sort(sensor.second.begin(), sensor.second.end(),
			[](const std::pair<PoseDiagnostics, std::string> & a, const std::pair<PoseDiagnostics, std::string> & b) -> bool
		{
			return a.first.focusMetric > b.first.focusMetric;
		});

		std::cout << "focusMeasure extreme poses: " << sensor.second.front().second << ", " << sensor.second.back().second << std::endl;

		// Sort by contrast score
		std::sort(sensor.second.begin(), sensor.second.end(),
			[](const std::pair<PoseDiagnostics, std::string> & a, const std::pair<PoseDiagnostics, std::string> & b) -> bool
		{
			return a.first.hv_ContrastScore > b.first.hv_ContrastScore;
		});

		std::cout << "ContrastScore extreme poses: " << sensor.second.front().second << ", " << sensor.second.back().second << std::endl;

		// Sort by exposure score
		std::sort(sensor.second.begin(), sensor.second.end(),
			[](const std::pair<PoseDiagnostics, std::string> & a, const std::pair<PoseDiagnostics, std::string> & b) -> bool
		{
			return a.first.hv_ExposureScore > b.first.hv_ExposureScore;
		});

		std::cout << "ExposureScore extreme poses: " << sensor.second.front().second << ", " << sensor.second.back().second << std::endl;

		// Sort by homogeneity score
		std::sort(sensor.second.begin(), sensor.second.end(),
			[](const std::pair<PoseDiagnostics, std::string> & a, const std::pair<PoseDiagnostics, std::string> & b) -> bool
		{
			return a.first.hv_HomogeneityScore > b.first.hv_HomogeneityScore;
		});

		std::cout << "Homogeneity extreme poses: " << sensor.second.front().second << ", " << sensor.second.back().second << std::endl;

		// Sort by size marks
		std::sort(sensor.second.begin(), sensor.second.end(),
			[](const std::pair<PoseDiagnostics, std::string> & a, const std::pair<PoseDiagnostics, std::string> & b) -> bool
		{
			return a.first.hv_SizeMarks > b.first.hv_SizeMarks;
		});

		std::cout << "hv_SizeMarks extreme poses: " << sensor.second.front().second << ", " << sensor.second.back().second << std::endl;

		// Testing only
		for (const auto & pose : sensor.second)
			std::cout << pose.first.focusMetric << std::endl;

		// Create a map of messages
		std::cout << "Unique messages" << std::endl;
		std::map<std::string, std::string> messageMap;
		for (const auto & pose : sensor.second)
			messageMap[pose.first.hv_Message] = pose.second;
		for (const auto& uniqueMsg : messageMap)
			std::cout << uniqueMsg.second << ": " << uniqueMsg.first << std::endl;
	}

}

void BoeingMetrology::ImageCharacteristics::PoseDiagnosticsGroup::ExportFeatureCSV(const std::string & fname)
{
	// Open out file
	std::ofstream ofile;
	ofile.open(fname);

	// Write out a header
	ofile << "sensorName, poseDir, focusMetric, hv_ExposureScore, hv_HomogeneityScore, v_ContrastScore, hv_SizeMarks, hv_Message, hv_FocusScore" << std::endl;
	
	// Loop through sensors
	for (const auto & sensor : this->data)
	{
		// Loop through poses
		for (const auto & pose : sensor.second)
		{
			// Write out this pose feature row
			ofile << sensor.first << ", " << pose.second << ", ";
			ofile << pose.first.focusMetric << ", ";
			ofile << pose.first.hv_ExposureScore << ", ";
			ofile << pose.first.hv_HomogeneityScore << ", ";
			ofile << pose.first.hv_ContrastScore << ", ";
			ofile << pose.first.hv_SizeMarks << ", ";
			ofile << pose.first.hv_Message << ", ";
			ofile << pose.first.hv_FocusScore << std::endl;
		}
	}
	ofile.close();
}

void BoeingMetrology::ImageCharacteristics::PoseDiagnosticsGroup::InsertPose(const std::string & poseDir, const std::string & sensorName, const PoseDiagnostics & diag)
{
	if (this->data.find(sensorName) == this->data.end())
	{
		// Create new key
		std::vector<std::pair<BoeingMetrology::ImageCharacteristics::PoseDiagnostics, std::string>> newpair = { { diag, poseDir } };
		this->data[sensorName] = newpair;
	}
	else
	{
		// Append to existing key
		std::vector<std::pair<BoeingMetrology::ImageCharacteristics::PoseDiagnostics, std::string>> * existinglist = &this->data[sensorName];
		existinglist->push_back({ diag, poseDir });
	}
}
