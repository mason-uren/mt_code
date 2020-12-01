/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    BrightnessMeasure.cpp
*  @author  Brittany Haffner
*
*  @brief Used to quantify brightness of an image
*
*  @note This class is used in BrightnessMeasureOp workspace plugin
*
*****************************************************************************/

#include "ImageProcessing/BrightnessMeasure.h"
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

void printTextCentered(cv::InputOutputArray img, cv::String text, cv::Point nonCenteredOrigin, int fontFace, double fontScale, cv::Scalar color, int thickness = 1) {
	int baseline;
	cv::Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);

	cv::Point centeredOrigin(nonCenteredOrigin.x - textSize.width / 2, nonCenteredOrigin.y + textSize.height / 2);

	putText(img, text, centeredOrigin, fontFace, fontScale, color, thickness);
}
/*
	input: any image
	output: gray (intensity) image
	description: take any input image (gray, RGB, BGR, etc.) and convert to gray scale for BrightnessMeasure functions;
	unable to use "ConvertColor" plugin uniformly across JPEG and PNG images (if already gray scale)
*/
void BoeingMetrology::ImageCharacteristics::BrightnessMeasure::cvtToGray(const cv::Mat& src, cv::Mat& grayImg)
{
	if (src.channels() == 3) {
		cv::cvtColor(src, grayImg, cv::COLOR_BGR2GRAY, 0);
	} 
	else {
		grayImg = src.clone();
	}
	
}

/*
    input: two intensity (grayscale) images
    output: absolute difference between the two images
    purpose: visualize differences between "same" image (two masks of same image) 
*/
void BoeingMetrology::ImageCharacteristics::BrightnessMeasure::imageCompare(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& output) 
{
    cv::Mat grayImg1;
    cv::Mat grayImg2;
    BrightnessMeasure::cvtToGray(img1, grayImg1);
    BrightnessMeasure::cvtToGray(img2, grayImg2);

    // since only looking for overall difference, use absdiff
    cv:: absdiff(grayImg1, grayImg2, output);
}

/*
	input: grayscale (intensity) image
	output: plot of histogram
	description: function to look at a histogram of the pixel intensity of an image
*/
void BoeingMetrology::ImageCharacteristics::BrightnessMeasure::intensityHistogram(const cv::Mat& src, cv::Mat& histImage)
{
	cv::Mat grayImg;
	BrightnessMeasure::cvtToGray(src, grayImg);

	// cited source: https://github.com/GeorgeSeif/Image-Processing-OpenCV/blob/master/Histogram%20Equalization/Source.cpp
	// set up parameters for histogram calculation
	int histSize = 256;
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true; bool accumulate = false;
	cv::Mat hist; //empty histogram
	// compute histogram
	calcHist(&grayImg, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

	// margin on all sides for scale
	const int margin = 100;

	// Draw the histogram
	int hist_w = 672; int hist_h = 500;
	double bin_w = (double)(hist_w - 2 * margin) / histSize;

	histImage = cv::Mat(hist_h, hist_w, CV_8UC3, cv::Scalar(255, 255, 255));

	// Find highest value in histogram (must be before normalization)
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(hist, &minVal, &maxVal, &minLoc, &maxLoc);

	// Normalize the result to [ 0, histImage.rows ]
	normalize(hist, hist, margin, histImage.rows - margin, cv::NORM_MINMAX, -1, cv::Mat());

	// y axis and grid lines
	const int textCenterXPos = margin / 2;
	const int textMinY = hist_h - margin;
	const int textMaxY = margin;
	const int numStepsY = 10;
	const int stepY = (textMinY - textMaxY) / numStepsY;

	for (int textYPos = textMinY; textYPos >= textMaxY; textYPos -= stepY) {
		cv::line(histImage, cv::Point(margin - 10, textYPos), cv::Point(hist_w - margin, textYPos), cv::Scalar(165, 165, 165));
		const double currentScaleValue = (double)maxVal * (textMinY - textYPos) / (hist_h - 2 * margin);

		// make scientific notation string
		std::ostringstream valueStream;
		valueStream << std::scientific << std::setprecision(2) << currentScaleValue;
		std::string valueAsString = valueStream.str();

		// remove unnecessary zeros in exponent
		for (int i = 6; i <= valueAsString.length(); i++) {
			if (valueAsString[i] == '0') {
				valueAsString.erase(i, 1);
				i -= 1; // lost a member, so cancel out the next incrementation
			}
		}

		// remove unnecessary exponent notation (e+000)
		if (valueAsString.length() == 6) {
			valueAsString.erase(4, 2);
		}

		printTextCentered(histImage, valueAsString, cv::Point(textCenterXPos, textYPos), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 0), 1);
	}

	// x axis and grid lines
	const int textCenterYPos = hist_h - margin + 25;
	const int textMinX = margin;
	const int textMaxX = hist_w - margin;

	for (int x = 0; x <= 10; x++) {
		int xValue = margin + cvRound(x * (textMaxX - textMinX) / 10);
		cv::line(histImage, cv::Point(xValue, margin), cv::Point(xValue, hist_h - margin + 10), cv::Scalar(165, 165, 165));
		printTextCentered(histImage, std::to_string(cvRound(255.0 / 10.0 * x)), cv::Point(xValue, textCenterYPos), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 0), 1);
	}

	// border
	cv::rectangle(histImage, cv::Point(margin, margin), cv::Point(hist_w - margin, hist_h - margin), cv::Scalar(0, 0, 0));

	// draw histogram
	for (int i = 1; i < histSize; i++)
	{
		line(histImage, cv::Point(cvRound(bin_w*(i - 1) + margin), hist_h - cvRound(hist.at<float>(i - 1))),
			cv::Point(cvRound(bin_w*(i)) + margin, hist_h - cvRound(hist.at<float>(i))),
			cv::Scalar(0, 0, 255), 2, 8, 0);
	}
}


/*
input: grayscale (intensity) image
output: percentage of pixels that are below the threshold for ambient light & B/W masked image based on threshold
description: function to determine how many pixels are dark
*/
double BoeingMetrology::ImageCharacteristics::BrightnessMeasure::percentageDark(const cv::Mat& src, int ambientThresh, cv::Mat& imageMask)
{
	cv::Mat grayImg;
	BrightnessMeasure::cvtToGray(src, grayImg);

	//Binary thresholding: anything above threshold is MAX, everything else is 0
	double maxPixelVal = 255;
	cv::threshold(grayImg, imageMask, ambientThresh, maxPixelVal, CV_THRESH_BINARY);
	size_t cntDark = imageMask.total() - cv::countNonZero(imageMask);

	double fracDark = (double)cntDark / (double)src.total();
	double percDark = fracDark * 100;

	return percDark;
}

/*
input: grayscale (intensity) image
output: redrawn image with saturated pixels marked as red and number of saturated pixels
description: function to look at saturated portions of an image
*/
int BoeingMetrology::ImageCharacteristics::BrightnessMeasure::pixelsSaturated(const cv::Mat& src, cv::Mat& saturated)
{
	cv::Mat grayImg;
	BrightnessMeasure::cvtToGray(src, grayImg);

	//convert grayscale image to RGB for displaying red pixels
	cv::cvtColor(grayImg, saturated, cv::COLOR_GRAY2BGR, 0);
	// set threshold for saturation definition
	int satThresh = 250; // out of 255
	int cntSat = 0; // for counting number of saturated pixels
	// loop through image matrix to change saturated pixels to red
	for (int row = 0; row < saturated.rows; row++) {
		for (int col = 0; col < saturated.cols; col++) {
			cv::Vec3b color = saturated.at<cv::Vec3b>(row, col);
			//if pixel is saturated, change its color to red
			if (color[0] > satThresh || color[1] > satThresh || color[2] > satThresh) {
				// update color vector to red
				color[0] = 0;
				color[1] = 0;
				color[2] = 255;
				// store color vector to pixel
				saturated.at<cv::Vec3b>(row, col) = color; 
				// update counter of saturated pixels
				cntSat++;
			}
		}
	}

	return cntSat;
}

/*
input: grayscale (intensity) image
output: redrawn image with B/W determined by Otsu's method and return Otsu's threshold
description: function to determine how a machine would see B/W via Otsu's method
*/
double BoeingMetrology::ImageCharacteristics::BrightnessMeasure::otsuMask(const cv::Mat& src, cv::Mat& maskImg)
{
    // make sure input image is grayscale for running otsuMask in its own operation
    cv::Mat grayImg;
    BrightnessMeasure::cvtToGray(src, grayImg);

	double minPixelVal = 0;
	double maxPixelVal = 255;
	double otsuThreshold = cv::threshold(grayImg, maskImg, minPixelVal, maxPixelVal, CV_THRESH_BINARY | CV_THRESH_OTSU);
	return otsuThreshold;
}

/*
([WHITE PIXELS] - [AMBIENT PIXELS])/([255] - [AMBIENT PIXELS])
*/
double BoeingMetrology::ImageCharacteristics::BrightnessMeasure::intensityVsAmbient(const cv::Mat& ambient, const cv::Mat& white,
	cv::Mat& diffScaled, cv::Mat& mask)
{
	// make sure input images are grayscale 
	cv::Mat grayAmbient;
	BrightnessMeasure::cvtToGray(ambient, grayAmbient);
	cv::Mat grayWhite;
	BrightnessMeasure::cvtToGray(white, grayWhite);

	// create a matrix of max pixel values
	double maxPixelVal = 255;  //Note: probably need to make this a global somewhere
	cv::Mat maxPixels = cv::Mat::ones(grayAmbient.rows, grayAmbient.cols, CV_8U)*maxPixelVal;

	cv::Mat numerator = grayWhite - grayAmbient;
	cv::Mat denominator = maxPixels - grayAmbient;
	cv::divide(numerator, denominator, diffScaled, 100);  //scaling by 100%

	// create a mask from diffScaled and return threshold value
	return BrightnessMeasure::otsuMask(diffScaled, mask);
}

/*
input: grayscale (intensity) image
output: redrawn image with B/W determined by Otsu's method, "black" as true black, and red as saturated pixels.
Image text includes Otsu's threshold as optimized threshold and percentage "white" pixels that are saturated.
Also returns Otsu's threshold as a double for quick reference.
description: function to determine how a machine would see B/W via Otsu's method
*/
void BoeingMetrology::ImageCharacteristics::BrightnessMeasure::brightnessSummary(std::string pathName, const cv::Mat& src, cv::Mat& output,
	double& otsuThreshold, double& percentSaturated)
{
	cv::Mat grayImg;
	BrightnessMeasure::cvtToGray(src, grayImg);

	double maxPixelVal = 255;
	cv::Mat maskImg;
	cv::Mat threshImg;
    cv::Mat dummy;

	otsuThreshold = BrightnessMeasure::otsuMask(grayImg, maskImg);
    cv::threshold(grayImg, threshImg, otsuThreshold, maxPixelVal, CV_THRESH_TOZERO);
    double percentBlack = BrightnessMeasure::percentageDark(grayImg, (int)otsuThreshold, dummy);
	int numSaturated = BrightnessMeasure::pixelsSaturated(threshImg, output);

	double percentWhite = 100 - percentBlack;
	double numWhite = (double)percentWhite / 100 * output.total();
	percentSaturated = (double)numSaturated / numWhite * 100;
	
	std::string threshold = "Optimal threshold: " + std::to_string(otsuThreshold);
	std::string saturated = "% White pixels saturated: " + std::to_string(percentSaturated);

	cv::putText(output, threshold, cv::Point(100, output.rows - 200), CV_FONT_HERSHEY_COMPLEX, 4, 
		cv::Scalar(200, 200, 0), 2, 8, false);
	cv::putText(output, saturated, cv::Point(100, output.rows - 100), CV_FONT_HERSHEY_COMPLEX, 4,
		cv::Scalar(200, 200, 0), 2, 8, false);

	if (!pathName.empty()){

#ifdef WIN32
		// Make a directory to hold the output files
		std::wstring wstrPath = std::wstring(pathName.begin(), pathName.end());
		const wchar_t* wcharPath = wstrPath.c_str();
		_wmkdir(wcharPath);
#endif
		// Write summary image to PNG
		std::string imgName = pathName + "/BrightnessSummary.png";
		cv::imwrite(imgName, output);
		// Write data to text file
		std::string filename = pathName + "/BrightnessSummary.txt";
		std::ofstream ofile;
		ofile.open(filename, std::ios::ios_base::app);
		ofile << "Optimal/Otsu threshold: " << threshold << std::endl;
		ofile << "% White pixels saturated: " << saturated << std::endl;
		ofile.close();
	}
}
