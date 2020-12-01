/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    BrightnessMeasure.h
*  @author  Brittany Haffner
*
*  @brief Used to quantify brightness of an image
*
*  @note This class is used in BrightnessMeasureOp workspace plugin
*
*****************************************************************************/

#pragma once
#include <vector>
#include <map>
#include <cv.h>
#include "json/json.h"

#include "BoeingMetrologyLib_API.h"


namespace BoeingMetrology
{
	namespace ImageCharacteristics
	{		
		class BOEINGMETROLOGYLIB_API BrightnessMeasure
		{
		public:
			// ==============================================================================
			// \fn:       Boeing::ImageCharacteristics::BrightnessMeasure::cvtToGray
			// \brief:    Take any input image and convert it to grayscale for use with 
			//					open cv thresholding in other BrightnessMeasure functions
			//
			// @param     const cv::Mat& src  - source image to convert
			// @param     cv::Mat& grayImg - output grayscale image
			// @return    void                            - but grayImg is modified
			// ==============================================================================
			void cvtToGray(const cv::Mat& src, cv::Mat& grayImg);

            // ==============================================================================
            // \fn:       Boeing::ImageCharacteristics::BrightnessMeasure::imageCompare
            // \brief:    Take two input images and do a pixel-to-pixel comparison (diff), 
            //            specifically meant to compare image masks to visualize differences
            //
            // @param     const cv::Mat& img1 - first input image for comparison
            // @param     const cv::Mat& img2 - second input image for comparison
            // @param     cv::Mat& output - output image that is the diff between img1 & img2
            // @return    void                            - but output is modified
            // ==============================================================================
            void imageCompare(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& output);

			// ==============================================================================
			// \fn:       Boeing::ImageCharacteristics::BrightnessMeasure::intensityHistogram
			// \brief:    plot a histogram of the image intensity for an image
			//
			// @param     const cv::Mat& src  - source image to convert
			// @param     cv::Mat& histImage - output intensity histogram image
			// @return    void                            - but histImage is modified
			// ==============================================================================
			void intensityHistogram(const cv::Mat& src, cv::Mat& histImage);

			// ==============================================================================
			// \fn:       Boeing::ImageCharacteristics::BrightnessMeasure::percentageDark
			// \brief:    Determine the percentage of pixels that are "dark" or black 
			//					based on a threshold and return corresponding B/W image mask
			//
			// @param     const cv::Mat& src  - source image to convert to gray scale
			// @param	  int ambientThresh - user input ambient threshold for B/W distinction
			// @param     cv::Mat& maskedImg - output mask based on ambient threshold
			// @return    double - percentage of dark pixels & maskedImg is modified
			// ==============================================================================
            double percentageDark(const cv::Mat&, int ambientThresh, cv::Mat& maskedImg);

			// ==============================================================================
			// \fn:       Boeing::ImageCharacteristics::BrightnessMeasure::pixelsSaturated
			// \brief:    Determine the number of pixels that are saturated and replot the 
			//					original image with the saturated pixels in red for effect
			//
			// @param     const cv::Mat& src  - source image to convert to gray scale
			// @param     cv::Mat& saturated - original image replotted with oversatured->red
			// @return    int - number of pixels saturated & modify saturated
			// ==============================================================================
			int pixelsSaturated(const cv::Mat& src, cv::Mat& saturated);

			// ==============================================================================
			// \fn:       Boeing::ImageCharacteristics::BrightnessMeasure::otsuMask
			// \brief:    Use OpenCV's Otsu's method to determine the optimal B/W threshold
			//					and create a B/W mask for the image accordingly
			//
			// @param     const cv::Mat& src  - source image to convert to gray scale
			// @param     cv::Mat& maskImg - B/W mask based on Otsu threshold
			// @return    double - Otsu threshold & modify maskImg
			// ==============================================================================
			double otsuMask(const cv::Mat& src, cv::Mat& maskImg);

			// ==============================================================================
			// \fn:       Boeing::ImageCharacteristics::BrightnessMeasure::intensityVsAmbient
			// \brief:    Compare lit pixels to their ambient counterparts as a function of 
			//				how bright ambient already is
			//
			// @param     const cv::Mat& ambient  - ambient image to convert to gray scale
			// @param     const cv::Mat& white  - white light image to convert to gray scale
			// @param     cv::Mat& diffScaled - resulting comparison as a matrix (scaled by 100%)
			// @param	  cv::Mat& mask - diffScaled image mask using Otsu threshold
			// @return    double - percent difference between pixels in white image and ambient image
			// ==============================================================================
			double intensityVsAmbient(const cv::Mat& ambient, const cv::Mat& white, cv::Mat& diffScaled, cv::Mat& mask);

			// ==============================================================================
			// \fn:       Boeing::ImageCharacteristics::BrightnessMeasure::otsuMask
			// \brief:    Generate a user-readable summary of image brightness quantification
			//
			// @param     std::string pathName - optional pathname to store data
			// @param     const cv::Mat& src  - source image to convert to gray scale
			// @param     cv::Mat& output - user-readable summary image
			// @param     double& otsuThreshold - calculated Ostu threshold to read as output
			//					(particularly in Workspace)
			// @param     double& percentSaturated - output of pixelsSaturated() to read as 
			//					output (particularly in Workspace)
			// @return    void - but modify output, otsuThreshold, and percentSaturated
			// ==============================================================================
			void brightnessSummary(std::string pathName, const cv::Mat& src, cv::Mat& output, double& otsuThreshold, double& percentSaturated);
		};
	}
}

