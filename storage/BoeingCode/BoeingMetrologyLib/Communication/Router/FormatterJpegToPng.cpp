/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    FormatterJpegToPng.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for FormatterJpegToPng class
*
*****************************************************************************/
#include <opencv2/opencv.hpp>

#include "FormatterJpegToPng.h"
#include "FormatterJpegToMat.h"

namespace BoeingMetrology
{
	namespace Communication
	{
		void FormatterJpegToPng::format(ImageType &inputData, ImageType &outputData)
		{
			outputData.info = inputData.info;
			cv::Mat imageMat;
			//first convert Jpeg to Open CV Mat type
			FormatterJpegToMat::format(inputData, imageMat);
			//now convert from OpenCV Mat to PNG
			if (!cv::imencode(".png", imageMat, outputData.buffer))
			{
				throw runtime_error("Failed to format file (JPEG to PNG)");
			}
		}
	}
}
