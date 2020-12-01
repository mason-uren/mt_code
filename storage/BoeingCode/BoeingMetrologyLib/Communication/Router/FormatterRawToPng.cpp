/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    FormatterRawToPng.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for FormatterRawToPng class
*
*****************************************************************************/
#include <opencv2/opencv.hpp>
#include <sstream>

#include "FormatterInterface.h"
#include "FormatterRawToMat.h"
#include "FormatterRawToPng.h"

namespace BoeingMetrology
{
	namespace Communication
	{
		void FormatterRawToPng::format(ImageType &inputData, ImageType &outputData)
		{
			outputData.info = inputData.info;
			cv::Mat imageMat;
			//first convert Raw to OpenCV Mat type 
			FormatterRawToMat::format(inputData, imageMat);			
			//now convert from OpenCV Mat to PNG
			if (!cv::imencode(".png", imageMat, outputData.buffer))
			{
				throw runtime_error("Failed to format file (Raw to PNG)");
			}
		}
	}
}
