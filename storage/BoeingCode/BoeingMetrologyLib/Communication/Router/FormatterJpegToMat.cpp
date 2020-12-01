/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    FormatterJpegToMat.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for FormatterJpegToMat class
*
*****************************************************************************/
#include <opencv2/opencv.hpp>

#include "FormatterJpegToMat.h"

namespace BoeingMetrology
{
	namespace Communication
	{
		void FormatterJpegToMat::format(ImageType &inputData, cv::Mat &outputData)
		{
			outputData = cv::imdecode(inputData.buffer, CV_LOAD_IMAGE_UNCHANGED);
		}

		void FormatterJpegToMat::format(ImageType &inputData, ImageType &outputData)
		{
			outputData.info = inputData.info;
			cv::Mat outImage;
			//first convert to OpenCV Mat type
			format(inputData, outImage);
			//populate output buffer
			if (outImage.isContinuous()) 
			{
				outputData.buffer.assign(outImage.datastart, outImage.dataend);
			}
			else
			{
				for (int rowIndex = 0; rowIndex < outImage.rows; ++rowIndex)
				{
					outputData.buffer.insert(outputData.buffer.end(), outImage.ptr<uint8_t>(rowIndex), outImage.ptr<uint8_t>(rowIndex) + outImage.cols);
				}
			}
		}
	}
}
