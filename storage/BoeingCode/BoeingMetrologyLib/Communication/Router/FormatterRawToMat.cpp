/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    FormatterRawToMat.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for FormatterRawToMat class
*
*****************************************************************************/
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>

#include "FormatterRawToMat.h"

namespace BoeingMetrology
{
	namespace Communication
	{
		void FormatterRawToMat::format(ImageType &inputData, cv::Mat &outputData)
		{
			//create Open CV Mat image
			int cvDepth = 0;
			switch (inputData.info.pixelSize)
			{
			case 8:
				cvDepth = inputData.info.isPixelSigned ? CV_8S : CV_8U;
				break;
			case 16:
				cvDepth = inputData.info.isPixelSigned ? CV_16S : CV_16U;
				break;
			default:
				stringstream ss;
				ss << "Unsupported pixel size: " << inputData.info.pixelSize;
				//report error
				throw runtime_error(ss.str());
			}
			cv::Mat tempInputImage = cv::Mat(inputData.info.height,
							 	             inputData.info.width,
								             CV_MAKETYPE(cvDepth, inputData.info.numChannels),
								             inputData.buffer.data(),
								             inputData.info.rowSize);
			//see if image is flipped on any axis
			if ((inputData.info.flipVertical) && (inputData.info.flipHorizontal))
			{
				cv::Mat tempOutputImage;
				cv::flip(tempInputImage.clone(), tempOutputImage, 0);
				cv::flip(tempOutputImage.clone(), outputData, 1);
			}
			else if (inputData.info.flipVertical)
			{
				cv::flip(tempInputImage.clone(), outputData, 0);
			}
			else if (inputData.info.flipHorizontal)
			{
				cv::flip(tempInputImage.clone(), outputData, 1);
			}
			else
			{
				outputData = tempInputImage.clone();
			}
		}

		void FormatterRawToMat::format(ImageType &inputData, ImageType &outputData)
		{
			outputData.info = inputData.info;
			cv::Mat imageMat;
			//format to OpenCV Mat type
			format(inputData, imageMat);
			//now populate output buffer with image data
			if (imageMat.isContinuous())
			{
				outputData.buffer.assign(imageMat.datastart, imageMat.dataend);
			}
			else
			{
				for (int rowIndex = 0; rowIndex < imageMat.rows; ++rowIndex)
				{
					outputData.buffer.insert(outputData.buffer.end(), imageMat.ptr<uint8_t>(rowIndex), imageMat.ptr<uint8_t>(rowIndex) + imageMat.cols);
				}
			}
		}
	}
}
