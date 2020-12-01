/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    FormatterRawToMat.h
*  @author  Tom Childress
*
*  @brief Class derived from FormatterInterface that provides unique implementation
*         of format() method.
*
*****************************************************************************/
#pragma once
#include <opencv2/opencv.hpp>
#include "FormatterInterface.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Communication
	{
		class BOEINGMETROLOGYLIB_API FormatterRawToMat : public FormatterInterface
		{
		public:
			/************************************************************************
			@brief Constructor

			This is the default constuctor
			************************************************************************/
			FormatterRawToMat() = default;
			/************************************************************************
			@brief Destructor

			This is the default destructor
			************************************************************************/
			~FormatterRawToMat() = default;
			/************************************************************************
			@brief Helper method for converting Raw to Mat
			@param[in] inputData Data to be formatted
			@param[out] outputData Data that is formatted

			This method provides the ability to convert from a raw image type to OpenCV
			Mat type
			************************************************************************/
			virtual void format(ImageType &inputData, cv::Mat &outputData);
			/************************************************************************
			@brief Implements unique formatting behavior
			@param[in] inputData Data to be formatted
			@param[out] outputData Data that is formatted

			This method implements the abstract interface of FormatterInterface.
			Specifically, this formats a Raw image to PNG
			************************************************************************/
			virtual void format(ImageType &inputData, ImageType &outputData) override;
		};
	}
}

