/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    FormatterRawToPng.h
*  @author  Tom Childress
*
*  @brief Class derived from FormatterInterface and FormatterRawToMat that 
*         provides unique implementation of format() method.
*
*****************************************************************************/
#pragma once
#include "FormatterInterface.h"
#include "FormatterRawToMat.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Communication
	{
		class BOEINGMETROLOGYLIB_API FormatterRawToPng : public FormatterRawToMat
		{
		public:
			/************************************************************************
			@brief Constructor

			This is the default constuctor
			************************************************************************/
			FormatterRawToPng() = default;
			/************************************************************************
			@brief Destructor

			This is the default destructor
			************************************************************************/
			~FormatterRawToPng() = default;
			/************************************************************************
			@brief Implements unique formatting behavior
			@param[in] inputData Data to be formatted
			@param[out] outputData Data that is formatted

			This method implements the abstract interface of FormatterInterface.
			Specifically, this formats a Raw image to PNG
			************************************************************************/
			virtual void format(ImageType &inputData, ImageType &outputData);
		};
	}
}

