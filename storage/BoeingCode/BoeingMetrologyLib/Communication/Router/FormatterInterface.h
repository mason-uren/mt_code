/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    FormatterInterface.h
*  @author  Tom Childress
*
*  @brief Abstract class that exposes the virtual format() method for formatting
*         images
*
*****************************************************************************/
#pragma once
#include <cstdint>
#include <map>
#include <vector>
#include <string>
#include "BoeingMetrologyLib_API.h"
using namespace std;

namespace BoeingMetrology
{
	namespace Communication
	{
		class BOEINGMETROLOGYLIB_API FormatterInterface
		{
		public:
			/************************************************************************
			@brief Image types needed for formatting
			************************************************************************/
			typedef struct
			{
				uint32_t height;
				uint32_t width;
				uint32_t rowSize;
				uint32_t pixelSize;//in bits
				bool isPixelSigned;
				uint32_t numChannels;
				bool flipVertical;
				bool flipHorizontal;
			} ImageInfoType;

			typedef vector<uint8_t> ImageBufferType;

			typedef struct
			{
				ImageInfoType info;
				ImageBufferType buffer;
			} ImageType;
			/************************************************************************
			@brief Constructor

			This is the default constructor
			************************************************************************/
			FormatterInterface() = default;
			/************************************************************************
			@brief Destructor

			This is the default destructor
			************************************************************************/
			virtual ~FormatterInterface() = default;
			/************************************************************************
			@brief Pure virtual method that exposes the formatter interface
			@param[in] inputData Data to be formatted
			@param[out] outputData Data that has been formatted

			This method exposes an abstract interface for formatting data
			************************************************************************/
			virtual void format(ImageType &inputData, ImageType &outputData) = 0;
			/************************************************************************
			@brief Constructs unique derived instances of FormatterInterface
			@param[in] formatterID_Label Label used to determine which instance to return

			This static method serves as the factory interface for objects that implement the FormatterInterface
			************************************************************************/
			static FormatterInterface* createFormatter(string formatterID_Label);
			//formatter labels
			static const string JPEG_TO_MAT_FORMATTER_LABEL;
			static const string JPEG_TO_PNG_FORMATTER_LABEL;
			static const string RAW_TO_MAT_FORMATTER_LABEL;
			static const string RAW_TO_PNG_FORMATTER_LABEL;

		protected:
			//formatter IDs
			enum
			{
				JPEG_TO_MAT_FORMATTER,
				JPEG_TO_PNG_FORMATTER,
				RAW_TO_MAT_FORMATTER,
				RAW_TO_PNG_FORMATTER,
				NUM_FORMATTERS
			};
			//map of labels to ids
			static map<string, uint32_t> formatterLabelMap;
		};
	}
}

