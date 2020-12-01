/******************************************************************************
*  BOEING PROPRIETARY
*
*  @copyright
*     Copyright 2018 The Boeing Company
*     Unpublished Work-All Rights Reserved
*     Third Party Disclosure Requires Written Approval
*
*  @file    FormatterInterface.cpp
*  @author  Tom Childress
*
*  @brief Implementation file for FormatterInterface class
*
*****************************************************************************/
#include <map>
#include <string>
#include "FormatterInterface.h"
#include "FormatterJpegToMat.h"
#include "FormatterJpegToPng.h"
#include "FormatterRawToMat.h"
#include "FormatterRawToPng.h"
using namespace std;

namespace BoeingMetrology
{
	namespace Communication
	{
		const string FormatterInterface::JPEG_TO_MAT_FORMATTER_LABEL = "JpegToMat";
		const string FormatterInterface::JPEG_TO_PNG_FORMATTER_LABEL = "JpegToPng";
		const string FormatterInterface::RAW_TO_MAT_FORMATTER_LABEL = "RawToMat";
		const string FormatterInterface::RAW_TO_PNG_FORMATTER_LABEL = "RawToPng";

		map<string, uint32_t> FormatterInterface::formatterLabelMap =
		{
			{ JPEG_TO_MAT_FORMATTER_LABEL, JPEG_TO_MAT_FORMATTER },
			{ JPEG_TO_PNG_FORMATTER_LABEL, JPEG_TO_PNG_FORMATTER },
			{ RAW_TO_MAT_FORMATTER_LABEL, RAW_TO_MAT_FORMATTER },
			{ RAW_TO_PNG_FORMATTER_LABEL, RAW_TO_PNG_FORMATTER }
		};

		FormatterInterface* FormatterInterface::createFormatter(string formatterID_Label)
		{
			FormatterInterface *retVal = nullptr;
			//make sure this is a valid formatter ID label
			if (formatterLabelMap.count(formatterID_Label) > 0)
			{
				const uint32_t routeID = formatterLabelMap[formatterID_Label];
				switch (routeID)
				{
				case JPEG_TO_MAT_FORMATTER:
					retVal = new FormatterJpegToMat();
					break;
				case JPEG_TO_PNG_FORMATTER:
					retVal = new FormatterJpegToPng();
					break;
				case RAW_TO_MAT_FORMATTER:
					retVal = new FormatterRawToMat();
					break;
				case RAW_TO_PNG_FORMATTER:
					retVal = new FormatterRawToPng();
					break;
				default:
					break;
				}
			}
			return retVal;
		}
	}
}