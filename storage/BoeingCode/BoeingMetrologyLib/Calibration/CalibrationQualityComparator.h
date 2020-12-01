#ifndef BOEINGMETROLOGYLIB_CALIBRATIONQUALITYCOMPARATOR_H
#define BOEINGMETROLOGYLIB_CALIBRATIONQUALITYCOMPARATOR_H


#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "QualityAssessor.h"
namespace BoeingMetrology
{
	namespace Calibration
	{
		// Container for sensor extrinsics in a reference camera's frame
		class BOEINGMETROLOGYLIB_API CalibrationQualityComparator
		{
		public:
			//set of all assessors that contribute to this comparison (sort by date?)
			std::vector<QualityAssessor> assessors;


			//write out CSV/s containing the data assessments done by the above classes in a week-on-week comparable format
			void writeToCSVs(std::string path);

		};//class CalibrationQualityComparator
	}//namespace Calibration
}//namespace BoeingMetrology






#endif