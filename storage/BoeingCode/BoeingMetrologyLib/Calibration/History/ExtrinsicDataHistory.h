#ifndef BOEINGMETROLOGYLIB_EXTRINSICDATAHISTORY_H
#define BOEINGMETROLOGYLIB_EXTRINSICDATAHISTORY_H

#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"
#include "Calibration/ExtrinsicData.h"
#include "DataHistory.h"
#include <map>
namespace BoeingMetrology
{
	namespace Calibration
	{
		class BOEINGMETROLOGYLIB_API ExtrinsicDataHistory : public DataHistory
		{
		public:
			std::map<TIMESTAMP, ExtrinsicData> sortedExtrinsics;
			CAMERA_NAME name;
			ExtrinsicDataHistory();
			ExtrinsicDataHistory(std::map<TIMESTAMP, ExtrinsicData> sourceData);
			void addData(ExtrinsicData data);
			void WriteToCSV(std::ostream& csv) override;

			// Populate this object from the input stream.
			// Throws exception on failure.
			void JsonDeserialize(const Json::Value &jsonNode) override;

			// Populate the output Json from this object's data members.
			// Throws exception on failure.
			void JsonSerialize(Json::Value &jsonNode) const override;
		};//class ExtrinsicDataHistory
	}//namespace Calibration
}//namespace BoeingMetrology

#endif