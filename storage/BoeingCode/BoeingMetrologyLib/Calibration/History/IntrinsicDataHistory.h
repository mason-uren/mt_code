#ifndef BOEINGMETROLOGYLIB_INTRINSICDATAHISTORY_H
#define BOEINGMETROLOGYLIB_INTRINSICDATAHISTORY_H

#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"
#include "Calibration/IntrinsicData.h"
#include "DataHistory.h"
#include <map>
namespace BoeingMetrology
{
    namespace Calibration
    {
        class BOEINGMETROLOGYLIB_API IntrinsicDataHistory : public DataHistory
        {
        public:
            std::map<TIMESTAMP, IntrinsicData> sortedIntrinsics;
            CAMERA_NAME name;
            IntrinsicDataHistory();
            IntrinsicDataHistory(std::map<TIMESTAMP, IntrinsicData> sourceData);
            void addData(IntrinsicData data);
            void WriteToCSV(std::ostream& csv) override;

            // Populate this object from the input stream.
            // Throws exception on failure.
            void JsonDeserialize(const Json::Value &jsonNode) override;

            // Populate the output Json from this object's data members.
            // Throws exception on failure.
            void JsonSerialize(Json::Value &jsonNode) const override;
        };//class IntrinsicDataHistory
    }//namespace Calibration
}//namespace BoeingMetrology

#endif
