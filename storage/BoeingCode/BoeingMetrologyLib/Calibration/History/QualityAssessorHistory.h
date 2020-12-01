#ifndef BOEINGMETROLOGYLIB_QUALITYASSESSORHISTORY_H
#define BOEINGMETROLOGYLIB_QUALITYASSESSORHISTORY_H

#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"
#include "DataHistory.h"
#include "Calibration/QualityAssessor.h"
#include <map>
namespace BoeingMetrology
{
    namespace Calibration
    {
        class BOEINGMETROLOGYLIB_API QualityAssessorHistory : public DataHistory
        {
        public:
            std::map<TIMESTAMP, QualityAssessor> sortedAssessments;
            CAMERA_NAME name;
            QualityAssessorHistory();
            QualityAssessorHistory(std::map<TIMESTAMP, QualityAssessor> sourceData);
            void addData(QualityAssessor data);
            void WriteToCSV(std::ostream& csv) override;

            // Populate this object from the input stream.
            // Throws exception on failure.
            void JsonDeserialize(const Json::Value &jsonNode) override;

            // Populate the output Json from this object's data members.
            // Throws exception on failure.
            void JsonSerialize(Json::Value &jsonNode) const override;

        };//class QualityAssessorHistory
    }//namespace Calibration
}//namespace BoeingMetrology

#endif
