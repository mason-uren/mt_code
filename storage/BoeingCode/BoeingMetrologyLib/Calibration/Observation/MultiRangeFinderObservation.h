#ifndef BOEINGMETROLOGYLIB_MULTIRANGEFINDEROBSERVATION_H
#define BOEINGMETROLOGYLIB_MULTIRANGEFINDEROBSERVATION_H

#include <string>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "DeviceIO/RangeFinderObservation.h"
#include "TypeDefs.h"

namespace BoeingMetrology
{
	namespace Calibration
    {
        namespace Observation
        {
            // Multiple observations for a rangefinder (e.g. acuity)
            class BOEINGMETROLOGYLIB_API MultiRangeFinderObservation : public Boeing::Interface::Serializer
            {
            public:
                // Name of the depth sensor
                std::string rangeFinderName;

                // RangeFinderObservations indexed by pose name
                std::map<POSE_NAME, DeviceIO::RangeFinderObservation> observations;

                MultiRangeFinderObservation() {};

                // Constructor that populates this class from a pose directory.  Optionally, only considers poses
                // for which an observing reference camera has observations.
                MultiRangeFinderObservation(const std::string & rangeFinderName, const std::string & poseDirName, const std::string & refCameraName = "");

                // Populate the output Json from this object's data members.
                // Throws exception on failure.
                virtual void JsonSerialize(Json::Value &jsonNode) const override;

                // Populate this object from the input stream.
                // Throws exception on failure.
                virtual void JsonDeserialize(const Json::Value &jsonNode) override;

                // Write this class's contents to a json-formatted text file
                virtual void SerializeFile(const std::string &fileName) const override;

                // Read a json-formatted text file into this class
                virtual void DeserializeFile(const std::string &fileName) override;
            };
        }
	}
}

#endif
