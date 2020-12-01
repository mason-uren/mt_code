#ifndef BOEINGMETROLOGYLIB_RANGEFINDEROBSERVATION_H
#define BOEINGMETROLOGYLIB_RANGEFINDEROBSERVATION_H

#include <string>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace DeviceIO
    {
        // A single observation for a rangefinder (e.g. acuity)
        class BOEINGMETROLOGYLIB_API RangeFinderObservation : public Boeing::Interface::Serializer
        {
        public:

            // Name of the depth sensor
            std::string rangeFinderName;

            // Depth sensor measurement value (mm)
            double rangeFinderValue;

            // Identifier of this pose
            std::string poseName;

            // Timestamp of data collect
            std::string timestamp;

            // e.g. "mm", "m"
            std::string units;

            RangeFinderObservation() {};

            // Constructor sets timestamp
            RangeFinderObservation(const std::string & name, const double & value, const std::string & poseName, const std::string & units = "m");

            // Get value in meters
            double GetRangeFinderValueInMeters() const;

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

#endif
