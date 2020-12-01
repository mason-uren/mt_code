#ifndef BOEINGMETROLOGYLIB_SensorFOVGroups_H
#define BOEINGMETROLOGYLIB_SensorFOVGroups_H

#include <string>
#include <map>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"

namespace BoeingMetrology
{
	namespace Scanning
	{
		namespace Configuration
		{
			// Organize groups of cameras that share overlapping fields of view.  Groups are not disjoint.
            // For example, Nikon_B.1 can exist in more than one group.
            class BOEINGMETROLOGYLIB_API SensorFOVGroups : public Boeing::Interface::Serializer
			{
            private:

                void SerializeStream(std::ostream &strm) const;

                void DeserializeStream(std::istream &strm);

            public:

                // A map of lists of cameras that share overlapping fields of view
                std::map<std::string, std::vector<CAMERA_NAME>> sensorFovGroups;

                // Get groups with a rangefinder.  Each group should have either 0 or 1 rangefinders.
                std::map<std::string, std::string> GetRangeFindersPerGroup() const;

				// Populate the output Json from this object's data.
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
