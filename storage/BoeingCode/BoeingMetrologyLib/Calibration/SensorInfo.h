#ifndef BOEINGMETROLOGYLIB_SENSORINFO_H
#define BOEINGMETROLOGYLIB_SENSORINFO_H

#include <string>

#include <cv.h>

#include "Common/Interface/Serializer.h"
#include "IntrinsicData.h"


namespace BoeingMetrology
{
namespace Calibration
{
    /**
     * \brief Container class for information about a specific sensor type.
     */
    class BOEINGMETROLOGYLIB_API SensorInfo : public Boeing::Interface::Serializer
    {
    public:
        SensorInfo() = default;
        SensorInfo(const SensorInfo& other) = default;
        SensorInfo& operator=(const SensorInfo& other) = default;

        // Name of the sensor
        std::string name;

        // Name of the sensor type
        std::string typeName;

        // Description of the sensor type, if available
        std::string serialNumber;

        // Request address for ZMQ
        std::string zmqRequestAddress;

        // Trigger type
        int triggerType;

        // Populate this object from the input Json.
        // Throws exception on failure.
        virtual void JsonDeserialize(const Json::Value& jsonNode) override;

        // Populate the output Json from this object's data members.
        // Throws exception on failure.
        virtual void JsonSerialize(Json::Value& jsonNode) const override;
    };
}

    // TODO: 1) Create a serialisable array of these
    //       2) Make it an input to the CreateVirtualSensors operation
    //       3) Internally, convert this array into a map indexed by sensor name
    //       4) Internally, have a corresponding map of sensorType to MeshModelSource/Instance
    //          for type-specific models that will be displayed
}

#endif
