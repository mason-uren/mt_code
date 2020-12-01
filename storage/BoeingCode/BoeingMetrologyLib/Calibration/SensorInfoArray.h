#ifndef BOEINGMETROLOGYLIB_SENSORINFOARRAY_H
#define BOEINGMETROLOGYLIB_SENSORINFOARRAY_H

#include <string>

#include <cv.h>

#include "Common/Interface/Serializer.h"
#include "IntrinsicData.h"
#include "SensorInfo.h"


namespace BoeingMetrology
{
namespace Calibration
{

    /**
     *
     */
    class BOEINGMETROLOGYLIB_API SensorInfoArray : public Boeing::Interface::Serializer
    {
    public:
        SensorInfoArray() = default;
        SensorInfoArray(const SensorInfoArray& other) = default;
        SensorInfoArray& operator=(const SensorInfoArray& other) = default;

        std::vector<SensorInfo> sensors;

        // Populate this object from the input Json.
        // Throws exception on failure.
        virtual void JsonDeserialize(const Json::Value& jsonNode) override;

        // Populate the output Json from this object's data members.
        // Throws exception on failure.
        virtual void JsonSerialize(Json::Value& jsonNode) const override;
    };
}
}

#endif
