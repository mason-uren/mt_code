#include <fstream>

#include <opencv2/opencv.hpp>

#include "Utilities/Utilities.h"
#include "SensorInfoArray.h"
#include "json/reader.h"

/**
 *
 */
void BoeingMetrology::Calibration::SensorInfoArray::JsonDeserialize(const Json::Value& jsonNode)
{
    sensors.clear();
    for (int i = 0; i < (int)jsonNode.size(); ++i)
    {
        sensors.push_back(BoeingMetrology::Calibration::SensorInfo());
        sensors.back().JsonDeserialize(jsonNode[i]);
    }
}

/**
 *
 */
void BoeingMetrology::Calibration::SensorInfoArray::JsonSerialize(Json::Value& jsonNode) const
{
    for (auto& sensor : sensors)
    {
        jsonNode = Json::Value(Json::arrayValue);
        auto& value = jsonNode.append(Json::Value(Json::objectValue));
        sensor.JsonSerialize(value);
    }
}
