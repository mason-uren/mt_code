#include <fstream>

#include <opencv2/opencv.hpp>

#include "Utilities/Utilities.h"
#include "SensorInfo.h"
#include "json/reader.h"

/**
 *
 */
void BoeingMetrology::Calibration::SensorInfo::JsonDeserialize(const Json::Value& jsonNode)
{
	try
	{
        std::string jsonNodeName = jsonNode["Name"].asString();
        if (!jsonNodeName.empty())
		{
			// Standard format
            this->name = jsonNodeName;
            this->typeName = jsonNode["_sensortype_name"].asString();
            this->serialNumber = jsonNode["SerialNumber"].asString();
            this->triggerType = jsonNode["TriggerType"].asInt();
            this->zmqRequestAddress = jsonNode["ZMQRequestAddress"].asString();
		}
		else
        {
            throw std::runtime_error("SensorInfo::JsonDeserialize() : Unrecognized json format");
        }
	}
	catch (...)
	{
		throw;
	}
}

/**
 *
 */
void BoeingMetrology::Calibration::SensorInfo::JsonSerialize(Json::Value& jsonNode) const
{
    jsonNode["Name"] = this->name;
    jsonNode["_sensortype_name"] = this->typeName;
    jsonNode["SerialNumber"] = this->serialNumber;
    jsonNode["TriggerType"] = this->triggerType;
    jsonNode["ZMQRequestAddress"] = this->zmqRequestAddress;
}
