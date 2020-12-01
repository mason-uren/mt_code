#include "Configuration3DUReconstruction.h"
#include <fstream>
#include <json/reader.h>
#include <json/writer.h>

BoeingMetrology::Scanning::Configuration::Configuration3DUReconstruction::Configuration3DUReconstruction(const std::vector<Configuration3DUCollection> & configuration3DUCollections, 
    const Calibration::MultiCameraIntrinsicData & multiCameraIntrinsicData, const Calibration::MultiCameraExtrinsicData & multiCameraExtrinsicData, const std::string & reconstructionPath, 
    const double & reconstructionBlackThreshold, const double & reconstructionWhiteThreshold)
{
    // Validate inputs
    for (const auto& config : configuration3DUCollections)
    {
        for (const auto& camera : config.GetCameraSettings())
        {
            auto name = camera.first;
            if (multiCameraIntrinsicData.cameraData.count(name) == 0)
                throw std::runtime_error("Configuration3DUReconstruction: " + name + " camera is missing from MultiCameraIntrinsicData");
            if (multiCameraExtrinsicData.cameraData.count(name) == 0)
                throw std::runtime_error("Configuration3DUReconstruction: " + name + " camera is missing from MultiCameraExtrinsicData");
        }
    }

    // Generate a timestamp
    this->timestamp = Utilities::getCurrentTimeInSeconds();

    // Copy remaining inputs
    this->configuration3DUCollections = configuration3DUCollections;
    this->multiCameraIntrinsicData = multiCameraIntrinsicData;
    this->multiCameraExtrinsicData = multiCameraExtrinsicData;
    this->reconstructionPath = reconstructionPath;
    this->reconstructionBlackThreshold = reconstructionBlackThreshold;
    this->reconstructionWhiteThreshold = reconstructionWhiteThreshold;
}

void BoeingMetrology::Scanning::Configuration::Configuration3DUReconstruction::JsonSerialize(Json::Value &jsonNode) const
{
    jsonNode["timestamp"] = this->timestamp;
    for (const auto & coll : this->configuration3DUCollections)
    {
        Json::Value collJson;
        coll.JsonSerialize(collJson);
        jsonNode["configuration3DUCollections"].append(collJson);
    }
    this->multiCameraIntrinsicData.JsonSerialize(jsonNode["multiCameraIntrinsicData"]);
    this->multiCameraExtrinsicData.JsonSerialize(jsonNode["multiCameraExtrinsicData"]);
    jsonNode["reconstructionPath"] = this->reconstructionPath;
    jsonNode["reconstructionBlackThreshold"] = this->reconstructionBlackThreshold;
    jsonNode["reconstructionWhiteThreshold"] = this->reconstructionWhiteThreshold;
}

void BoeingMetrology::Scanning::Configuration::Configuration3DUReconstruction::JsonDeserialize(const Json::Value &jsonNode)
{
    this->timestamp = jsonNode["timestamp"].asString();
    for (const auto & collJson : jsonNode["configuration3DUCollections"])
    {
        Configuration3DUCollection coll;
        coll.JsonDeserialize(collJson);
        this->configuration3DUCollections.push_back(coll);
    }
    Calibration::MultiCameraIntrinsicData multiCameraIntrinsicDataInit;
    multiCameraIntrinsicDataInit.JsonDeserialize(jsonNode["multiCameraIntrinsicData"]);
    this->multiCameraIntrinsicData = multiCameraIntrinsicDataInit;
    Calibration::MultiCameraExtrinsicData multiCameraExtrinsicDataInit;
    multiCameraExtrinsicDataInit.JsonDeserialize(jsonNode["multiCameraExtrinsicData"]);
    this->multiCameraExtrinsicData = multiCameraExtrinsicDataInit;
    this->reconstructionPath = jsonNode["reconstructionPath"].asString();
    this->reconstructionBlackThreshold = jsonNode["reconstructionBlackThreshold"].asDouble();
    this->reconstructionWhiteThreshold = jsonNode["reconstructionWhiteThreshold"].asDouble();
}
