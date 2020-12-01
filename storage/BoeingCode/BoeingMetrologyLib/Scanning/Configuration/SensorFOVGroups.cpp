#include "SensorFOVGroups.h"
#include <fstream>
#include <json/reader.h>
#include <json/writer.h>

void BoeingMetrology::Scanning::Configuration::SensorFOVGroups::SerializeStream(std::ostream &strm) const
{
    // Camera info
    Json::Value root;
    JsonSerialize(root);
    Json::StyledStreamWriter json_writer;
    json_writer.write(strm, root);
}

void BoeingMetrology::Scanning::Configuration::SensorFOVGroups::DeserializeStream(std::istream &strm)
{
    Json::Reader jsonReader;
    Json::Value root;
    //Read through the JSON file and collect the points
    if (jsonReader.parse(strm, root))
    {
        JsonDeserialize(root);
    }
    else
    {
        throw std::runtime_error("SensorFOVGroups::DeserializeStream(): Failed to parse json.");
    }
}

std::map<std::string, std::string> BoeingMetrology::Scanning::Configuration::SensorFOVGroups::GetRangeFindersPerGroup() const
{
    std::map<std::string, std::string> rangeFindersPerGroup;
    for (const auto & group : this->sensorFovGroups)
    {
        std::vector<std::string> rangeFindersThisGroup;
        for (const auto & sensorName : group.second)
        {
            if (sensorName.find("Acuity") != std::string::npos)
                rangeFindersThisGroup.push_back(sensorName);
        }
        if (rangeFindersThisGroup.size() == 1)
            rangeFindersPerGroup[group.first] = rangeFindersThisGroup.front();
        else if (rangeFindersThisGroup.size() > 1)
            throw std::runtime_error("SensorFOVGroups::GetRangeFindersPerGroup(): Multiple rangefinders found in a single group!");
    }
    return rangeFindersPerGroup;
}

void BoeingMetrology::Scanning::Configuration::SensorFOVGroups::JsonSerialize(Json::Value &jsonNode) const
{
    for (const auto & group : this->sensorFovGroups)
    {
        Json::Value groupJson;
        groupJson["groupName"] = group.first;
        for (const auto & camera : group.second)
            groupJson["sensors"].append(camera);
        jsonNode.append(groupJson);
    }
}

void BoeingMetrology::Scanning::Configuration::SensorFOVGroups::JsonDeserialize(const Json::Value &jsonNode)
{
    for (const auto & groupJson : jsonNode)
    {
        const std::string groupName = groupJson["groupName"].asString();
        std::vector<CAMERA_NAME> cameras;
        for (const auto & cameraJson : groupJson["sensors"])
            cameras.push_back(cameraJson.asString());
        this->sensorFovGroups[groupName] = cameras;
    }
}

void BoeingMetrology::Scanning::Configuration::SensorFOVGroups::SerializeFile(const std::string &fileName) const
{
    //Write the serialization to the file
    std::ofstream strm(fileName);
    SerializeStream(strm);
    strm.close();
}

void BoeingMetrology::Scanning::Configuration::SensorFOVGroups::DeserializeFile(const std::string &fileName)
{
    //Read the content from the file and de-serialize into the class
    std::ifstream strm(fileName);
    DeserializeStream(strm);
    strm.close();
}
