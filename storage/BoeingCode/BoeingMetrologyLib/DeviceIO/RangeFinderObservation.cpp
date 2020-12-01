#include "RangeFinderObservation.h"
#include "json/reader.h"
#include "json/writer.h"
#include <fstream>
#include "Utilities/Utilities.h"

BoeingMetrology::DeviceIO::RangeFinderObservation::RangeFinderObservation(const std::string & name, const double & value, 
    const std::string & poseName, const std::string & units)
{
    this->rangeFinderName = name;
    this->rangeFinderValue = value;
    this->poseName = poseName;
    this->units = units;
    this->timestamp = Utilities::getCurrentTimeInSeconds();
}

double BoeingMetrology::DeviceIO::RangeFinderObservation::GetRangeFinderValueInMeters() const
{
    double result = this->rangeFinderValue;
    if (this->units == "mm")
        result /= 1000.0;
    else if (this->units != "m")
        throw std::runtime_error("RangeFinderObservation::GetRangeFinderValueInMeters(): ERROR Unsupported units " + this->units);

    return result;
}

void BoeingMetrology::DeviceIO::RangeFinderObservation::JsonSerialize(Json::Value &jsonNode) const
{
    try
    {
        jsonNode["rangeFinderName"]  = this->rangeFinderName;
        jsonNode["rangeFinderValue"] = this->rangeFinderValue;
        jsonNode["poseName"]         = this->poseName;
        jsonNode["timestamp"]        = this->timestamp;
        jsonNode["units"]            = this->units;
    }
    catch (...)
    {
        throw;
    }
}

void BoeingMetrology::DeviceIO::RangeFinderObservation::JsonDeserialize(const Json::Value &jsonNode)
{
    try
    {
        this->rangeFinderName  = jsonNode["rangeFinderName"].asString();
        this->rangeFinderValue = jsonNode["rangeFinderValue"].asDouble();
        this->poseName         = jsonNode["poseName"].asString();
        this->timestamp        = jsonNode["timestamp"].asString();
        this->units            = jsonNode["units"].asString();
    }
    catch (...)
    {
        throw;
    }
}

void BoeingMetrology::DeviceIO::RangeFinderObservation::SerializeFile(const std::string &fileName) const
{
    try
    {
        Json::Value jsonNode;
        this->JsonSerialize(jsonNode);
        Json::StyledWriter styledWriter;
        std::ofstream writer(fileName, std::ifstream::binary);
        writer << styledWriter.write(jsonNode);
        writer.close();
    }
    catch (...)
    {
        throw std::runtime_error("RangeFinderObservation::SerializeFile failed");
    }
}

void BoeingMetrology::DeviceIO::RangeFinderObservation::DeserializeFile(const std::string &fileName)
{
    try
    {
        Json::Reader jsonReader;
        std::ifstream fReader(fileName, std::ifstream::binary);
        Json::Value root;
        if (jsonReader.parse(fReader, root))
        {
            this->JsonDeserialize(root);
        }
        else
            throw std::runtime_error("RangeFinderObservation::DeserializeFile failed");
    }
    catch (...)
    {
        throw std::runtime_error("RangeFinderObservation::DeserializeFile failed");
    }
}