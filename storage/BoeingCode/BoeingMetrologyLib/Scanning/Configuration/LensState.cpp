#include "Scanning/Configuration/LensState.h"
#include <fstream>
#include "Utilities/Utilities.h"


/**
 * Construct a default (invalid) LensState
 */
BoeingMetrology::Scanning::Configuration::LensState::LensState()
{
    this->apertureRange = { 0.0, 0.0 };
    this->focusRange = { 0, 0 };
    this->zoomRange = { 0, 0 };
    this->timestamp = BoeingMetrology::Utilities::getCurrentTimeInSeconds();
}

/**
 * Construct a valid LensState if \a deviceName is not empty
 */
BoeingMetrology::Scanning::Configuration::LensState::LensState(const std::string & deviceName, const std::string & ipAddress)
{
    *this = LensState();
    this->deviceName = deviceName;
    this->ipAddress = ipAddress;
}

/**
 * \return true if this object represents a valid LensState for a camera as opposed to a default constructed object.
 */
bool BoeingMetrology::Scanning::Configuration::LensState::IsValid() const
{
    return !this->deviceName.empty();
}

void BoeingMetrology::Scanning::Configuration::LensState::SerializeStream(std::ostream &strm) const
{
    // Camera info
    Json::Value root;
    JsonSerialize(root);
    Json::StyledStreamWriter json_writer;
    json_writer.write(strm, root);
}

void BoeingMetrology::Scanning::Configuration::LensState::DeserializeStream(std::istream &strm)
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
        throw std::runtime_error("LensState::DeserializeStream(): Failed to parse json.");
    }
}

void BoeingMetrology::Scanning::Configuration::LensState::SetZoomRange(const std::pair<int, int> & zoomRangePair)
{
    this->zoomRange = zoomRangePair;
}

void BoeingMetrology::Scanning::Configuration::LensState::SetFocusRange(const std::pair<int, int> & focusRangePair)
{
    this->focusRange = focusRangePair;
}

void BoeingMetrology::Scanning::Configuration::LensState::SetApertureRange(const std::pair<double, double> & apertureRangePair)
{
    this->apertureRange = apertureRangePair;
}

void BoeingMetrology::Scanning::Configuration::LensState::SetSoftwareVersion(const std::string & softwareVersionName)
{
    this->softwareVersion = softwareVersionName;
}

void BoeingMetrology::Scanning::Configuration::LensState::SetApertureValue(const double & apertureVal)
{
    this->aperture = apertureVal;
    this->timestamp = BoeingMetrology::Utilities::getCurrentTimeInSeconds();
}

void BoeingMetrology::Scanning::Configuration::LensState::SetFocusValue(const int & focusVal)
{
    this->focus = focusVal;
    this->timestamp = BoeingMetrology::Utilities::getCurrentTimeInSeconds();
}

void BoeingMetrology::Scanning::Configuration::LensState::SetZoomValue(const int & zoomVal)
{
    this->zoom = zoomVal;
    this->timestamp = BoeingMetrology::Utilities::getCurrentTimeInSeconds();
}

bool BoeingMetrology::Scanning::Configuration::LensState::IsInFocusRange(const int & focusValue) const
{
    if (this->focusRange.first <= focusValue && focusValue <= this->focusRange.second)
        return true;
    return false;
}

bool BoeingMetrology::Scanning::Configuration::LensState::IsInApertureRange(const double & apertureValue) const
{
    if (this->apertureRange.first <= apertureValue && apertureValue <= this->apertureRange.second)
        return true;
    return false;
}

std::pair<double, double> BoeingMetrology::Scanning::Configuration::LensState::GetApertureRange() const
{
    return this->apertureRange;
}

std::pair<int, int> BoeingMetrology::Scanning::Configuration::LensState::GetFocusRange() const
{
    return this->focusRange;
}

std::string BoeingMetrology::Scanning::Configuration::LensState::GetDeviceName() const
{
    return this->deviceName;
}

std::string BoeingMetrology::Scanning::Configuration::LensState::GetIPAddress() const
{
    return this->ipAddress;
}

int BoeingMetrology::Scanning::Configuration::LensState::GetFocusValue() const
{
    return this->focus;
}

double BoeingMetrology::Scanning::Configuration::LensState::GetApertureValue() const
{
    return this->aperture;
}

int BoeingMetrology::Scanning::Configuration::LensState::GetZoomValue() const
{
    return this->zoom;
}

std::string BoeingMetrology::Scanning::Configuration::LensState::GetTimeStamp() const
{
    return this->timestamp;
}

void BoeingMetrology::Scanning::Configuration::LensState::JsonSerialize(Json::Value &jsonNode) const
{
    jsonNode["deviceName"] = this->deviceName;
    jsonNode["ipAddress"] = this->ipAddress;
    jsonNode["softwareVersion"] = this->softwareVersion;
    jsonNode["timestamp"] = this->timestamp;
    jsonNode["focus"] = this->focus;
    jsonNode["aperture"] = this->aperture;
    jsonNode["zoom"] = this->zoom;
    jsonNode["zoomRange"][0U] = this->zoomRange.first;
    jsonNode["zoomRange"][1] = this->zoomRange.second;
    jsonNode["apertureRange"][0U] = this->apertureRange.first;
    jsonNode["apertureRange"][1] = this->apertureRange.second;
    jsonNode["focusRange"][0U] = this->focusRange.first;
    jsonNode["focusRange"][1] = this->focusRange.second;
}

void BoeingMetrology::Scanning::Configuration::LensState::JsonDeserialize(const Json::Value &jsonNode)
{
    this->deviceName = jsonNode["deviceName"].asString();
    this->ipAddress = jsonNode["ipAddress"].asString();
    this->softwareVersion = jsonNode["softwareVersion"].asString();
    this->timestamp = jsonNode["timestamp"].asString();
    this->focus = jsonNode["focus"].asInt();
    this->aperture = jsonNode["aperture"].asDouble();
    this->zoom = jsonNode["zoom"].asInt();
    this->zoomRange = { jsonNode["zoomRange"][0U].asInt(), jsonNode["zoomRange"][1].asInt() };
    this->apertureRange = { jsonNode["apertureRange"][0U].asDouble(), jsonNode["apertureRange"][1].asDouble() };
    this->focusRange = { jsonNode["focusRange"][0U].asInt(), jsonNode["focusRange"][1].asInt() };
}

void BoeingMetrology::Scanning::Configuration::LensState::SerializeFile(const std::string &fileName) const
{
    //Write the serialization to the file
    std::ofstream strm(fileName);
    SerializeStream(strm);
    strm.close();
}

void BoeingMetrology::Scanning::Configuration::LensState::DeserializeFile(const std::string &fileName)
{
    //Read the content from the file and de-serialize into the class
    std::ifstream strm(fileName);
    DeserializeStream(strm);
    strm.close();
}

bool BoeingMetrology::Scanning::Configuration::LensState::operator<(const LensState& rhs) const
{
    return (this->focus < rhs.focus);
}

bool BoeingMetrology::Scanning::Configuration::LensState::operator!=(const LensState& rhs) const
{
    return !operator==(rhs);
}

bool BoeingMetrology::Scanning::Configuration::LensState::operator==(const LensState& rhs) const
{
    if (this->deviceName != rhs.deviceName)
        return false;
    if (this->softwareVersion != rhs.softwareVersion)
        return false;
    if (this->focus != rhs.focus)
        return false;
    if (std::abs(this->aperture - rhs.aperture) > 1e-3)
        return false;
    if (this->zoom != rhs.zoom)
        return false;

    return true;
}
