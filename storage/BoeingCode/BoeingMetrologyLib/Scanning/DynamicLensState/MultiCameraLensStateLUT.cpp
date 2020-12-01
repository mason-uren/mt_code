#include "MultiCameraLensStateLUT.h"


size_t BoeingMetrology::Scanning::DynamicLensState::MultiCameraLensStateLUT::GetLensStateCount(const std::string & cameraName) const
{
    return this->lensStates.count(cameraName);
}

void BoeingMetrology::Scanning::DynamicLensState::MultiCameraLensStateLUT::AddLensState(const std::string & cameraName, const double & focusDistance, 
    const Scanning::Configuration::LensState & lensState)
{
    this->lensStates[cameraName][focusDistance] = lensState;
}

void BoeingMetrology::Scanning::DynamicLensState::MultiCameraLensStateLUT::GetLensStateLinearlyInterpolate(const std::string & cameraName, const int & zoomValue, const double & apertureValue, 
    const double & focusDistance, std::pair<double, Scanning::Configuration::LensState> & lensState) const
{
    // Determine the lens states that match this camera name, zoom, and aperture
    std::map<double, Scanning::Configuration::LensState> lensStateMatches;
    if (this->lensStates.count(cameraName) > 0)
    {
        for (const auto & lensStateItem : this->lensStates.at(cameraName))
        {
            if (lensStateItem.second.GetZoomValue() == zoomValue && std::abs(lensStateItem.second.GetApertureValue() - apertureValue) < 0.01)
                lensStateMatches.insert(lensStateItem);
        }
    }
    if (lensStateMatches.size() < 2)
        throw std::runtime_error("MultiCameraLensStateLUT::GetLensStateLinearlyInterpolate: Requires at least two lens states in LUT for " + cameraName);

    // Find the bound entries of the map for this distance.  Careful with extrapolating cases.  
    auto lowerBound = lensStateMatches.begin();
    auto upperBound = lensStateMatches.lower_bound(focusDistance);
    if (upperBound == lensStateMatches.begin())
        upperBound++;
    else if (upperBound == lensStateMatches.end())
    {
        upperBound--;
        lowerBound = --(--lensStateMatches.lower_bound(focusDistance));
    }
    else
        lowerBound = (--lensStateMatches.lower_bound(focusDistance));

    // Initialize output
    lensState = { focusDistance, lowerBound->second };

    // Linearly interpolate (or extrapolate) the focus value and round to int
    double proportion = (focusDistance - lowerBound->first) / (upperBound->first - lowerBound->first);
    double focusValueD = lowerBound->second.GetFocusValue() + proportion * (double)(upperBound->second.GetFocusValue() - lowerBound->second.GetFocusValue());
    int focusValue = (int)(focusValueD + 0.5);

    // Update the output
    lensState.second.SetFocusValue(focusValue);
}

void BoeingMetrology::Scanning::DynamicLensState::MultiCameraLensStateLUT::JsonDeserialize(const Json::Value &jsonNode)
{
    try
    {
        // Loop through lens states
        for (const auto & lensStateListJson : jsonNode)
        {
            for (const auto & lensStateJson : lensStateListJson["lensStates"])
            {
                Scanning::Configuration::LensState lensState;
                lensState.JsonDeserialize(lensStateJson["lensState"]);
                this->lensStates[lensStateListJson["cameraName"].asString()][lensStateJson["focusDistance"].asDouble()] = lensState;
            }
        }
    }
    catch (...)
    {
        throw std::runtime_error("MultiCameraLensStateLUT::JsonDeserialize failed");
    }
}

void BoeingMetrology::Scanning::DynamicLensState::MultiCameraLensStateLUT::JsonSerialize(Json::Value &jsonNode) const
{
    try
    {
        // Loop through lens states
        for (const auto & lensStateList : this->lensStates)
        {
            Json::Value lensStateListJson;
            lensStateListJson["cameraName"] = lensStateList.first;
            for (const auto & lensState : lensStateList.second)
            {
                Json::Value lensStateJson;
                lensStateJson["focusDistance"] = lensState.first;
                lensState.second.JsonSerialize(lensStateJson["lensState"]);
                lensStateListJson["lensStates"].append(lensStateJson);
            }
            jsonNode.append(lensStateListJson);
        }
    }
    catch (...)
    {
        throw std::runtime_error("MultiCameraLensStateLUT::JsonSerialize failed");
    }
}
