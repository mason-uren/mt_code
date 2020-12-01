#include "Configuration3DUCollection.h"
#include <fstream>
#include <json/reader.h>
#include <json/writer.h>
#include "Utilities/Utilities.h"

namespace BoeingMetrology
{
    namespace Scanning
    {
        namespace Configuration
        {
            Configuration3DUCollection::Configuration3DUCollection(
                const std::string & cameraImagePath,
                const CameraSettingsMap& cameraSettings,
                const std::vector<PROJECTOR_NAME>& projectorNames,
                const cv::Size& projectedImageSize,
                const double projectedImageIntensity,
                const int numberOfPatterns)
            {
                // Generate a timestamp
                this->timestamp = Utilities::getCurrentTimeInSeconds();

                // Copy remaining inputs
                this->cameraImagePath = cameraImagePath;
                this->cameras = cameraSettings;
                this->projectorNames = projectorNames;
                this->projectedImageSize = projectedImageSize;
                this->projectedImageIntensity = projectedImageIntensity;
                this->numberOfPatterns = numberOfPatterns;
            }

            const Configuration3DUCollection::CameraSettingsMap& Configuration3DUCollection::GetCameraSettings() const
            {
                return cameras;
            }

            void Configuration3DUCollection::JsonSerialize(Json::Value &jsonNode) const
            {
                jsonNode["timestamp"] = this->timestamp;
                jsonNode["cameraImagePath"] = this->cameraImagePath;
                for (const auto & camera : this->cameras)
                {
                    Json::Value cameraJson;
                    cameraJson["cameraName"] = camera.first;
                    cameraJson["exposureTimeUs"] = camera.second.exposureTime;
                    cameraJson["gain"] = camera.second.gain;
                    if (camera.second.lenState.IsValid())
                    {
                        camera.second.lenState.JsonSerialize(cameraJson["lensState"]);
                    }
                    jsonNode["cameras"].append(cameraJson);
                }

                for (const auto& name : this->projectorNames)
                {
                    jsonNode["projectorNames"].append(name);
                }

                jsonNode["projectedImageSize"][0U] = this->projectedImageSize.width;
                jsonNode["projectedImageSize"][1] = this->projectedImageSize.height;
                jsonNode["projectedImageIntensity"] = this->projectedImageIntensity;
                jsonNode["numberOfPatterns"] = this->numberOfPatterns;
            }

            void Configuration3DUCollection::JsonDeserialize(const Json::Value &jsonNode)
            {
                this->timestamp = jsonNode["timestamp"].asString();
                this->cameraImagePath = jsonNode["cameraImagePath"].asString();
                this->cameras.clear();
                this->projectorNames.clear();

                if (jsonNode.isMember("lensStates"))
                {
                    // Old format
                    for (const auto & cameraJson : jsonNode["lensStates"])
                    {
                        cameras[cameraJson["cameraName"].asString()].lenState.JsonDeserialize(cameraJson["lensState"]);
                    }
                    for (const auto & cameraJson : jsonNode["exposureTimeUs"])
                    {
                        cameras[cameraJson["cameraName"].asString()].exposureTime = static_cast<float>(cameraJson["exposureTimeUs"].asInt());
                    }
                }
                else
                {
                    // New format
                    for (const auto & cameraJson : jsonNode["cameras"])
                    {
                        auto cameraName = cameraJson["cameraName"].asString();
                        auto& cameraSettings = cameras[cameraName];
                        Scanning::Configuration::LensState lcState;
                        if (cameraJson.isMember("lensState"))
                        {
                            cameraSettings.lenState.JsonDeserialize(cameraJson["lensState"]);
                        }
                        cameraSettings.exposureTime = cameraJson["exposureTimeUs"].asFloat();
                        cameraSettings.gain = cameraJson["gain"].asFloat();
                    }
                }

                for (const auto& projectorName : jsonNode["projectorNames"])
                {
                    this->projectorNames.push_back(projectorName.asString());
                }
                this->projectedImageSize = cv::Size(jsonNode["projectedImageSize"][0U].asInt(), jsonNode["projectedImageSize"][1].asInt());
                this->projectedImageIntensity = jsonNode["projectedImageIntensity"].asDouble();
                this->numberOfPatterns = jsonNode["numberOfPatterns"].asInt();
            }
        }
    }
}
