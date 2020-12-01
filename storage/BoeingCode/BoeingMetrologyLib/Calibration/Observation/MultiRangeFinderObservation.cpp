#include "MultiRangeFinderObservation.h"
#include "json/reader.h"
#include "json/writer.h"
#include <fstream>
#include "Calibration/CameraCalibrationUtilities.h"
#include <iostream>


BoeingMetrology::Calibration::Observation::MultiRangeFinderObservation::MultiRangeFinderObservation(const std::string & rangeFinderName, 
    const std::string & poseDirName, const std::string & refCameraName)
{
    // Parse all the pose folders
    std::string poseDirNameClean = "";
    CameraCalibrationUtilities::CleanFileName(poseDirName, poseDirNameClean);
    std::vector<std::string> poseDirs;
    std::map < std::string, std::vector<std::tuple<std::string, std::string, std::string>>> cameraFiles;
    BoeingMetrology::CameraCalibrationUtilities::getPoseDirectoriesAndFiles(poseDirNameClean, poseDirs, cameraFiles);

    std::cout << rangeFinderName << ": Loading depth values " << std::endl;

    // Loop through cameras
    for (const auto & camera : cameraFiles)
    {
        if (refCameraName != "" && refCameraName != camera.first)
            continue;

        // Loop through pose directories that contain observations for this camera
        for (const auto & poseDir : cameraFiles[camera.first])
        {
            // Load the observation if it exists for the sensor.  Ignore depth values of 0.
            try
            {
                POSE_NAME poseName = std::get<1>(poseDir);
                if (this->observations.count(poseName) == 0)
                {
                    const std::string filename = poseName + "/" + rangeFinderName + "_RangeObservation.json";
                    DeviceIO::RangeFinderObservation rangeFinderObservation;
                    rangeFinderObservation.DeserializeFile(filename);
                    if (rangeFinderObservation.rangeFinderValue != 0.0)
                        this->observations[poseName] = rangeFinderObservation;
                    std::cout << rangeFinderObservation.rangeFinderValue << std::endl;
                }
            }
            catch (...) {}
        }
    }

    size_t numObs = this->observations.size();
    if (numObs == 0)
        std::cout << "MultiRangeFinderObservation: Failed to load any observations" << std::endl;
    else
        std::cout << rangeFinderName << ": Successfully loaded " << numObs << " observations" << std::endl;

    this->rangeFinderName = rangeFinderName;
}

void BoeingMetrology::Calibration::Observation::MultiRangeFinderObservation::JsonSerialize(Json::Value &jsonNode) const
{
    try
    {
        jsonNode["rangeFinderName"]  = this->rangeFinderName;
        for (const auto & pose : this->observations)
        {
            Json::Value poseJson;
            poseJson["poseName"] = pose.first;
            pose.second.JsonSerialize(poseJson["rangeFinderObservation"]);
            jsonNode["observations"].append(poseJson);
        }
    }
    catch (...)
    {
        throw;
    }
}

void BoeingMetrology::Calibration::Observation::MultiRangeFinderObservation::JsonDeserialize(const Json::Value &jsonNode)
{
    this->observations.clear();
    try
    {
        this->rangeFinderName = jsonNode["rangeFinderName"].asString();
        for (const auto & poseJson : jsonNode["observations"])
        {
            POSE_NAME poseName = poseJson["poseName"].asString();
            DeviceIO::RangeFinderObservation rangeFinderObservation;
            rangeFinderObservation.JsonDeserialize(poseJson["rangeFinderObservation"]);
            this->observations[poseName] = rangeFinderObservation;
        }
    }
    catch (...)
    {
        throw;
    }
}

void BoeingMetrology::Calibration::Observation::MultiRangeFinderObservation::SerializeFile(const std::string &fileName) const
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
        throw std::runtime_error("MultiRangeFinderObservation::SerializeFile failed");
    }
}

void BoeingMetrology::Calibration::Observation::MultiRangeFinderObservation::DeserializeFile(const std::string &fileName)
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
            throw std::runtime_error("MultiRangeFinderObservation::DeserializeFile failed");
    }
    catch (...)
    {
        throw std::runtime_error("MultiRangeFinderObservation::DeserializeFile failed");
    }
}