#include "MultiLensStateCameraObservations.h"
#include <fstream>

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::SerializeStream(std::ostream &strm) const
{
    // Camera info
    Json::Value root;
    JsonSerialize(root);
    Json::StyledStreamWriter json_writer;
    json_writer.write(strm, root);
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::DeserializeStream(std::istream &strm)
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
        throw std::runtime_error("MultiLensStateCameraObservations::DeserializeStream(): Failed to parse json.");
    }
}

std::vector<std::string> BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::GetCameraNames() const
{
    std::vector<std::string> names;
    for (const auto & cameras : this->observations)
        names.push_back(cameras.first);
    return names;
}

std::vector<BoeingMetrology::Scanning::Configuration::LensState> BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::GetLensStates(const std::string & cameraName) const
{
    if (this->observations.count(cameraName) == 0)
        throw std::runtime_error("MultiLensStateCameraObservations::GetLensStates: camera not found " + cameraName);
    
    std::vector<Scanning::Configuration::LensState> states;
    for (const auto & state : this->observations.at(cameraName))
        states.push_back(state.first);
    return states;
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::AddObservation(const std::string & cameraName, 
    const Scanning::Configuration::LensState & lensState, const std::string & poseName, const Calibration::Observation::CameraObservation & observation)
{
    std::pair<POSE_NAME, Calibration::Observation::CameraObservation> poseObs = { poseName, observation };
    this->observations[cameraName][lensState].insert(poseObs);
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::GetObservations(const std::string & cameraName, 
    const Scanning::Configuration::LensState & lensState, const Calibration::IntrinsicData & intrinsicData, 
    std::map<POSE_NAME, Calibration::Observation::CameraObservation> & observationList, const int & minNumCorners, const double & maxReprojThresh) const
{
    if (this->observations.count(cameraName) == 0 && this->observations.at(cameraName).count(lensState) == 0)
        throw std::runtime_error("MultiLensStateCameraObservations::GetObservations: " + cameraName + " not found for this lens state");

    // Deep copy the contents
    observationList = this->observations.at(cameraName).at(lensState);

    // Loop through poses
    std::vector<POSE_NAME> badPoses;
    for (auto & pose : observationList)
    {
        // Number of markers check
        if (pose.second.controlPoints.size() < minNumCorners)
            badPoses.push_back(pose.first);
        else
        {
            // Perform 3d pose estimation and discard observations that have poor reprojection error
            cv::Mat r, t;
            std::map<MARKER_IDENTIFIER, cv::Point2f> outliers;
            //std::cout << pose.first << std::endl;
            pose.second.rms = pose.second.Estimate3dPose(intrinsicData, r, t, outliers, true);
            if (pose.second.rms > maxReprojThresh)
                badPoses.push_back(pose.first);
        }
    }

    // Erase the poses
    size_t numObsBefore = observationList.size();
    for (const auto & poseName : badPoses)
        observationList.erase(poseName);

    // Validate the output
    size_t numObsAfter = observationList.size();

    if (numObsAfter == 0)
        throw std::runtime_error("MultiLensStateCameraObservations::GetObservations: " + cameraName + " Every single pose was filtered out!");

    std::cout << "MultiLensStateCameraObservations::GetObservations: " << cameraName << " " << numObsAfter << " out of " << numObsBefore << " observations remaining" << std::endl;
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::GetObservations(const std::string & cameraName, const Scanning::Configuration::LensState & lensState, 
    const Calibration::IntrinsicData & intrinsicData, std::vector<std::vector<cv::Point2f>> & imagePoints, std::vector<std::vector<cv::Point3f>> & objectPoints, 
    std::vector<POSE_NAME> & poseNames, const int & minNumCorners /*= 20*/, const double & maxReprojThresh /*= 1.0*/) const
{
    // Get the observations in map format and apply the filters
    std::map<POSE_NAME, Calibration::Observation::CameraObservation> filteredObs;
    this->GetObservations(cameraName, lensState, intrinsicData, filteredObs, minNumCorners, maxReprojThresh);

    // Dump data into desired output format
    for (const auto & obs : filteredObs)
    {
        poseNames.push_back(obs.first);
        imagePoints.push_back(obs.second.observedPoints);
        objectPoints.push_back(obs.second.controlPoints);
    }
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::GetObservations(const std::string & cameraName, 
    std::map<Scanning::Configuration::LensState, std::map<POSE_NAME, Calibration::Observation::CameraObservation>> & observationList) const
{
    if (this->observations.count(cameraName) > 0)
        observationList = this->observations.at(cameraName);
    else
        throw std::runtime_error("MultiLensStateCameraObservations::GetObservations: camera not found " + cameraName);
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::GetObservations(std::map<POSE_NAME, std::map<CAMERA_NAME, std::pair<Scanning::Configuration::LensState, Calibration::Observation::CameraObservation>>> & observationList) const
{
    // Loop through cameras
    for (const auto & camera : this->observations)
    {
        const auto cameraName = camera.first;

        // Loop through lens states for this camera
        for (const auto & lensState : camera.second)
        {
            // Loop through poses for this lens state
            for (const auto & pose : lensState.second)
            {
                observationList[pose.first][cameraName] = { lensState.first, pose.second };
            }
        }
    }
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::JsonSerialize(Json::Value &jsonNode) const
{
    jsonNode["poseDir"] = this->poseDir;

    // Loop through cameras
    for (const auto & camera : this->observations)
    {
        Json::Value cameraJson;
        cameraJson["cameraName"] = camera.first;

        // Loop through lens states for this camera
        for (const auto & ls : camera.second)
        {
            Json::Value lsJson;
            ls.first.JsonSerialize(lsJson["lensState"]);

            // Loop through poses for this camera and lens state
            for (const auto & pose : ls.second)
            {
                Json::Value poseJson;
                poseJson["poseName"] = pose.first;
                pose.second.JsonSerialize(poseJson["cameraObservation"]);
                lsJson["poses"].append(poseJson);
            }
            cameraJson["cameraLensStates"].append(lsJson);
        }
        jsonNode["observations"].append(cameraJson);
    }
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::JsonDeserialize(const Json::Value &jsonNode)
{
    this->poseDir = jsonNode["poseDir"].asString();

    // Loop through cameras
    for (const auto & cameraJson : jsonNode["observations"])
    {
        const CAMERA_NAME & cameraName = cameraJson["cameraName"].asString();

        // Loop through lens states for this camera
        std::map<Scanning::Configuration::LensState, std::map<POSE_NAME, Calibration::Observation::CameraObservation>> lensObs;
        for (const auto & lsJson : cameraJson["cameraLensStates"])
        {
            Scanning::Configuration::LensState lensState;
            lensState.JsonDeserialize(lsJson["lensState"]);

            // Loop through poses for this camera and lens state
            std::map<POSE_NAME, Calibration::Observation::CameraObservation> poseObs;
            for (const auto & poseJson : lsJson["poses"])
            {
                const POSE_NAME & poseName = poseJson["poseName"].asString();
                Calibration::Observation::CameraObservation cameraObservation;
                cameraObservation.JsonDeserialize(poseJson["cameraObservation"]);
                poseObs[poseName] = cameraObservation;
            }
            lensObs[lensState] = poseObs;
        }
        this->observations[cameraName] = lensObs;
    }
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::SerializeFile(const std::string &fileName) const
{
    //Write the serialization to the file
    std::ofstream strm(fileName);
    SerializeStream(strm);
    strm.close();
}

void BoeingMetrology::Scanning::Observation::MultiLensStateCameraObservations::DeserializeFile(const std::string &fileName)
{
    //Read the content from the file and de-serialize into the class
    std::ifstream strm(fileName);
    DeserializeStream(strm);
    strm.close();
}
