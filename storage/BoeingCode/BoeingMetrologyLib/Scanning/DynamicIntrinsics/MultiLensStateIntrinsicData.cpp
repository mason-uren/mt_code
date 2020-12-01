#include "MultiLensStateIntrinsicData.h"
#include <mutex>
#include <thread>
#include "opencv2/calib3d.hpp"
#include "Utilities/Utilities.h"

BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::MultiLensStateIntrinsicData(const std::string & name)
{
    this->name = name;
}

BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::MultiLensStateIntrinsicData(const Scanning::Observation::MultiLensStateCameraObservations & multiLensStateCameraObservations,
    const Calibration::IntrinsicData & initialIntrinsicData, const int & flags, const int & minNumCorners, const double & maxReprojThresh, std::set<CAMERA_NAME> & failedCameras)
{
    // Set camera name
    this->name = initialIntrinsicData.name;

    // Initialize multi-threading stuff
    std::vector<std::thread*> threads;
    std::recursive_mutex multiLensStateIntrinsicDataLock;

    // Common timestamp across the group
    const std::string timestamp = BoeingMetrology::Utilities::getCurrentTimeInSeconds();
    
    // Loop through lens states for this camera
    for (const auto & lensState : multiLensStateCameraObservations.GetLensStates(this->name))
    {
        // Compute intrinsics for this camera and lens state on a thread
        threads.push_back(new std::thread([this, &multiLensStateIntrinsicDataLock, &multiLensStateCameraObservations, &initialIntrinsicData, lensState, minNumCorners, maxReprojThresh, flags, &failedCameras, timestamp]()
        {
            try
            {
                // Get all observations for this camera and lens state that satisfy the quality criteria
                std::vector<std::vector<cv::Point2f>> imagePoints;
                std::vector<std::vector<cv::Point3f>> objectPoints;
                std::vector<POSE_NAME> poseNames;
                multiLensStateCameraObservations.GetObservations(name, lensState, initialIntrinsicData, imagePoints, objectPoints, poseNames, minNumCorners, maxReprojThresh);

                // Initialize the intrinsics
                Calibration::IntrinsicData intrinsicData = initialIntrinsicData.Clone();
                intrinsicData.poseDataPath = poseNames.front();
                intrinsicData.timestamp = timestamp;
                cv::Size imageSize(initialIntrinsicData.imageSize.first, initialIntrinsicData.imageSize.second);

                // Compute intrinsics with remaining observations
                std::vector<cv::Mat> rvecs, tvecs;
                intrinsicData.rmsError = cv::calibrateCamera(objectPoints, imagePoints, imageSize, intrinsicData.cameraMatrix, intrinsicData.distortionCoeffs, rvecs, tvecs, flags);

                // Thread-safe update this class
                {
                    std::lock_guard<std::recursive_mutex> locker(multiLensStateIntrinsicDataLock);
                    this->AddLensState(lensState, intrinsicData);
                    std::cout << "MultiLensStateIntrinsicData: Successfully computed " << this->name << " intrinsics for f = " << lensState.GetFocusValue() << ".  Rms = "  << intrinsicData.rmsError << std::endl;
                }
            }
            catch (std::exception & e)
            {
                std::lock_guard<std::recursive_mutex> locker(multiLensStateIntrinsicDataLock);
                std::cout << "MultiLensStateIntrinsicData: Failed to compute intrinsics for " << this->name << ".  " << e.what() << std::endl;
                failedCameras.insert(this->name);
            }
            catch (...)
            {
                std::lock_guard<std::recursive_mutex> locker(multiLensStateIntrinsicDataLock);
                std::cout << "MultiLensStateIntrinsicData: Failed to compute intrinsics for " << this->name << ".  " << std::endl;
                failedCameras.insert(this->name);
            }
        })); // End lambda
    } // End loop through lens states
    
    // Wait for threads to finish
    std::for_each(threads.begin(), threads.end(), [](std::thread* x){ x->join(); });
    
    // Free threads
    for (unsigned iter = 0; iter < threads.size(); iter++) delete threads[iter];
}

std::string BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::GetCameraName() const
{
    return this->name;
}

std::string BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::GetTimestamp() const
{
    if (this->intrinsics.size() == 0)
        throw std::runtime_error("MultiLensStateIntrinsicData::GetTimestamp() there are no cameras loaded!");

    std::string timestamp = "";
    for (const auto & camera : this->intrinsics)
    {
        if (timestamp == "")
            timestamp = camera.second.timestamp;
        else if (timestamp != camera.second.timestamp)
            throw std::runtime_error("MultiLensStateIntrinsicData::GetTimestamp() NOT all cameras have an identical timestamp");
    }
    if (timestamp == "")
        throw std::runtime_error("MultiLensStateIntrinsicData::GetTimestamp() intrinsic timestamps are empty!");

    return timestamp;
}

void BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::AddLensState(const Scanning::Configuration::LensState & lensState, 
    const Calibration::IntrinsicData & intrinsic)
{
    // Lens state and intrinsics must be for the same camera
    if (lensState.GetDeviceName() != intrinsic.name)
        throw std::runtime_error("MultiLensStateIntrinsicData::AddLensState: Failed to add camera because camera names of lens state and intrinsic data do not match.");

    if (this->name == "")
        this->name = lensState.GetDeviceName();
    if (intrinsic.name == this->name)
        this->intrinsics[lensState] = intrinsic;
    else
        throw std::runtime_error("MultiLensStateIntrinsicData::AddLensState: Failed to add camera because camera name is wrong.");
}

BoeingMetrology::Calibration::IntrinsicData BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::GetIntrinsicDataExact(const Scanning::Configuration::LensState & lensState) const
{
    // Loop through intrinsics
    for (const auto & state : this->intrinsics)
    {
        if (state.first == lensState)
            return state.second;
    }
    throw std::runtime_error("MultiLensStateIntrinsicData::GetIntrinsicDataExact: No match found");
}

int BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::GetIntrinsicDataNearestNeighbor(const Scanning::Configuration::LensState & lensState, 
    std::pair<Scanning::Configuration::LensState, Calibration::IntrinsicData> & nearestNeighbor) const
{
    int resultFocusAbsDiff = std::numeric_limits<int>::max();

    // Loop through intrinsics
    for (const auto & state : this->intrinsics)
    {
        if (state.first.GetZoomValue() == lensState.GetZoomValue() && std::abs(state.first.GetApertureValue() - lensState.GetApertureValue()) < 1e-3)
        {
            // We have a matching zoom and aperture state.  Now compare focus values.
            int focusAbsDiff = std::abs(state.first.GetFocusValue() - lensState.GetFocusValue());

            if (focusAbsDiff < resultFocusAbsDiff)
            {
                // Update the result
                resultFocusAbsDiff = focusAbsDiff;
                nearestNeighbor = state;
            }
        }
    }

    if (resultFocusAbsDiff == std::numeric_limits<int>::max())
        throw std::runtime_error("MultiLensStateIntrinsicData::GetIntrinsicDataNearestNeighbor: No match found");

    return resultFocusAbsDiff;
}

void BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::GetIntrinsicDataLinearlyInterpolate(const Scanning::Configuration::LensState & lensState, 
    std::pair<Scanning::Configuration::LensState, Calibration::IntrinsicData> & interpolatedState) const
{
    throw std::runtime_error("MultiLensStateIntrinsicData::GetIntrinsicDataLinearlyInterpolate : Not yet implemented");
}

void BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::JsonDeserialize(const Json::Value &jsonNode)
{
    try
    {
        // Camera name
        this->name = jsonNode["name"].asString();

        this->intrinsics.clear();

        // Loop through states
        for (const auto & stateJson : jsonNode["intrinsicsPerLensState"])
        {
            Calibration::IntrinsicData stateIntrinsics;
            stateIntrinsics.JsonDeserialize(stateJson["intrinsics"]);
            Scanning::Configuration::LensState state;
            state.JsonDeserialize(stateJson["lensState"]);
            this->intrinsics[state] = stateIntrinsics;
        }
    }
    catch (...)
    {
        throw std::runtime_error("MultiLensStateIntrinsicData::JsonDeserialize failed");
    }
}

void BoeingMetrology::Scanning::DynamicIntrinsics::MultiLensStateIntrinsicData::JsonSerialize(Json::Value &jsonNode) const
{
    try
    {
        // Camera name
        jsonNode["name"] = this->name;

        // Loop through states
        for (const auto & state : this->intrinsics)
        {
            Json::Value stateJson;
            state.first.JsonSerialize(stateJson["lensState"]);
            state.second.JsonSerialize(stateJson["intrinsics"]);
            jsonNode["intrinsicsPerLensState"].append(stateJson);
        }
    }
    catch (...)
    {
        throw std::runtime_error("MultiLensStateIntrinsicData::JsonSerialize failed");
    }
}
