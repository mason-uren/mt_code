#include "MultiCameraObservation.h"
#include "CameraObservation.h"

namespace BoeingMetrology
{
namespace Calibration
{
namespace Observation
{

namespace
{
    const std::string JSON_POSE_ARRAY_TAG = "MultiCameraObservation";
    const std::string JSON_POSE_ID_TAG = "PoseID";
    const std::string JSON_POSE_IDENTIFIER_TAG = "PoseIdentifier";
    const std::string JSON_CAMERA_NAME_TAG = "CameraName";
    const std::string JSON_POSE_DATA_TAG = "PoseData";
}

void MultiCameraObservation::JsonSerialize(Json::Value &jsonNode) const
{
    try
    {
        //Generate the metadata with the observations
        Json::Value poseArray = Json::Value(Json::arrayValue);

        for (const auto &pose : this->cameraObservation)
        {
            std::string cameraName = pose.first;

            Json::Value individualCamera;
            individualCamera[JSON_CAMERA_NAME_TAG] = cameraName;
            pose.second.JsonSerialize(individualCamera[JSON_POSE_DATA_TAG]);
            poseArray.append(individualCamera);
        }
        jsonNode[JSON_POSE_ARRAY_TAG] = poseArray;
        jsonNode[JSON_POSE_ID_TAG] = poseId;
        jsonNode[JSON_POSE_IDENTIFIER_TAG] = poseIdentifier;
    }
    catch (...)
    {
        throw;
    }
}

void MultiCameraObservation::JsonDeserialize(const Json::Value& jsonValue)
{
    try
    {
        poseId = jsonValue[JSON_POSE_ID_TAG].asInt();
        poseIdentifier = jsonValue[JSON_POSE_IDENTIFIER_TAG].asString();

        Json::Value poseArray = jsonValue[JSON_POSE_ARRAY_TAG];
        if (!poseArray.isArray())
        {
            return;
        }
        auto numElems = poseArray.size();
        for (auto i = decltype(numElems)(0); i < numElems; ++i)
        {
            Json::Value cameraNode = poseArray[i];
            std::string cameraName = cameraNode[JSON_CAMERA_NAME_TAG].asString();
            cameraObservation[cameraName].JsonDeserialize(cameraNode[JSON_POSE_DATA_TAG]);
        }
    }
    catch (...)
    {
        throw;
    }
}

void MultiCameraObservation::RetrieveCameraObservationPoints(
    const std::string &cameraName, std::vector<std::vector<int>> &cameraMarkerIds,
    std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
    std::vector<std::vector<cv::Point2f>> &cameraImagePoints,
    cv::Size &imageSize,
    std::vector<std::vector<int>> *cameraPoseIndex /*= NULL*/)
{
    //Retrieve the observations for a single camera into a friendly format for opencv calibrateCamera

    if (this->contains(cameraName))
    {
        CameraObservation &cameraObs = this->GetCameraPose(cameraName);
        std::vector <ObservationPoints<float>::CONTROL_POINT> controlPoints;
        //Points relative to the image plane (in pixels)
        std::vector <ObservationPoints<float>::OBSERVATION_POINT> observedPoints;
        //Marker identifier to correlate across
        std::vector<int> markerIdentifier;
        cameraObs.GetObservations(markerIdentifier, observedPoints, controlPoints);
        cameraMarkerIds.push_back(markerIdentifier);
        cameraObjectPoints.push_back(controlPoints);
        cameraImagePoints.push_back(observedPoints);

        if (cameraPoseIndex != NULL)
        {
            //Generate parallel pose index constant for this pose
            std::vector <int> poseValues;
            for (int count = 0; count < controlPoints.size(); ++count)
            {
                poseValues.push_back(poseId);
            }
            cameraPoseIndex->push_back(poseValues);
        }

        //Image size is constant across the camera
        imageSize = cameraObs.imageSize;
    }
}

void MultiCameraObservation::RetrieveCameraObservationPoints(
    const std::string &cameraName, std::vector<std::vector<int>> &cameraMarkerIds,
    std::vector<std::vector<cv::Point3f >> &cameraObjectPoints,
    std::vector<std::vector<cv::Point2f>> &cameraImagePoints,
    cv::Size &imageSize,
    std::vector<std::pair<std::vector<int>, std::string>> *cameraPoseIndex /*= NULL*/)
{
    //Retrieve the observations for a single camera into a friendly format for opencv calibrateCamera

    if (this->contains(cameraName))
    {
        CameraObservation &cameraObs = this->GetCameraPose(cameraName);
        std::vector <ObservationPoints<float>::CONTROL_POINT> controlPoints;
        //Points relative to the image plane (in pixels)
        std::vector <ObservationPoints<float>::OBSERVATION_POINT> observedPoints;
        //Marker identifier to correlate across
        std::vector<int> markerIdentifier;
        cameraObs.GetObservations(markerIdentifier, observedPoints, controlPoints);
        cameraMarkerIds.push_back(markerIdentifier);
        cameraObjectPoints.push_back(controlPoints);
        cameraImagePoints.push_back(observedPoints);

        if (cameraPoseIndex != NULL)
        {
            //Generate parallel pose index constant for this pose
            std::vector <int> poseValues;
            for (int count = 0; count < controlPoints.size(); ++count)
            {
                poseValues.push_back(poseId);
            }
            cameraPoseIndex->push_back({ poseValues, cameraObs.fileNameOfObservation });
        }

        //Image size is constant across the camera
        imageSize = cameraObs.imageSize;
    }
}

std::map<MARKER_IDENTIFIER, std::map<CAMERA_NAME, int>> MultiCameraObservation::retrieveSharedMarkers()
{
	std::map<MARKER_IDENTIFIER, std::map<CAMERA_NAME, int>> result = std::map<MARKER_IDENTIFIER, std::map<CAMERA_NAME, int>>();
	for (auto camObservation : cameraObservation)
	{
		CAMERA_NAME cam = camObservation.first;
		CameraObservation observation = camObservation.second;
		for (int i = 0; i < observation.markerIdentifier.size(); ++i)
		{
			MARKER_IDENTIFIER markerId = observation.markerIdentifier[i];
			if (result.count(markerId) != 0)
			{
				result[markerId][cam] = i;
			}
			else
			{
				std::map<CAMERA_NAME, int> entry = std::map<CAMERA_NAME, int>();
				entry[cam] = i;
				result[markerId] = entry;
			}
		}
	}

	return result;
}

void MultiCameraObservation::FilteredObservationsByCamera(MultiCameraObservation &obj, const CAMERA_REGEX_STRING &cameraRegex)
{
    obj.poseIdentifier = this->poseIdentifier;
    obj.poseId = this->poseId;

    for (auto &observation : cameraObservation)
    {
        CAMERA_NAME name = observation.first;
        if (BoeingMetrology::Utilities::RegexMatch(cameraRegex, name))
        {
            obj.AddCameraPose(name, observation.second);
        }
    }
}

void MultiCameraObservation::AddCameraPose(const CAMERA_NAME &cameraName, const CameraObservation &cameraObservationPose)
{
    cameraObservation[cameraName] = cameraObservationPose;
}

CameraObservation & MultiCameraObservation::GetCameraPose(const CAMERA_NAME &cameraName)
{
    return cameraObservation[cameraName];
}

CameraObservation MultiCameraObservation::GetCameraPoseCopy(const CAMERA_NAME &cameraName) const
{
    return cameraObservation.at(cameraName);
}

bool MultiCameraObservation::contains(const std::string &cameraName) const
{
    return (cameraObservation.count(cameraName) != 0);
}
}//namespace Observation
}//namespace Calibration
}//namespace BoeingMetrology

