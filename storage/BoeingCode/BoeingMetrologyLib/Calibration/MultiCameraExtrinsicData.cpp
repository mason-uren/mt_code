#include "MultiCameraExtrinsicData.h"
#include "json/reader.h"
#include <fstream>
#include "json/writer.h"
#include "TypeDefs.h"
#include <map>
#include <iostream>

using namespace BoeingMetrology;

void BoeingMetrology::Calibration::MultiCameraExtrinsicData::JsonDeserialize(const Json::Value &jsonNode)
{
    try
    {
        this->cameraData.clear();

        for (const Json::Value & camera : jsonNode)
        {
            ExtrinsicData extrinsicData;
            extrinsicData.JsonDeserialize(camera);
            this->cameraData[extrinsicData.name] = extrinsicData;
        }
    }
    catch (...)
    {
        throw;
    }
}

void BoeingMetrology::Calibration::MultiCameraExtrinsicData::JsonSerialize(Json::Value &jsonNode) const
{
    jsonNode = Json::Value(Json::arrayValue); // init with a valid empty array rather than a null object

    try
    {
        for (const auto &cameraExtrinsics : this->cameraData)
        {
            std::string cameraName = cameraExtrinsics.first;

            Json::Value individualCamera;
            cameraExtrinsics.second.JsonSerialize(individualCamera);
            jsonNode.append(individualCamera);
        }
    }
    catch (...)
    {
        throw;
    }
}

double BoeingMetrology::Calibration::MultiCameraExtrinsicData::ComputeMagnitudeOfDistanceBetweenCameraPair(const std::string & camName1, const std::string & camName2) const
{
    if (this->cameraData.find(camName1) == this->cameraData.end())
        throw std::runtime_error("MultiCameraExtrinsicData::ComputeMagnitudeOfDistanceBetweenCameraPair Error: " + camName1 + " not found");
    if (this->cameraData.find(camName2) == this->cameraData.end())
        throw std::runtime_error("MultiCameraExtrinsicData::ComputeMagnitudeOfDistanceBetweenCameraPair Error: " + camName2 + " not found");

    try
    {
        // Compute the distance
        return cv::norm(this->cameraData.at(camName1).GetTranslationVector() - this->cameraData.at(camName2).GetTranslationVector());
    }
    catch (...)
    {
        throw std::runtime_error("MultiCameraExtrinsicData::ComputeMagnitudeOfDistanceBetweenCameraPair failed");
    }
}

cv::Point3d BoeingMetrology::Calibration::MultiCameraExtrinsicData::ComputePositionVectorBetweenCameraPair(const std::string & camName1, const std::string & camName2) const
{
    if (this->cameraData.find(camName1) == this->cameraData.end())
        throw std::runtime_error("MultiCameraExtrinsicData::ComputePositionVectorBetweenCameraPair Error: " + camName1 + " not found");
    if (this->cameraData.find(camName2) == this->cameraData.end())
        throw std::runtime_error("MultiCameraExtrinsicData::ComputePositionVectorBetweenCameraPair Error: " + camName2 + " not found");

    try
    {
        // Compute the distance
        cv::Mat diff = this->cameraData.at(camName2).GetTranslationVector() - this->cameraData.at(camName1).GetTranslationVector();
        cv::Point3d output(diff);
        return output;
    }
    catch (...)
    {
        throw std::runtime_error("MultiCameraExtrinsicData::ComputePositionVectorBetweenCameraPair failed");
    }
}

void BoeingMetrology::Calibration::MultiCameraExtrinsicData::ComputeMagnitudeOfDistanceBetweenAllCameraPairs(std::map < std::pair<std::string, std::string>, double> & distances) const
{
    try
    {
        // Construct a vector of names
        std::vector<std::string> names;
        for (const auto & name : this->cameraData)
        {
            names.push_back(name.first);
        }

        // Loop through all pairs
        for (int idx = 0; idx < (int)names.size() - 1; idx++)
        {
            for (int j = idx + 1; j < (int)names.size(); j++)
            {
                // Compute distance
                std::pair<std::string, std::string> namepair = { names[idx], names[j] };
                double distance = this->ComputeMagnitudeOfDistanceBetweenCameraPair(names[idx], names[j]);
                distances[namepair] = distance;
            }
        }
    }
    catch (...)
    {
        throw std::runtime_error("MultiCameraExtrinsicData::ComputeMagnitudeOfDistanceBetweenAllCameraPairs failed");
    }
}

void BoeingMetrology::Calibration::MultiCameraExtrinsicData::ComputePositionVectorBetweenAllCameraPairs(std::map < std::pair<std::string, std::string>, cv::Point3d> & distances) const
{
    try
    {
        // Construct a vector of names
        std::vector<std::string> names;
        for (const auto & name : this->cameraData)
            names.push_back(name.first);

        // Loop through all pairs
        for (int i = 0; i < (int)names.size() - 1; i++)
        {
            for (int j = i + 1; j < (int)names.size(); j++)
            {
                // Compute distance
                std::pair<std::string, std::string> namepair = { names[i], names[j] };
                cv::Point3d distance = this->ComputePositionVectorBetweenCameraPair(names[i], names[j]);
                distances[namepair] = distance;
            }
        }
    }
    catch (...)
    {
        throw std::runtime_error("MultiCameraExtrinsicData::ComputePositionVectorBetweenAllCameraPairs failed");
    }
}

void BoeingMetrology::Calibration::MultiCameraExtrinsicData::GetCommonReferenceCameraName(std::string & refCameraName) const
{
    refCameraName = "";
    for (const auto & ext : this->cameraData)
    {
        if (refCameraName == "")
            refCameraName = ext.second.refCameraName;
        else if (refCameraName != ext.second.refCameraName)
            throw std::runtime_error("MultiCameeraExtrinsicData::GetCommonReferenceCameraName: Multiple reference cameras found!");
    }
}

void BoeingMetrology::Calibration::MultiCameraExtrinsicData::RedefineReferenceCamera(const std::string & newRefCameraName)
{
    // Get the name of the current reference camera
    std::string currentRefCameraName = "";
    try
    {
        this->GetCommonReferenceCameraName(currentRefCameraName);
    }
    catch (...)
    {
        throw;
    }

    if (newRefCameraName != currentRefCameraName)
    {
        if (this->cameraData.count(newRefCameraName) == 1)
        {
            // Get the transform from new to old
            auto newToOldRot = this->cameraData[newRefCameraName].GetRotationMatrix().clone();
            auto newToOldTrans = this->cameraData[newRefCameraName].GetTranslationVector().clone();

            // Loop through cameras
            for (auto & camera : this->cameraData)
            {
                // Shift this camera to the new frame
                auto posInOldFrame = camera.second.GetTranslationVector();
                camera.second.SetTranslationVector(newToOldRot.t() * (posInOldFrame - newToOldTrans));

                // Rotate this camera into the new frame
                auto rotInOldFrame = camera.second.GetRotationMatrix();
                camera.second.SetRotationMatrix(newToOldRot.t() * rotInOldFrame);

                // Update the reference camera's name
                camera.second.refCameraName = newRefCameraName;
            }
        }
        else
            throw std::runtime_error("MultiCameraExtrinsicData::RedefineReferenceCamera: Specified reference camera not found!");
    }
}

void BoeingMetrology::Calibration::MultiCameraExtrinsicData::ComputeDepthAlongCameraZ(const std::pair<CAMERA_NAME, double> & src, const CAMERA_NAME & dstCameraName, double & dst) const
{
    // Compute the angle between the two cameras' z-axes.  This is the incident angle where they intersect on the surface.
    cv::Point3d srcOpticalAxis = this->cameraData.at(src.first).GetOpticalAxisVector();
    cv::Point3d dstOpticalAxis = this->cameraData.at(dstCameraName).GetOpticalAxisVector();

    double theta = std::acos(srcOpticalAxis.ddot(dstOpticalAxis)); // radians

    // Compute the vector from the destination camera to the source camera, specified in the reference frame
    cv::Point3d vecFromDstToSrc = this->ComputePositionVectorBetweenCameraPair(dstCameraName, src.first);

    // Handle special case of parallel optical axes
    if (std::abs(theta) < 1e-5)
    {
        std::cout << "MultiCameraExtrinsicData::ComputeDepthAlongCameraZ: Sensors are parallel! " << src.first << ", " << dstCameraName << std::endl;
        dst = src.second + srcOpticalAxis.ddot(vecFromDstToSrc);
        return;
    }

    // Compute the angle opposite the provided depth value
    double alpha = std::acos(dstOpticalAxis.ddot(vecFromDstToSrc / cv::norm(vecFromDstToSrc)));

    // Compute the third angle
    const double pi = 3.14159265358979323846264338;
    double beta = pi - theta - alpha;

    std::cout << cv::norm(vecFromDstToSrc) << ", " << alpha * 180.0 / pi << ", " << beta * 180.0 / pi << ", " << theta * 180.0 / pi << std::endl;

    // We have now defined a proportional triangle based on extrinsics directional vectors.  Now we effectively scale the triangle by the provided distance.
    // This is Law of Sines.
    double salpha = std::sin(alpha);
    dst = src.second * std::sin(beta) / salpha;

    if ((theta + alpha > pi) || std::abs(salpha) < 1e-5 || std::isnan(dst))
        throw std::runtime_error("MultiCameraExtrinsicData::ComputeDepthAlongCameraZ: Returning NaN.  Input depth and extrinsics violate the triangle property.");
}

std::map<CAMERA_NAME_PAIR, double> BoeingMetrology::Calibration::MultiCameraExtrinsicData::getCameraPairDistance() const
{
    std::map<CAMERA_NAME_PAIR, double> distances;
    ComputeMagnitudeOfDistanceBetweenAllCameraPairs(distances);
    return distances;
}

double BoeingMetrology::Calibration::MultiCameraExtrinsicData::getExtrinsicDataDistance(ExtrinsicMetricEnum::Type type, const MultiCameraExtrinsicData &obj, bool &mismatchedIndices) const
{
    mismatchedIndices = false;

    if (ExtrinsicMetricEnum::LocationDistance == type)
    {
        double total = 0.0;
        for (const auto &cameraExtrinsics : this->cameraData)
        {
            CAMERA_NAME cameraName = cameraExtrinsics.first;

            if (obj.cameraData.count(cameraName) > 0)
            {
                cv::Mat diff = cameraExtrinsics.second.GetTranslationVector() - obj.cameraData.at(cameraName).GetTranslationVector();
                cv::Point3d output(diff);
                total += cv::norm(output);
            }
            else
            {
                mismatchedIndices = true;
            }
        }
        return total;
    }
    else if (ExtrinsicMetricEnum::LocationAngleDistance == type)
    {
        double total = 0.0;
        for (const auto &cameraExtrinsics : this->cameraData)
        {
            CAMERA_NAME cameraName = cameraExtrinsics.first;

            if (obj.cameraData.count(cameraName) > 0)
            {
                cv::Mat diff = cameraExtrinsics.second.GetTranslationVector() - obj.cameraData.at(cameraName).GetTranslationVector();
                cv::Mat diff2 = cameraExtrinsics.second.GetRotationMatrix() - obj.cameraData.at(cameraName).GetRotationMatrix();
                cv::Point3d output(diff);
                total += cv::norm(diff2);
                total += cv::norm(output);
            }
            else
            {
                mismatchedIndices = true;
            }
        }
        return total;
    }
    return 0.0;
}
