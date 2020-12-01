#ifndef BOEINGMETROLOGYLIB_IMAGEPLANEPOINTS_H
#define BOEINGMETROLOGYLIB_IMAGEPLANEPOINTS_H

#include <string>
#include <cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <map>
#include <fstream>
#include <string>

#include "Common/Interface/Serializer.h"
#include "json/writer.h"
#include "json/reader.h"
#include "Calibration/IntrinsicData.h"
#include "Common/Utilities/Math/Transformations.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"
#include "Calibration/CameraCalibrationUtilities.h"

namespace BoeingMetrology
{
    namespace ImageProcessing
    {
        template <class Type = float>
        // Observed calibration points and corresponding known geometry of those points 
        // for a single camera and a single pose
        class ImagePlanePoints // : public Boeing::Interface::Serializer
        {
        public:
            typedef cv::Point_<Type> IMAGE_PLANE_POINT;

            cv::Size imageSize;

            //Points relative to the image plane (in pixels)
			std::vector <IMAGE_PLANE_POINT> imagePlanePoints;

            //Marker identifier to correlate across 
            std::vector<MARKER_IDENTIFIER> markerIdentifier;

            void InitializeObservations(const std::string& cameraName_, cv::Size imageSize_)
            {
                this->imageSize = imageSize_;
                this->imagePlanePoints.clear();
                this->markerIdentifier.clear();
            }

			void AddObservation(const MARKER_IDENTIFIER markId, const IMAGE_PLANE_POINT &imagePlanePoint)
            {
                markerIdentifier.push_back(markId);
                imagePlanePoints.push_back(imagePlanePoint);
            }

			void GetObservations(std::vector<MARKER_IDENTIFIER> &markerIdentifier_, std::vector <IMAGE_PLANE_POINT> &imagePlanePoints_) const
            {
                markerIdentifier_.clear();
                imagePlanePoints_.clear();
                markerIdentifier_ = markerIdentifier;
                imagePlanePoints_ = imagePlanePoints;
            }

#if 0
            int ObservationCount() const
            {
                //return the number of observations
                return (int)controlPoints.size();
            }
#endif

            // Overlay the detected pixels onto an image
            void OverlayDetectionsOnImage(cv::Mat & image) const
            {
                try
                {
                    int radius = 25;
                    for (const auto & pt : this->imagePlanePoints)
                        cv::circle(image, pt, radius, cv::Scalar(0, 0, 255), 6, cv::LINE_AA);
                }
                catch (...)
                {
                    throw;
                }
            }

            void JsonSerialize(Json::Value &jsonNode) const
            {
                // Camera info
                jsonNode["ImageHeight"] = imageSize.height;
                jsonNode["ImageWidth"] = imageSize.width;

                //Generate the metadata with the observations
                Json::Value cornerPoints = Json::Value(Json::arrayValue);
                Json::Value actualPoints = Json::Value(Json::arrayValue);
                for (int i = 0; i < imagePlanePoints.size(); i++)
                {
                    Json::Value cornerpoint;
                    cornerpoint["Num"] = i;
                    cornerpoint["observed_x"] = imagePlanePoints[i].x; // image coordinates
                    cornerpoint["observed_y"] = imagePlanePoints[i].y; // image coordinates
                    cornerpoint["id"] = markerIdentifier[i];

                    cornerPoints.append(cornerpoint);
                }

                jsonNode["observation_points"] = cornerPoints;
            }

            void JsonDeserialize(const Json::Value &jsonValue)
            {
                try
                {
                    std::string cName = jsonValue["cameraName"].asString();
                    int iWidth = jsonValue["ImageWidth"].asInt();
                    int iHeight = jsonValue["ImageHeight"].asInt();

                    this->InitializeObservations(cName, cv::Size(iWidth, iHeight));

                    for (const Json::Value & itm : jsonValue["observation_points"])
                    {
                        int markerId;
                        if (itm.isMember("Id"))
                            markerId = itm["Id"].asInt();
                        else
                            markerId = itm["id"].asInt();

                        double obsX = itm["observed_x"].asDouble();
                        double obsY = itm["observed_y"].asDouble();

                        // Get the observed points in image coordinates
                        cv::Point2f observationPoint(static_cast<float>(obsX), static_cast<float>(obsY));
                        this->AddObservation(markerId, observationPoint);
                    }
                }
                catch (...)
                {
                    throw;
                }
            }

            void SerializeStream(std::ostream &strm) const
            {
                // Camera info
                Json::Value root;
                JsonSerialize(root);
                Json::StyledStreamWriter json_writer;
                json_writer.write(strm, root);
            }

            void DeserializeStream(std::istream &strm)
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
                    throw std::runtime_error("ObservationPoints::DeserializeStream(): Failed to parse json.");
                }
            }

            void SerializeFile(std::string fileName) const
            {
                //Write the serialization to the file
                std::ofstream strm(fileName);
                SerializeStream(strm);
                strm.close();
            }

            void DeserializeFile(std::string fileName)
            {
                //Read the content from the file and de-serialize into the class
                std::ifstream strm(fileName);
                DeserializeStream(strm);
                strm.close();
            }
        };
    }
}

#ifdef BOEINGMETROLOGYLIB_EXPORT
template class BOEINGMETROLOGYLIB_API BoeingMetrology::ImageProcessing::ImagePlanePoints<float>;
#else
extern template class BOEINGMETROLOGYLIB_API BoeingMetrology::Calibration::Observation::ObservationPoints<float>;
#endif

#endif
