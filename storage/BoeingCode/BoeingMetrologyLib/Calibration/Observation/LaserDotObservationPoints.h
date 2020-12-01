#ifndef BOEINGMETROLOGYLIB_LASERDOTOBSERVATIONPOINTS_H
#define BOEINGMETROLOGYLIB_LASERDOTOBSERVATIONPOINTS_H

#include <string>
#include <cv.h>
#include <opencv2/imgproc.hpp>
#include <map>
#include <fstream>
#include "Common/Interface/Serializer.h"
#include "json/writer.h"
#include "json/reader.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		namespace Observation
		{
            class LaserDotObservationPoints : public Boeing::Interface::Serializer
			{
			public:

                // Name of camera
				std::string cameraName = "";

                // Image
                cv::Mat image;

                // Pose identifier
                std::string poseName = "";

                // Pixel coordinates
                std::vector<cv::Point2f> positions;

                LaserDotObservationPoints()
                {

                }

                // Deep copy the input image
                LaserDotObservationPoints(const std::string & cameraNameIn, const cv::Mat & imageIn, const std::string & poseNameIn = "")
                {
                    this->cameraName = cameraNameIn;
                    this->poseName = poseNameIn;
                    this->image = imageIn.clone();
                }

                // Return a deep copy of the image with the selected pixel position highlighted
                cv::Mat GetImageOverlaidWithLaserDot(const uint & index, const cv::Scalar & color = cv::Scalar(0, 0, 255))
                {
                    try
                    {
                        int radius = 25;
                        cv::Mat dest = this->image.clone();
                        cv::circle(dest, this->positions[index], radius, color, 6, cv::LINE_AA);
                        return dest;
                    }
                    catch (...)
                    {
                        throw;
                    }
                }


                // Return a deep copy of the image with the selected pixel positions highlighted
                cv::Mat GetImageOverlaidWithLaserDot(const std::vector<uint> & indices)
                {
                    try
                    {
                        int radius = 25;
                        cv::Mat dest = this->image.clone();
                        for (const auto & index : indices)
                            cv::circle(dest, this->positions[index], radius, cv::Scalar(0, 0, 255), 6, cv::LINE_AA);
                        return dest;
                    }
                    catch (...)
                    {
                        throw;
                    }
                }
                
                // Serialize as a json (except image)
				void JsonSerialize(Json::Value &jsonNode) const
				{
					// Camera info
					jsonNode["cameraName"] = cameraName;
                    jsonNode["poseName"] = poseName;

                    for (const auto & pos : positions)
                    {
                        Json::Value p;
                        p.append(pos.x);
                        p.append(pos.y);
                        jsonNode["positions"].append(p);
                    }
				}

                // Populate the object from a json.  Image is ignored.
				void JsonDeserialize(const Json::Value &jsonValue)
				{
					try
					{
						this->cameraName = jsonValue["cameraName"].asString();
                        this->poseName = jsonValue["poseName"].asString();
                        this->positions.clear();
                        for (const auto & node : jsonValue["positions"])
                        {
                            cv::Point2f pos;
                            pos.x = (float)node[0U].asDouble();
                            pos.y = (float)node[1].asDouble();
                            this->positions.push_back(pos);
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
					//int index = 0; // This variable is not used
					//Read through the JSON file and collect the points
					if (jsonReader.parse(strm, root))
					{
						JsonDeserialize(root);
					}
					else
					{
                        throw std::runtime_error("LaserDotObservationPoints::DeserializeStream: Failed to parse json.");
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
}


#endif