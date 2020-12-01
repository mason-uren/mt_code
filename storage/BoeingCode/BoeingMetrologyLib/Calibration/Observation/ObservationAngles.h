#ifndef BOEINGMETROLOGYLIB_OBSERVATIONANGLES_H
#define BOEINGMETROLOGYLIB_OBSERVATIONANGLES_H

#include <string>
#include "Common/Interface/Serializer.h"
#include "Utilities/Histogram.h"
#include <cv.h>
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Calibration
	{
		namespace Observation
		{
            // Observed pose angles in degrees
            class BOEINGMETROLOGYLIB_API ObservationAngles : public Boeing::Interface::Serializer
			{
            private:
                // Histograms of rotations about X, Y, Z.  Values should be degrees.
                BoeingMetrology::Histogram rx, ry, rz;

                void SerializeStream(std::ostream &strm) const;

                void DeserializeStream(std::istream &strm);

			public:

                // Initialize histograms with nominal bin ranges
                ObservationAngles();

                // Modulate angles
                // 0 <= rx <= 360
                // ry unconstrained
                // 0 <= rz < 360
                void ModulateObservation(const double & rxsrc, const double & rysrc, const double & rzsrc, double & rxdst, double & rydst, double & rzdst);

                // Add an observation (angles are degrees)
                void AddObservation(const double & rxnew, const double & rynew, const double & rznew);
                
                // Serialize this class as a json
				void JsonSerialize(Json::Value &jsonNode) const;

                // Deserialize the json into this class
				void JsonDeserialize(const Json::Value &jsonValue);

                // Serialize this class and write json to file
				void SerializeFile(std::string fileName) const;

                // Write a CSV file with contents of one of the rotation angle dimensions
                void WriteToCsv(const std::string & filename, const std::string & rstr = "rx");

                // Deserialize a json-formatted text file to this class
				void DeserializeFile(std::string fileName);

                // Create an image to display the histogram and write it to file
                void WriteHistImage(const std::string & filename, const std::string & rstr = "rx");

                // Get histogram images of each rotation angle
                void GetHistImages(cv::Mat & rximg, cv::Mat & ryimg, cv::Mat & rzimg) const
                {
                    std::pair<double, double> minMaxBin;
                    size_t maxCount;
                    this->rx.GetImage(cv::Scalar(0, 0, 255), rximg, minMaxBin, maxCount);
                    this->ry.GetImage(cv::Scalar(0, 255, 0), ryimg, minMaxBin, maxCount);
                    this->rz.GetImage(cv::Scalar(255, 0, 0), rzimg, minMaxBin, maxCount);
                }
			};
		}
	}
}

#endif
