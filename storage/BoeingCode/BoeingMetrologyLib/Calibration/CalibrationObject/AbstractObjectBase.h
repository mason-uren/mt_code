#ifndef BOEINGMETROLOGYLIB_ABSTRACTOBJECTBASE_H
#define BOEINGMETROLOGYLIB_ABSTRACTOBJECTBASE_H

#include <string>
#include <fstream>
#include <cv.h>
#include "Calibration/Observation/ObservationPoints.h"

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    namespace Calibration
    {
        using namespace Observation;
		namespace CalibrationObject
		{
            class BOEINGMETROLOGYLIB_API AbstractObjectBase : public Boeing::Interface::Serializer
			{
			public:
				virtual void ObjectGenerator(cv::Mat &matOut)
				{
					throw "no method defined";
				}
				virtual void Detector(const cv::Mat &inputImage, const int numRequiredDetectedMarkers, bool &detectedTarget,
					const std::string &cameraName, ObservationPoints<float> &observationPts,
					cv::Mat &targetImageMask, bool doFilter=false, float filterCutoff=10)
				{
					throw std::runtime_error("no method defined");
				}

                virtual void MarkerSanityCheck(const cv::Mat srcImage, std::vector<cv::Point2f> observations, int sanityWindowSize,
                    bool &badMarkersFound, std::vector<cv::Point2f> &badMarkers, cv::Mat &badMarkerImage)
                {
                    throw std::runtime_error("no method defined");
                }

                virtual void JsonSerialize(Json::Value &jsonNode) const
                {
                    throw std::runtime_error("no method defined");
                }
                virtual void JsonDeserialize(const Json::Value &jsonNode)
                {
                    throw std::runtime_error("no method defined");
                }
			};
		}
	}
}

#endif
