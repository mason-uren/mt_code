#ifndef BOEINGMETROLOGYLIB_STEREORECONSTRUCT3D_H
#define BOEINGMETROLOGYLIB_STEREORECONSTRUCT3D_H

#include <string>
#include <map>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "Calibration/IntrinsicData.h"
#include "Calibration/ExtrinsicData.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    namespace StereoReconstruct3d
	{
        class BOEINGMETROLOGYLIB_API StereoReconstruct3d : public Boeing::Interface::Serializer
		{
		public:
            // Create a disparity map
            static void CreateDisparityMap(const std::pair<BoeingMetrology::Calibration::IntrinsicData, cv::Mat> & leftimage, 
                const std::pair<BoeingMetrology::Calibration::IntrinsicData, cv::Mat> & rightimage, const bool & rectifyImages, cv::Mat & disp);

            // Create a point cloud from a stereo pair
            static void CreatePointCloudFromStereoPair(const std::pair<BoeingMetrology::Calibration::IntrinsicData, cv::Mat> & leftimage,
                const std::pair<BoeingMetrology::Calibration::IntrinsicData, cv::Mat> & rightimage, const cv::Mat & R, const cv::Mat & T, const bool & rectifyImages, cv::Mat & xyz);

            // Export a point cloud to file
            static void SaveXYZ(const char* filename, const cv::Mat& xyz);

		};
	}
}

#endif
