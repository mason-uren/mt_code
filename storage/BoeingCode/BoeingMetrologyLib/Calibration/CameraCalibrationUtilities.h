#ifndef BOEINGMETROLOGYLIB_CAMERACALIBRATIONUTILITIES_H
#define BOEINGMETROLOGYLIB_CAMERACALIBRATIONUTILITIES_H

#include <string>
#include <vector>
#include <map>
#include "cv.h"
#include "json/value.h"

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    class BOEINGMETROLOGYLIB_API CameraCalibrationUtilities
	{
	public:
		static void getPoseDirectoriesAndFiles(const std::string &pathname, std::vector<std::string>& poseDirs,
			std::map<std::string, std::vector<std::tuple<std::string, std::string, std::string>>>& cameraFiles);

        static void FileParts(const std::string & fullFileName, std::string & dir, std::string & filename);

        // Replace part of a string with a new string
        static std::string ReplaceSubString(const std::string & src, const std::string & searchStr, const std::string & replaceStr);

        // Replace double backslashes and single slashes with forward slashes
        static void CleanFileName(const std::string & src, std::string & dst);

        // Compute mean and std of a list of doubles
        static void ComputeDistributionStats(const std::vector<double> &distribution, double &mean, double &stddev);
	};
}

#endif