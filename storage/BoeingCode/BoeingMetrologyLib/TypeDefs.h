#ifndef BOEINGMETROLOGYLIB_BoeingMetrology_TypeDefs_H
#define BOEINGMETROLOGYLIB_BoeingMetrology_TypeDefs_H

#include <string>
#include <vector>
#include <cv.h>

namespace BoeingMetrology
{
	typedef std::string CAMERA_NAME;
	typedef std::string POSE_NAME;
	typedef std::string TIMESTAMP;
	typedef std::pair<std::string, std::string> CAMERA_NAME_PAIR;
	typedef std::string CAMERA_REGEX_STRING;
	typedef std::string REGEX_STRING;
	typedef std::pair<std::string, std::string> CAMERA_MARKER_PAIR;
	typedef std::vector<cv::Point3f> LASER_DOT_COORDINATE;
	typedef std::vector<cv::Point3f> CHESS_CORNERS_COORDINATE;
    typedef std::string PROJECTOR_NAME;
    typedef int MARKER_IDENTIFIER;
}

#endif