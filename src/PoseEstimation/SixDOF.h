#ifndef __SIXDOF
#define __SIXDOF

#include <sstream>
#include <string>

// OpenCV
#include <opencv2/core/mat.hpp>
class SixDOF
{
public:
    SixDOF(): roll(0.0),pitch(0.0),yaw(0.0),x(0.0),y(0.0),z(0.0),
              num_aruco_markers_found(0), num_interpolated_corners_found(0),
              reprojection_error(0.0),pan(0.0),tilt(0.0)
    {}

	// Functions
    bool set_pose(const cv::Mat & extrinsics4x4);
    void reset();
	std::string toString() const;

	// Variables
    double  roll, pitch, yaw;
    double  x, y, z;
    cv::Mat extrinsics;         // equivalent to xyz + pitch/yaw/roll but more conevient for computing
    int num_aruco_markers_found;      // number of ChArUco squares found in frame
    int num_interpolated_corners_found;
    double reprojection_error;       // reprojection error RMS
    double  pan, tilt;          // pan and tilt angles of the PT unit

};

#endif //__SIXDOF
