#include <opencv2/calib3d/calib3d.hpp>
#include "SixDOF.h"
#include "OpenCVHelpers/Transformations/PoseTransformations.h"

void SixDOF::reset()
{
    roll=pitch=yaw=x=y=z=0.0;
    num_aruco_markers_found = num_interpolated_corners_found = 0;
    reprojection_error = 0.0;
    extrinsics.release();
}

// Set pitch, roll, yaw and x, y, z according to the given extrinsics matrix
bool SixDOF::set_pose(const cv::Mat & extrinsics4x4)
{
    cv::Mat rmat, tvec;
    decompose_extrinsics(extrinsics4x4, rmat, tvec);

    // Recover the Euler angles from the 3x3 rotation matrix
    cv::Mat euler_angle;

    if(!mat2euler(rmat, euler_angle)) {
        std::cout << "ERROR! This is not a valid rotation matrix:" << std::endl << rmat << std::endl;
        return false; // 'rmat' is not a valid rotation matrix
    }

// Populate the 6 DOF fields with the results from Euler angles and the tvec
    pitch = rad2deg(euler_angle.at<double>(0, 0));      // rotation about the x-axis (in degrees)
    yaw   = rad2deg(euler_angle.at<double>(1, 0));        // rotation about the y-axis (in degrees)
    roll  = rad2deg(euler_angle.at<double>(2, 0));       // rotation about the z-axis (in degrees)
    x = tvec.at<double>(0, 0);                   // translation along the x-axis (in meters)
    y = tvec.at<double>(1, 0);                   // translation along the y-axis (in meters)
    z = tvec.at<double>(2, 0);                   // translation along the z-axis (in meters)

    extrinsics = extrinsics4x4;

    return true;
}

std::string SixDOF::toString() const {
    std::stringstream sstream{};
    sstream << "x:" << this->x << " y:" << this->y << " z:" << this->z
            << " pitch:" << this->pitch << " yaw:" << this->yaw << " roll:" << this->roll;
            //<< "Pan: " << this->pan << " Tilt: " << this->tilt << std::endl;
	return sstream.str();
}
