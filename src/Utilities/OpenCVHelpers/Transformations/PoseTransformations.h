#ifndef METROLOGY2020_POSE_TRANSFORMATIONS_H
#define METROLOGY2020_POSE_TRANSFORMATIONS_H

#include <iostream>
#include <string>
#include <cmath>

// OpenCV
#include <opencv2/core/core.hpp>

template<typename T>
constexpr T SQ(T x) { return ((x)*(x)); }

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

//#######################################################################################
inline bool isRotationMatrix(const cv::Mat &R)
{
	// Determine if a matrix is a valid rotation matrix
	// https://www.learnopencv.com/rotation-matrix-to-euler-angles/

	cv::Mat Rt;
	cv::transpose(R, Rt);
	cv::Mat shouldBeIdentity = Rt * R;
	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

	return cv::norm(I, shouldBeIdentity) < 1.e-6;
}

//######################################################################################
inline void dumpRvecTvec(const cv::Mat & rvec, cv::Mat & tvec)
{
	//Print rvec (given in radians, print in degrees) and tvec on the console
	cv::Mat rvecDeg = rvec.clone();
	std::cout << "tvec = " << tvec.t() << std::endl;
	std::cout << "rvec = " << rvec.t() << std::endl;
	std::cout << "rvec(deg) = " << rvecDeg.t() / M_PI * 180.0 << std::endl;
}

//#######################################################################################
inline bool mat2euler(const cv::Mat &rmat, cv::Mat &rvec)
{
	// Transform a rotation matrix into euler angles (roll, pitch, yaw)
	// https://www.learnopencv.com/rotation-matrix-to-euler-angles/

	if (!isRotationMatrix(rmat)) {
		std::cout << "ERROR! This is not a valid rotation matrix:" << std::endl << rmat << std::endl;
		return false;
	}

	rvec.create(3, 1, CV_64F);
	double sy = sqrt(SQ(rmat.at<double>(0, 0)) + SQ(rmat.at<double>(1, 0)));
	if (sy < 1.e-6)
	{
		// singular
		rvec.at<double>(0, 0) = atan2(-rmat.at<double>(1, 2), rmat.at<double>(1, 1));
		rvec.at<double>(1, 0) = atan2(-rmat.at<double>(2, 0), sy);
		rvec.at<double>(2, 0) = 0.0;
	}
	else
	{
		// Not singular
		rvec.at<double>(0, 0) = atan2(rmat.at<double>(2, 1), rmat.at<double>(2, 2));
		rvec.at<double>(1, 0) = atan2(-rmat.at<double>(2, 0), sy);
		rvec.at<double>(2, 0) = atan2(rmat.at<double>(1, 0), rmat.at<double>(0, 0));
	}
	return true;
}

//#######################################################################################
inline bool decompose_extrinsics(const cv::Mat &extrinsics_4x4, cv::Mat & rotmat3x3, cv::Mat &tvec)
{
	// The rotation matrix is the upper-left 3x3 matrix of the 4x4 extrinsics matrix
	//rotmat3x3.create(3, 3, extrinsics_4x4.type());
	extrinsics_4x4(cv::Range(0, 3), cv::Range(0, 3)).copyTo(rotmat3x3);

	// The translation vector is the last column of the 4x4 extrinsics matrix
	//tvec.create(3, 1, extrinsics_4x4.type());
	extrinsics_4x4(cv::Range(0, 3), cv::Range(3, 4)).copyTo(tvec);

	return true;
}

//#######################################################################################
inline cv::Mat compose_extrinsics(const cv::Mat rvec, const cv::Mat tvec)
{
	cv::Mat rot3x3;
	cv::Rodrigues(rvec, rot3x3);   // Convert the rotation vector into rotation matrix via Rodrigues algorithm

	// Build a 4x4 matrix by stacking the rvec and tvec and adding the appropriate padding
	// Then assign it to the return matrix
	return (cv::Mat_<double>(4, 4) << rot3x3.at<double>(0, 0), rot3x3.at<double>(0, 1), rot3x3.at<double>(0, 2), tvec.at<double>(0, 0),
		rot3x3.at<double>(1, 0), rot3x3.at<double>(1, 1), rot3x3.at<double>(1, 2), tvec.at<double>(1, 0),
		rot3x3.at<double>(2, 0), rot3x3.at<double>(2, 1), rot3x3.at<double>(2, 2), tvec.at<double>(2, 0),
		0.0, 0.0, 0.0, 1.0);
}

#endif // METROLOGY2020_POSE_TRANSFORMATIONS_H
