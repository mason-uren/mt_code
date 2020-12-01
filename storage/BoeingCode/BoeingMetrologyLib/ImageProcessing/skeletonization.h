#ifndef SKELETON_H
#define SKELETON_H

#include "opencv2/opencv.hpp"
#include <list>
#include "../BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace ImageProcessing
	{
		/*
		* Computes the skeleton of a binary image  using the Voronoi diagram of
		* contour segments split at points of maximal curvature.
		*
		* Input:
		* inMask: input binary image
		* CurvatureKernelSize: gaussian kernel for curvature calculation (15.0)
		* MinCurvatureStrength:  threshold for curvature maxima (0.1)
		*
		* output:
		* skelMask: binary image of the skeleton
		*/
		void BOEINGMETROLOGYLIB_API Voronoi_skeleton(cv::Mat inMask, double CurvatureKernelSize, double MinCurvatureStrength,
			cv::Mat& skelMask);
	}
}


#endif // SKELETON_H
