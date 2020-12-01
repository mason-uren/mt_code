#ifndef VORONOI_DIAGRAM_H
#define VORONOI_DIAGRAM_H

#include "opencv2/opencv.hpp"

namespace BoeingMetrology
{
	namespace ImageProcessing
	{
		void voronoi_nomask(cv::Mat& labim, cv::Mat& distim, cv::Mat& pout_x, cv::Mat& pout_y, int qmax);
		void voronoi(cv::Mat& labim, cv::Mat mask,
			cv::Mat& distim, cv::Mat& pout_x, cv::Mat& pout_y, int qmax);
		void voronoi(cv::Mat& labim, cv::Mat mask, cv::Mat& distim, int qmax);
	}
}

#endif // VORONOI_DIAGRAM_H
