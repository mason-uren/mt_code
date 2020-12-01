#ifndef BOEINGMETROLOGY_BLOBDETECTOR_HPP
#define BOEINGMETROLOGY_BLOBDETECTOR_HPP

#include "opencv2/features2d.hpp"

using namespace cv;


namespace cv2
{
	class CV_EXPORTS_W SimpleBlobDetector : public Feature2D
	{
	public:
		struct CV_EXPORTS_W_SIMPLE Params
		{
			CV_WRAP Params();
			CV_PROP_RW float thresholdStep;
			CV_PROP_RW float minThreshold;
			CV_PROP_RW float maxThreshold;
			CV_PROP_RW size_t minRepeatability;
			CV_PROP_RW float minDistBetweenBlobs;

			CV_PROP_RW bool filterByColor;
			CV_PROP_RW uchar blobColor;

			CV_PROP_RW bool filterByArea;
			CV_PROP_RW float minArea, maxArea;

			CV_PROP_RW bool filterByCircularity;
			CV_PROP_RW float minCircularity, maxCircularity;

			CV_PROP_RW bool filterByInertia;
			CV_PROP_RW float minInertiaRatio, maxInertiaRatio;

			CV_PROP_RW bool filterByConvexity;
			CV_PROP_RW float minConvexity, maxConvexity;

			CV_PROP_RW bool filterByAreaGrouping;
			CV_PROP_RW float groupSplitRatio;
			CV_PROP_RW int minAreaGroupSize;
			CV_PROP_RW bool keepAreaGroupingWithLargestArea;
			CV_PROP_RW int desiredGroupSize;
			CV_PROP_RW bool removeEdgeContours;

			void read(const FileNode& fn);
			void write(FileStorage& fs) const;
		};

		CV_WRAP static Ptr<cv2::SimpleBlobDetector>
			create(const cv2::SimpleBlobDetector::Params &parameters = cv2::SimpleBlobDetector::Params());
		CV_WRAP virtual String getDefaultName() const override;
	};
}
#endif