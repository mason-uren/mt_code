#pragma once
#include <vector>
#include <map>
#include <cv.h>
#include "json/json.h"

#include "BoeingMetrologyLib_API.h"


namespace BoeingMetrology
{
	namespace ImageCharacteristics
	{		
		class BOEINGMETROLOGYLIB_API FocusMeasure
		{
		public:
			// OpenCV port of 'LAPM' algorithm (Nayar89)
			static double modifiedLaplacian(const cv::Mat& src);

			// OpenCV port of 'LAPV' algorithm (Pech2000)
			static double varianceOfLaplacian(const cv::Mat& src);

			// OpenCV port of 'TENG' algorithm (Krotkov86)
			static double tenengrad(const cv::Mat& src, int ksize);

			// OpenCV port of 'GLVN' algorithm (Santos97)
			static double normalizedGraylevelVariance(const cv::Mat& src);

			// For a greyscale image of a black and white object, compute a sharpness metric of the image.  This is done by discarding outliers,
			// normalizing the image, and then computing standard deviations of pixels of the two black-and-white modes.  The output
			// metric is an average of these two standard deviations.  The smaller the metric value, the sharper the image.  
			static double bimodalVariance(const cv::Mat & src, std::pair<double, double> & meanStd1, std::pair<double, double> & meanStd2, cv::Mat & threshMask, const cv::Mat mask = cv::Mat(), const double & lowerPerc = 0.1, const double & upperPerc = 0.9);
		};

		class BOEINGMETROLOGYLIB_API PoseDiagnostics
		{
		public:
			PoseDiagnostics() {};
			PoseDiagnostics(const double & focusMetricIn, const double & hv_ExposureScoreIn, const double & hv_HomogeneityScoreIn,
				const double & hv_ContrastScoreIn, const double & hv_SizeMarksIn, const std::string & hv_MessageIn,
				const double & hv_FocusScoreIn, const std::vector<double> & rowIn, const std::vector<double> & colIn,
				const std::vector<double> & indexIn, const std::vector<double> & poseIn, const std::vector<int> & poseIndexIn);

			double focusMetric;
			double hv_ExposureScore;
			double hv_HomogeneityScore;
			double hv_ContrastScore;
			double hv_SizeMarks;
			std::string hv_Message;
			double hv_FocusScore;
			std::vector<double> row;
			std::vector<double> col;
			std::vector<double> index;
			std::vector<double> pose;
			std::vector<int> poseIndex;

			bool Deserialize(const std::string & jsonFname);
			bool SerializeAndWrite(const std::string & jsonFname, const Json::Value & objpoints_root);
		};

		class BOEINGMETROLOGYLIB_API PoseDiagnosticsGroup
		{
		public:
			PoseDiagnosticsGroup() {};

			void ReportFeatureExtrema();

			void ExportFeatureCSV(const std::string & fname);

			void InsertPose(const std::string & poseDir, const std::string & sensorName, const PoseDiagnostics & diag);

			std::map<std::string, std::vector<std::pair<BoeingMetrology::ImageCharacteristics::PoseDiagnostics, std::string>>> data;
		};
	}
}

