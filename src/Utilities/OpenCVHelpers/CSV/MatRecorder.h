#ifndef METROLOGY2020_CSVRECORDER_H
#define METROLOGY2020_CSVRECORDER_H

#include <string>
#include <fstream>

// OpenCV
#include <opencv2/core/core.hpp>

// HRL
#include <ErrorHandler/ErrorHandler.h>

class MatRecorder {
public:
	explicit MatRecorder(ErrorHandler * instance = ErrorHandler::getInstance()) : eHandler(instance) {}
	~MatRecorder() {}

	bool save(const std::string & file = "CSVRecorder_data.csv");
	void write(const cv::Mat & data);

private:
	std::ofstream fileHandle;
	cv::Mat data;

	// Static
	ErrorHandler * eHandler;
};

#endif // !METROLOGY2020_CSVRECORDER_H

