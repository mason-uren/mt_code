#include "MatRecorder.h"

bool MatRecorder::save(const std::string & file) {
	// Open file
	this->fileHandle.open(file);
	if (!this->fileHandle.is_open()) {
		this->eHandler->report("Failed to open <" + file + "> for writing.", Shared::Error::Severity::WARN);
		return false;
	}

	// Write to file
	this->fileHandle << cv::format(this->data, cv::Formatter::FMT_CSV) << std::endl;

	// Close file
	this->fileHandle.close();

	// Ensure file is closed
	return !this->fileHandle.is_open();
}

void MatRecorder::write(const cv::Mat & data) {
	if (data.empty()) {
		// No data to add
	}

	this->data.push_back(data);
}
