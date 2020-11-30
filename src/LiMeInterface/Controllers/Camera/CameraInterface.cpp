#include "CameraInterface.h"

bool CameraInterface::setup(void * params) {
	bool result{};
	if ((result = this->createInstance())) {
		this->triggerAcquisition();
	}

    this->logGenIMsg();
	return result;
}

bool CameraInterface::listen(void * outRef, const bool sendQuery) {
	auto img{ static_cast<cv::Mat *>(outRef) };
	return this->getCurrentFrame(*img) > BAD_FRAME;
}

bool CameraInterface::step(void * input) {
    // TODO - IMPLEMENT ME!
	return true;
}

void * CameraInterface::getDevice() {
	return this->camera;
}

Presets::Device::Type CameraInterface::getType() const {
	switch (this->config.type) {
	case Interface::Imaging::Type::XIMEA:
		return Presets::Device::Type::XIMEA;
	case Interface::Imaging::Type::IMPERX:
		return Presets::Device::Type::IMPERX;
	default:
		this->eHandler->report("Unknown CameraInterface config type <" + std::to_string(static_cast<int>(this->config.type)) + ">.", Shared::Error::Severity::KILL_AND_EXIT);
		return Presets::Device::Type::BAD_DEVICE_TYPE;
	}
}

void CameraInterface::cliListener(const int & key) {

}

void CameraInterface::triggerAcquisition() {
	if (!this->camera->isAcquiring()) {
		this->camera->beginAcquire();
	}

	this->logGenIMsg();
}

bool CameraInterface::isReady() {
	return this->camera->isAcquiring();
}

void CameraInterface::logGenIMsg() {
	static std::unordered_map<std::string, unsigned long long> _messageFrequencyMap{};

    auto genIMsg{GenICamAdapterInterface::LogMessageType{}};
    while (this->camera->getNextLog(genIMsg)) {
        Shared::Error::Severity severity{};
        switch (genIMsg.severity) {
            case GenICamAdapterInterface::SeverityEnumType::INFO_MSG: severity = Shared::Error::Severity::INFO; break;
            case GenICamAdapterInterface::SeverityEnumType::DEBUG_MSG: severity = Shared::Error::Severity::DEBUG; break;
            case GenICamAdapterInterface::SeverityEnumType::WARNING_MSG: severity = Shared::Error::Severity::WARN; break;
            case GenICamAdapterInterface::SeverityEnumType::ERROR_MSG: severity = Shared::Error::Severity::KILL_AND_EXIT; break;
        }


		// Attempt to not spam logger with repeat messages 
		// Keep track of total messages - use message as key
		// FIXME - dislike hard-code numbers; consider move to JSON
		_messageFrequencyMap[genIMsg.what]++;
		if (_messageFrequencyMap[genIMsg.what] <= 5) {
			this->eHandler->report(genIMsg.what, severity);
		}
		else if (_messageFrequencyMap[genIMsg.what] == 6) {
			this->eHandler->report("Repeat messsage - [ " + genIMsg.what + "] - will be ignored. See... CameraInterface.cpp");
		}
        
        this->camera->popNextLog();
    }
}

void CameraInterface::displayLogMessage(GenICamAdapterInterface::LogMessageType &message) {
	while (this->camera->getNextLog(message)) {
		std::cout << "[ " << message.severity << "] : " << message.what << std::endl;
		this->camera->popNextLog();
	}
}

int CameraInterface::getCurrentFrame(cv::Mat & img) {
	img = this->imgBuffer.getImage();
	return this->imgBuffer.getFrameNumber();
}

void CameraInterface::setCameraSerialNumber(const std::string &serial) {
	this->camera->setSerialNumber(serial);
	this->config.serialNo = this->camera->getSerialNumber();
}
