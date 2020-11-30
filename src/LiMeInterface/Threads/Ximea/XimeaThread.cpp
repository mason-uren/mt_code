#include "XimeaThread.h"

bool XimeaThread::createInstance() {
	bool result{};

	this->setCameraSerialNumber(std::to_string(this->config.pcieSlot));

	// Connect camera
	if ((result = this->camera->initializeDevice(this->config.serialNo))) {

		for (auto & param : this->parameters) {
			// Attempt to set param
			int attempts{};
			//std::string returnedParam{};

			// FIXME - re-write; too much repative code
			while (!param->isValid && attempts++ < Presets::Device::MAX_DEVICE_CREATE_ATTEMPTS) {
				switch (param->type) {
				case ParamType::INT: {
					int64_t returnedParam{};
					this->camera->setIntegerParamVal(param->key, std::stoi(param->data));
					this->camera->getIntegerParamVal(param->key, returnedParam);
					param->isValid = returnedParam == std::stoi(param->data);
					break;
				}
				case ParamType::FLOAT: {
					double returnedParam{};
					this->camera->setFloatParamVal(param->key, std::stof(param->data));
					this->camera->getFloatParamVal(param->key, returnedParam);
					param->isValid = returnedParam == std::stof(param->data);
					break;
				}
				case ParamType::BOOL: {
					bool returnedParam{};
					this->camera->setBoolParamVal(param->key, std::stoi(param->data) == 1);
					this->camera->getBoolParamVal(param->key, returnedParam);
					param->isValid = returnedParam == (std::stoi(param->data) == 1);
					break;
				}
				// TODO
				case ParamType::ENUM: {
					this->camera->setEnumParamVal(param->key, (int64_t)std::stoi(param->data));
					break;
				}
				// TODO
				case ParamType::STRING: {
					this->camera->setStringParamVal(param->key, param->data);
					break;
				}
				default:
					this->eHandler->report(
						this->name + " : Unrecognized camera parameter type <" + std::to_string(static_cast<int>(param->type)) + ">",
						Shared::Error::WARN
					);
					break;
				}
			}

			this->logGenIMsg();

			// TODO - remove; allows all parameters
			// Necessary since GenICam doesn't return an appropriate value from getFloatParam()
			param->isValid = true;

			// Exit condition
			// One of the above parameters failed to set
			if (!param->isValid) {
				this->eHandler->report(
					this->name + " : Invalid param <" + param->key + ">",
					Shared::Error::Severity::KILL_AND_EXIT
				);
				return result = false;
			}
			else {
				this->eHandler->report(this->name + " : Param ( " + param->key + ") verified.");
			}
		}
	}

	this->logGenIMsg();
	return result;
}

void XimeaThread::info(Shared::Device::Info * info) {
    strcpy_s(info->buffer[Shared::Device::Info::Setting::NAME], this->name.c_str());

    xiGetDeviceInfoString(
		this->config.pcieSlot, XI_PRM_DEVICE_NAME,
		info->buffer[Shared::Device::Info::Setting::MODEL_NO],
		sizeof(info->buffer[Shared::Device::Info::Setting::MODEL_NO])
	);
    xiGetDeviceInfoString(
		this->config.pcieSlot, XI_PRM_DEVICE_SN,
		info->buffer[Shared::Device::Info::Setting::SERIAL_N0],
		sizeof(info->buffer[Shared::Device::Info::Setting::SERIAL_N0])
    );
}

//void XimeaThread::worker() {
void XimeaThread::run() {
	// The thread will process continuously until we tell it to stop. At a fixed interval, check and see if the frame
   // has updated (how to do this? Does the Ximea send a frame number?) and copy the image data into a rotating buffer.
   // The advantage of the buffer is that we should never have to deal with a case where we need to read an image at the 
   // exact same time that the sensor needs to write to it; i.e., mutex locks shouldn't cause dropped frames.

	this->running = this->initialized;

	if (!this->camera->isAcquiring()) {
		this->triggerAcquisition();
	}

#ifdef XIMEA_THREAD_TIMER
	std::chrono::steady_clock::time_point start;
	std::chrono::duration<double, std::milli> delta;
#endif

	// Run the sensor in a continuous loop, capturing frames to the buffer as we go. The loop break command will come from the main thread when ready.
	while (this->running && this->eHandler->shouldContinue()) {
#ifdef XIMEA_THREAD_TIMER
		start = std::chrono::high_resolution_clock::now();
#endif

        // Collect images in buffer
        this->camera->collect();

        if (this->imgBuffer.pullImageFromCamera(this->camera)) {
            this->camera->popNextImage();
        }
        else {
            this->eHandler->report(
                this->name + " : Failed to pull new image.",
                Shared::Error::Severity::WARN
            );
        }

		this->initialized = true; // block the main thread until we have at least one buffer stored.

		// Log time
#ifdef XIMEA_THREAD_TIMER
		delta = std::chrono::high_resolution_clock::now() - start;
		if (!((int) delta.count() % 1)) { // FIXME - create JSON variable as log timing (or as function of ErrorHandler) ; Itermitentally report thread times
			this->errorHandler->report(this->name + " : Worker Thread: Time ( " + std::to_string(delta.count()) + ")ms");
		}
#endif

        this->logGenIMsg();
	}

	if (this->camera->isAcquiring()) {
        this->camera->endAcquire();
    }

	this->initialized = false; // release any blocking that is waiting on this thread to complete
    this->logGenIMsg();
}

bool XimeaThread::cleanAndExit() {
	this->running = false;
	// 'initialized' will be set to false by the worker thread when the 'keepRunning' loop exits.
	// Be sure to set 'initialized' to false after your 'keepRunning' loop exits!!!
	// Block here until the worker thread is ready to terminate.

    if (this->camera->isAcquiring()) {
        this->camera->endAcquire();
    }
	
	Sleep(100);

    this->logGenIMsg();
	return !(this->initialized = false);
}

