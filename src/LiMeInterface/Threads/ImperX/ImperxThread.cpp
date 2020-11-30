#include "ImperxThread.h"

bool ImperxThread::createInstance() {
	static uint attempts{};
	bool result{};

	// Connect camera
	result = this->camera->initializeDevice(this->config.serialNo);

	this->logGenIMsg();
	return result;
}

void ImperxThread::info(Shared::Device::Info * info) {
    strcpy_s(info->buffer[Shared::Device::Info::Setting::NAME], this->name.c_str());
    strcpy_s(info->buffer[Shared::Device::Info::Setting::SERIAL_N0], this->camera->getSerialNumber().c_str());
}

//void ImperxThread::worker() {
void ImperxThread::run() {
	// The thread will process continuously until we tell it to stop. At a fixed interval, check and see if the frame
	// has updated (how to do this? Does the Ximea send a frame number?) and copy the image data into a rotating buffer.
	// The advantage of the buffer is that we should never have to deal with a case where we need to read an image at the 
	// exact same time that the sensor needs to write to it; i.e., mutex locks shouldn't cause dropped frames

	this->running = this->initialized;

	if (!this->camera->isAcquiring()) {
		this->triggerAcquisition();
	}

#ifdef IMPERX_THREAD_TIMER
	std::chrono::steady_clock::time_point start;
	std::chrono::duration<double, std::milli> delta;
#endif

	// Run the sensor in a continuous loop, capturing frames to the buffer as we go. The loop break command will come from the main thread when ready.
	//while (this->keepRunning)
	while (this->running && this->eHandler->shouldContinue()) {
#ifdef IMPERX_THREAD_TIMER
		start = std::chrono::high_resolution_clock::now();
#endif
        if (this->imgBuffer.pullImageFromCamera(this->camera)) {
            this->camera->popNextImage();
        }
        else {
			// FIXME - need to report but need to resole GenICam logic
			//		 - will polute logger with <Failed to pull...> messages
            //this->eHandler->report(                 
            //    this->name + "- Failed to pull new image.",
            //    Shared::Error::Severity::WARN
            //);
        }

        /*
         * TODO - consider not spamming
         *      - static call? or just remove?
         */
		this->initialized = true; // block the main thread until we have at least one buffer stored.

		// Log time
#ifdef IMPERX_THREAD_TIMER
		delta = std::chrono::high_resolution_clock::now() - start;
		if (!((int)delta.count() % 50)) { // FIXME - create JSON variable as log timing (or as function of ErrorHandler) ; Itermitentally report thread times
			this->eHandler->report(this->name + "- Worker Thread: Time ( " + std::to_string(delta.count()) + ")ms");
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

bool ImperxThread::cleanAndExit() {
	this->running = false;
	// 'initialized' will be set to false by the worker thread when the 'keepRunning' loop exits.
	// Be sure to set 'initialized' to false after your 'keepRunning' loop exits!!!
	// Block here until the worker thread is ready to terminate.

    if (this->camera->isAcquiring()) {
        this->camera->endAcquire();

		// Reset device in an attempt to prevent camera falling into error state 
		this->camera->reset();
		this->eHandler->report(				
			this->name + "- Reset camera",
			Shared::Error::Severity::WARN
		);
    }

	Sleep(100);

	this->logGenIMsg();
	return !(this->initialized = false);
}
