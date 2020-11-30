#include "PhotoAlbum.h"

bool PhotoAlbum::pullImageFromCamera(GenICamAdapterInterface * camera) {
    bool status{};
    GenICamAdapterInterface::RawImageType image{};

    if ((status = camera->getNextImage(image))) {
		// Check for valid image and duplicate images
		if (!(status = image.info.isValid) && this->geniImageId != image.info.frameID) {
			return status;
		}

		this->geniImageId = image.info.frameID;

		// Circular Buffer
		if (++this->activeID >= Presets::Imaging::CIRCULAR_BUF_SIZE) {
			this->activeID = 0;
		}
		this->bufFrameNum[this->activeID] = ++this->frameNum;

		auto mat{ this->genImageToMat(image) };

		std::unique_lock<std::mutex> lock(this->mutex);
		this->buffer[this->activeID] = mat;
		this->hasNewImage = true;
		this->lockConditional.notify_all();
    }

	return status;
}

cv::Mat PhotoAlbum::getImage() {
	std::unique_lock<std::mutex> lock(this->mutex);
	this->lockConditional.wait(lock, [&] {return this->hasNewImage; });

	auto img{ this->buffer[this->activeID] };
	this->hasNewImage = false;

	return img;
}

int PhotoAlbum::getFrameNumber() {
    return this->bufFrameNum[this->activeID];
}

cv::Mat PhotoAlbum::genImageToMat(const GenICamAdapterInterface::RawImageType &image) {
	return cv::Mat(image.info.height, image.info.width, CV_8UC(this->imageChannels), image.buffer->data()).clone();
}