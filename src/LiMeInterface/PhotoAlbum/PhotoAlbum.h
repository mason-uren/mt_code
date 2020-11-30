#ifndef METROLOGY2020_PHOTOALBUM_H
#define METROLOGY2020_PHOTOALBUM_H

#include <iostream>
#include <array>
#include <vector>
#include <condition_variable>


// OpenCV
#include <opencv2/core/core.hpp>

// Boeing
#include "Adapters/GenICamInterface/GenICamAdapterInterface.h"
// HRL
#include <Shared/InterfaceConfig.h>
#include <Shared/Presets.h>
#include <Shared/OSDefines.h>

/**
 * @class PhotoAlbum
 * @brief Collection of the most current images indexed by device serial number
 * @author Mason U'Ren
 */
class PhotoAlbum {
public:
    PhotoAlbum(const Interface::Imaging::Image & imageConfig) :
		imageChannels(imageConfig.pixelDepth),
		buffer(Presets::Imaging::CIRCULAR_BUF_SIZE, cv::Mat{}),
        bufFrameNum(Presets::Imaging::CIRCULAR_BUF_SIZE, -1),
        activeID(-1),
        frameNum(-1),
		hasNewImage(false),
		geniImageId(0)
    {}
	PhotoAlbum(const PhotoAlbum &) {};
    ~PhotoAlbum() = default;

	bool pullImageFromCamera(GenICamAdapterInterface * camera);
	// Blocking call... (till new image is populated)
	cv::Mat getImage(); // Mats are smart pointers
	int getFrameNumber();

protected:
	bool hasNewImage{};

private:
	// Functions
	cv::Mat genImageToMat(const GenICamAdapterInterface::RawImageType &image);

	// Variables
	std::mutex mutex;
	std::condition_variable lockConditional;

	std::vector<cv::Mat> buffer{};
	std::vector<int> bufFrameNum{};

	int imageChannels{};
	int activeID{};
	int frameNum{};

	uint64_t geniImageId{};
};

#endif // METROLOGY2020_PHOTOALBUM_H