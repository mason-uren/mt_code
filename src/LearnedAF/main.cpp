#include <exception>
#include <iostream>
#include <thread>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#if CV_VERSION_MAJOR  >= 4
#include <opencv2/imgcodecs/legacy/constants_c.h>
#endif

#include "xiApi.h"

#include "LIME/GenICamSDK.h"
#include "LIME/GenICamSDKAutoFocusing.h"

// Aaron added
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <ctime>
#include <fstream>
#include <direct.h>
#include <time.h>

#define HandleResult(res,place) if (res!=XI_OK) {printf("Error after %s (%d)",place,res);}


// User-defined constants
#define FOLDERNAME "C:/Users/aofeldman/Desktop/AFdataCollection"
// Initialize lower bound on the focus position to consider when randomly moving, be mindful that does not exceed the mechanical limit
#define LOWERFOCUS 460
// Initialize upper bound on the focus position to consider when randomly moving
#define UPPERFOCUS 660
#define APERTURE_LEVEL 11
#define EXPOSURE_TIME 100000 // microsec
#define GAIN 12 // dB, 12 is upper limit
#define WHITE_BALANCE true

cv::Mat resizeImage(cv::Mat inputImage, float scale) {
	cv::Mat outputImage;
	cv::resize(inputImage, outputImage, cv::Size(int(scale * inputImage.cols), int(scale * inputImage.rows)));
	return outputImage;
}

cv::Mat cvtMatImage(XI_IMG image) {
	// CODE FOR SAVING IMAGE
	int width = image.width;
	int height = image.height;
	// 0 indicates XI_MONO8, 8 bits per pixel grayscale
	//std::cout << "Image data format " << image.frm << std::endl;
	unsigned char* pixel = (unsigned char*)image.bp;
	return cv::Mat(height, width, CV_8UC1, pixel);
}



std::string type2str(int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

double mean(std::vector<double> inputs) {
	double total = 0;
	for (int i = 0; i < inputs.size(); i++) {
		total += inputs[i];
	}
	return total / inputs.size();
}

double standDev(std::vector<double> inputs) {
	double mu = mean(inputs);
	double var = 0;
	for (int i = 0; i < inputs.size(); i++) {
		var += (inputs[i] - mu) * (inputs[i] - mu) / (inputs.size() - 1);
	}
	return sqrt(var);
}

bool keyCheck(std::clock_t &lastCountedPress, double waitPeriod, int key) {
	if (GetKeyState(key) & 0x8000) {
		//std::cout << "Registered key press for " << key << std::endl;
		double duration = ((double)(std::clock()) - (double)(lastCountedPress)) / (double)CLOCKS_PER_SEC;
		// Only register at most one keystroke per waitPeriod (and after first waitPeriod of running program)
		if (duration > waitPeriod) {
			//std::cout << "Key press was accepted " << std::endl;
			lastCountedPress = std::clock();
			//std::cout << key << " was pressed" << std::endl;
			return true;
		}
	}
	return false;
}

int searchForFocus(std::string filename) {
	std::fstream file;
	file.open(filename, std::ios::in);

	std::string line;
	while (std::getline(file, line)) {
		// See if the line contains the desired focus number
		std::string prefix = "Best focus position is ";
		std::string suffix = ", focus value";
		std::size_t foundPrefix = line.find(prefix);
		std::size_t foundSuffix = line.find(suffix);
		std::cout << "Current line is: " << line << std::endl;
		if (foundPrefix != std::string::npos && foundSuffix != std::string::npos) {
			std::size_t start = foundPrefix + prefix.length(); // Inclusive
			std::size_t end = foundSuffix; // Exclusive
			std::string focusPositionStr = line.substr(start, end - start);
			return std::stoi(focusPositionStr, nullptr, 10);
		}
	}

	return -1;
}

int moveToFocusPos(HANDLE deviceHandle, int &currPos, int desiredPos) {
	XI_RETURN ret0 = xiSetParamInt(deviceHandle, XI_PRM_LENS_MODE, XI_ON);
	HandleResult(ret0, XI_PRM_LENS_MODE);
	XI_RETURN ret1 = xiSetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVEMENT_VALUE, desiredPos - currPos);
	HandleResult(ret1, XI_PRM_LENS_FOCUS_MOVEMENT_VALUE);
	XI_RETURN ret2 = xiSetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVE, 0);
	HandleResult(ret2, XI_PRM_LENS_FOCUS_MOVE);

	// Sleep to allow focus motor movement
	std::this_thread::sleep_for(std::chrono::seconds(3));

	if ((ret0 != XI_OK) || (ret1 != XI_OK) || (ret2 != XI_OK)) {
		std::cout << "Move failed" << std::endl;
		return -1;
	}
	else {
		// Update the focus motor position to reflect the move
		currPos = desiredPos;
		std::cout << "Successfully moved to " << currPos << std::endl;
		return 0;
	}
}


int main(int argc, char** argv)
{
	// The GenICamSDKContext has internal arguments parsing for this demo with the values
	// then being used by the perform() method below (When called with no arguments)
	// Usage: lenscontroldemo.exe [OPTIONS]
	 // --help                         Show help summary
	 // --verbose                      Verbose, default off
	 // --serialnumber [Serial number] Serial number of the Ximea device
	 // --searchtype [Search type]     Search type, fullrange search=1, neighbouring search=2
	 // --rangestart [Range start]     Focus search range start, default = 0 for searchtype 1 and -100 for serchtype 2
	 // --rangeend [Range end]         Focus search range end, default = 2000 for searchtype 1 and 100 for serchtype 2
	 // --partition [Partition]        Partitions per round, default = 6 for searchtype 1 and 4 for serchtype 2

	auto& context = BoeingMetrology::GenICamSDKContext::getInstance();

	// GenICamSDKContext must be initialized once before use
	if (!context.initialize(argc, argv, "lenscontroldemo"))
	{
		context.autoFocusing().showHelp();
		return 0;
	}

	// Get the device handle
	HANDLE deviceHandle;
	bool ret = context.autoFocusing().getDeviceHandler(deviceHandle);
	if (!ret) {
		std::cout << "Unable to find device handle " << std::endl;
	}

	// Get the device serialnumber
	char sn[100] = "";
	XI_RETURN gotSN = xiGetParamString(deviceHandle, XI_PRM_DEVICE_SN, sn, sizeof(sn));
	HandleResult(gotSN, XI_PRM_DEVICE_SN);
	char * serialNumber = sn;

	// Keep track of the focus motor position throughout the process
	int focusMotorPosition = 0;

	int numImagesSaved = 0; // Keep track of number images saved
	XI_RETURN state = XI_OK;

	// Initialize random seed
	srand(time(NULL));

	// Demonstration of retrieving the the device handle from GenICamSDKContext to make direct Ximea API calls in client code
	if (ret)
	{
		if (context.autoFocusing().lockDeviceMutex())
		{
			xiSetParamInt(deviceHandle, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
			int acquiring;
			xiGetParamInt(deviceHandle, XI_PRM_ACQUISITION_STATUS, &acquiring);
			if (acquiring)
			{
				xiStopAcquisition(deviceHandle);
			}
			xiStartAcquisition(deviceHandle);

			// Now, turn on lens mode and set all the relevant camera parameters
			XI_RETURN stat0 = xiSetParamInt(deviceHandle, XI_PRM_LENS_MODE, XI_ON);
			HandleResult(stat0, XI_PRM_LENS_MODE);
			XI_RETURN stat1;
			do {
				stat1 = xiSetParamFloat(deviceHandle, XI_PRM_LENS_APERTURE_VALUE, APERTURE_LEVEL);
				if (stat1 != XI_OK) {
					std::cout << "Failed to update aperture, retrying" << std::endl;
				}
			} while (stat1 != XI_OK);
			HandleResult(stat1, XI_PRM_LENS_APERTURE_VALUE);
			XI_RETURN stat2 = xiSetParamInt(deviceHandle, XI_PRM_EXPOSURE, EXPOSURE_TIME);
			HandleResult(stat2, XI_PRM_EXPOSURE);

			XI_RETURN stat3 = xiSetParamFloat(deviceHandle, XI_PRM_GAIN, GAIN);
			HandleResult(stat3, XI_PRM_GAIN);

			int gain;
			XI_RETURN ret = xiGetParamInt(deviceHandle, XI_PRM_GAIN, &gain);
			HandleResult(ret, XI_PRM_GAIN);
			std::cout << "Gain achieved was " << gain << std::endl;

			if (WHITE_BALANCE) {
				XI_RETURN stat4 = xiSetParamInt(deviceHandle, XI_PRM_AUTO_WB, WHITE_BALANCE);
				HandleResult(stat4, XI_PRM_AUTO_WB);
			}

			// Set the focus motor position to a large negative number to ensure that focus motor is in lowest position to start
			xiSetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVEMENT_VALUE, INT_MIN);
			xiSetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVE, 0);

			// Sleep to allow motor movement
			std::this_thread::sleep_for(std::chrono::seconds(3));

			std::cout << "How many training samples to collect? " << std::endl;

			int sampleSize;
			std::cin >> sampleSize;

			std::cout << "What number to start at?" << std::endl;

			int offset = 0;
			std::cin >> offset;

			int samplesCollected = 0;

			while (samplesCollected < sampleSize) {
				std::cout << "On sample: " << samplesCollected << std::endl;
				XI_IMG image;
				memset(&image, 0, sizeof(image));
				image.size = sizeof(XI_IMG);


				// Create a new folder for this datapoint
				std::string newFoldername = (std::string)FOLDERNAME + "/sample" + std::to_string(samplesCollected + offset);
				_mkdir(newFoldername.c_str());

				std::fstream saveFile;
				std::string saveTo = newFoldername + "/focusInfo.txt";
				saveFile.open(saveTo, std::ios::out);

				std::cout << "Was able to open " << saveTo << std::endl;

				// Randomly move to a focus position in the valid focus position range
				// Get random value in [0, 1) and shift it so that get random value in range [LOWERFOCUS, UPPERFOCUS)
				int desiredPos = (int)( ((double)rand() / (RAND_MAX)) * (UPPERFOCUS - LOWERFOCUS) + LOWERFOCUS);

				int ret = moveToFocusPos(deviceHandle, focusMotorPosition, desiredPos);

				std::this_thread::sleep_for(std::chrono::seconds(5));

				// Otherwise failed to execute move so skip this round and don't update number of samples collected
				if (ret == 0) {
					saveFile << "before focus: " << focusMotorPosition << std::endl;

					for (int i = 0; i < 2; i++) {
						// Need to trigger taking of image, and do so each time want to take an image
						if (XI_OK == xiSetParamInt(deviceHandle, XI_PRM_TRG_SOFTWARE, 1))
						{
							if (XI_OK == xiGetImage(deviceHandle, 1000, &image))
							{
								cv::Mat procImage = cvtMatImage(image);

								std::string prefix;
								if (i == 0) {
									prefix = "before";
								}
								else {
									prefix = "after";
								}

								// Display the image, save the image, compute the sharpness of image

								cv::imshow(prefix + " focus image", resizeImage(procImage, 0.15));
								cv::waitKey(10);

								std::string filename = newFoldername + "/" + prefix + std::to_string(samplesCollected + offset) + ".tif";
								cv::imwrite(filename, procImage);
								std::cout << "Saving image " + prefix + ": " << filename << std::endl;
								double sharpness = context.autoFocusing().getFocusValue(procImage);

								saveFile << prefix + " sharpness: " << sharpness << std::endl;
							}
							else {
								std::cout << "Failed to capture image " << std::endl;
							}

							if (i == 0) {
								// Temporarily stop acquisition and unlock so that can do AF stuff
								xiStopAcquisition(deviceHandle);
								context.autoFocusing().unlockDeviceMutex();

								std::cout << "Trying to AF" << std::endl;
								try
								{
									// Calling perform() with no args will use the command line args above (serialnumber, searchtype=1, rangestart, rangeend, partition)
									// Alternatively these can be provided directly using this signature
									// context.autoFocusing().perform(const char* serialNumber, int searchType, int rangeStart, int rangeEnd, int partition)

									std::fstream file;
									file.open(newFoldername + "/AF.txt", std::ios::out);

									// Backup streambuffers of cout, cin
									std::streambuf* stream_buffer_cout = std::cout.rdbuf();
									std::streambuf* stream_buffer_cin = std::cin.rdbuf();

									// Get the streambuffer of the file 
									std::streambuf* stream_buffer_file = file.rdbuf();

									// Redirect cout to file
									std::cout.rdbuf(stream_buffer_file);

									double bestFocusValue = context.autoFocusing().perform(serialNumber, 1, LOWERFOCUS, UPPERFOCUS, 6);

									// Sleep so don't cut anything off
									std::this_thread::sleep_for(std::chrono::seconds(1));

									file.close();

									// Redirect cout back to screen 
									std::cout.rdbuf(stream_buffer_cout);

									// Read the file that was just written to extract the focus motor position just set
									focusMotorPosition = searchForFocus(newFoldername + "/AF.txt");

									if (bestFocusValue <= 0.0)
									{
										std::cout << "Autofocus full range search " << " failed" << std::endl;
									}
									else
									{
										std::cout << "Autofocus full range search complete, best focus value is "
											<< bestFocusValue << std::endl;
									}
								}
								catch (const std::exception& e)
								{
									std::cout << "Autofocus full range search exception: " << e.what() << std::endl;
								}

								// Start back up after finish AF
								xiStartAcquisition(deviceHandle);
								context.autoFocusing().lockDeviceMutex();

								// Need to redo parameters once restart acquisition
								XI_RETURN stat0 = xiSetParamInt(deviceHandle, XI_PRM_LENS_MODE, XI_ON);
								HandleResult(stat0, XI_PRM_LENS_MODE);
								XI_RETURN stat1 = xiSetParamFloat(deviceHandle, XI_PRM_LENS_APERTURE_VALUE, APERTURE_LEVEL);
								HandleResult(stat1, XI_PRM_LENS_APERTURE_VALUE);
								XI_RETURN stat2 = xiSetParamInt(deviceHandle, XI_PRM_EXPOSURE, EXPOSURE_TIME);
								HandleResult(stat2, XI_PRM_EXPOSURE);
								XI_RETURN stat3 = xiSetParamFloat(deviceHandle, XI_PRM_GAIN, GAIN);
								HandleResult(stat3, XI_PRM_GAIN);
								if (WHITE_BALANCE) {
									XI_RETURN stat4 = xiSetParamInt(deviceHandle, XI_PRM_AUTO_WB, WHITE_BALANCE);
									HandleResult(stat4, XI_PRM_AUTO_WB);
								}

								saveFile << "after focus: " << focusMotorPosition << std::endl;
							}
						}
					}
					samplesCollected++;
				}
				else {
					std::cout << "Failed to execute random move, trying again " << std::endl;
				}
			}

			xiStopAcquisition(deviceHandle);
			context.autoFocusing().unlockDeviceMutex();
		}
	}
	else
	{
		std::cout << "Failed to get device handle" << std::endl;
	}

	// Generate output file with camera parameters used for collection

	std::fstream cameraFile;
	std::string saveLoc = (std::string)FOLDERNAME + "/CameraParameter.txt";
	cameraFile.open(saveLoc, std::ios::out);

	std::cout << "Was able to open " << saveLoc << std::endl;

	// current date/time based on current system
	time_t now = time(0);

	// convert now to string form
	char* dt = ctime(&now);

	cameraFile << "The local date and time is: " << dt << std::endl;
	cameraFile << "Aperture Level: " << APERTURE_LEVEL << '\n';
	cameraFile << "Exposure time: " << EXPOSURE_TIME << '\n';
	cameraFile << "Gain: " << GAIN << '\n';
	cameraFile << "Automatic white balance: " << WHITE_BALANCE << '\n';
	float lensLength;
	XI_RETURN check = xiGetParamFloat(deviceHandle, XI_PRM_LENS_FOCAL_LENGTH, &lensLength);
	HandleResult(check, XI_PRM_LENS_FOCAL_LENGTH);
	cameraFile <<  "Lens Focal Length (zoom): " << lensLength << std::endl;
	cameraFile.close();

	std::cout << "Was able to work with and close file " << std::endl;

	// Cannot seem to get code to exit gracefully after running AF code, if don't call the AF code will exit gracefully

	return 0;
}
