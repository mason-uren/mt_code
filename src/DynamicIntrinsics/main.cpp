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

#define HandleResult(res,place) if (res!=XI_OK) {printf("Error after %s (%d)",place,res);}


// User-defined constants
#define FOLDERNAME "C:/Users/aofeldman/Desktop/testCollection"
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
	// std::cout << "Image data format " << image.frm << std::endl;
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

	// Initialize timers for the three key strokes
	std::clock_t qCheckpoint = std::clock();
	std::clock_t cCheckpoint = std::clock();
	std::clock_t fCheckpoint = std::clock();
	std::clock_t gCheckpoint = std::clock();
	std::clock_t pCheckpoint = std::clock();
	std::clock_t mCheckpoint = std::clock();

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

	/*cv::Mat matOut = cv::imread("C:\\Users\\aofeldman\\Desktop\\WSLIME_SDK_vs2017\\appdata\\FocusingSample.png", CV_LOAD_IMAGE_UNCHANGED);
	if (matOut.empty())
	{
		std::cout << "Invalid image for calculating focusing value." << std::endl;
	}
	else
	{
		double focusValue = context.autoFocusing().getFocusValue(matOut);
		std::cout << "Testing out sample image: " << std::endl;
		std::cout << "Focusing value for image is " << focusValue << std::endl;
		std::cout << "Type of sample image: " << type2str(matOut.type()) << std::endl;
		std::cout << "Testing image shape: (rows, cols) " << matOut.rows << ' ' << matOut.cols << std::endl;
	}

	// Try some smaller fiducial images

	std::vector<double> fiducialFocusValues;
	for (int i = 0; i < 5; i++)
	{
		// Read in stored ximea images to see how accurate the focus prediction is
		std::string imageName = "C:\\Users\\aofeldman\\Desktop\\fiducialImages\\ximea" + std::to_string(i) + ".jpg";
		//cv::Mat matOut = cv::imread(imageName, CV_LOAD_IMAGE_UNCHANGED);
		cv::Mat matOut = cv::imread(imageName, CV_LOAD_IMAGE_GRAYSCALE); // Load as grayscale or not?
		if (matOut.empty())
		{
			std::cout << "Invalid image for calculating focusing value." << std::endl;
		}
		else
		{
			std::cout << "Size of Fiducial image: (rows, cols) " << matOut.rows << ' ' << matOut.cols << std::endl;
			double focusValue = context.autoFocusing().getFocusValue(matOut);
			std::cout << "Focusing value for fiducial image " + std::to_string(i) + " is " << focusValue << std::endl;
			std::cout << "Type of fiducial image: " << type2str(matOut.type()) << std::endl;
			fiducialFocusValues.push_back(focusValue);
			//cv::imshow("Display window", matOut);
			//cv::waitKey(0);
		}
	}

	std::cout << "Mean of fiducial focus values: " << mean(fiducialFocusValues) << std::endl;
	std::cout << "Standard deviation of fiducal focus values: " << standDev(fiducialFocusValues) << std::endl;

	std::vector<double> calibrationFocusValues;
	for (int i = 0; i < 34; i++)
	{
		// Read in stored ximea images to see how accurate the focus prediction is
		std::string imageName = "C:\\Users\\aofeldman\\Desktop\\ximeaData\\ximea" + std::to_string(i) + ".tif";
		//cv::Mat matOut = cv::imread(imageName, CV_LOAD_IMAGE_UNCHANGED);
		cv::Mat matOut = cv::imread(imageName, CV_LOAD_IMAGE_GRAYSCALE);
		if (matOut.empty())
		{
			std::cout << "Invalid image for calculating focusing value." << std::endl;
		}
		else
		{
			std::cout << "Size of Ximea image: (rows, cols) " << matOut.rows << ' ' << matOut.cols << std::endl;
			double focusValue = context.autoFocusing().getFocusValue(matOut);
			std::cout << "Focusing value for Ximea image " + std::to_string(i) + " is " << focusValue << std::endl;
			std::cout << "Type of Ximea image: " << type2str(matOut.type()) << std::endl;
			calibrationFocusValues.push_back(focusValue);
			//cv::imshow("Display window", matOut);
			//cv::waitKey(0);
		}
	}

	std::cout << "Mean of calibration focus values: " << mean(calibrationFocusValues) << std::endl;
	std::cout << "Standard deviation of calibration focus values: " << standDev(calibrationFocusValues) << std::endl;
	*/
	int numImagesSaved = 0; // Keep track of number images saved, reset for each AF setting
	int numTimesAF = 0;
	XI_RETURN state = XI_OK;

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
			// XI_RETURN stat1 = xiSetParamFloat(deviceHandle, XI_PRM_LENS_APERTURE_VALUE, APERTURE_LEVEL);
			// HandleResult(stat1, XI_PRM_LENS_APERTURE_VALUE);

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

			// Aaron testing what value is given when use auto gain
			/* XI_RETURN stat = xiSetParamInt(deviceHandle, XI_PRM_AEAG, XI_ON);
			HandleResult(stat, XI_PRM_AEAG); 
			int gain;
			XI_RETURN ret = xiGetParamInt(deviceHandle, XI_PRM_GAIN, &gain);
			HandleResult(ret, XI_PRM_GAIN);
			std::cout << "Gain achieved was " << gain << std::endl; */

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

			XI_RETURN retval = xiSetParamInt(deviceHandle, XI_PRM_LENS_FEATURE_SELECTOR, XI_LENS_FEATURE_IMAGE_STABILIZATION_SWITCH_STATUS);
			HandleResult(retval, XI_PRM_LENS_FEATURE_SELECTOR);
			int switch_status = -1;
			XI_RETURN retval2 = xiGetParamInt(deviceHandle, XI_PRM_LENS_FEATURE, &switch_status);
			HandleResult(retval2, XI_PRM_LENS_FEATURE);
			if (switch_status > 0) {
				std::cout << "Image Stabilization is On" << std::endl;
			}
			else if (switch_status == 0) {
				std::cout << "Image Stabilization is Off" << std::endl;
			}
			else if (switch_status == -1) {
				std::cout << "Failed to Read Image Stabilization Status" << std::endl;
			}

			// 12-bit depth not supported
			/* XI_RETURN ret1 = xiSetParamInt(deviceHandle, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO16);
			HandleResult(ret1, XI_PRM_IMAGE_DATA_FORMAT);
			int bitDepth = 0;
			XI_RETURN ret2 = xiGetParamInt(deviceHandle, XI_PRM_IMAGE_DATA_BIT_DEPTH, &bitDepth);
			HandleResult(ret2, XI_PRM_IMAGE_DATA_BIT_DEPTH);
			std::cout << "Bit depth used: " << bitDepth << std::endl; */

			// Set the focus motor position to a large negative number to ensure that focus motor is in lowest position to start
			xiSetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVEMENT_VALUE, INT_MIN);
			xiSetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVE, 0);

			// Sleep to allow motor movement
			std::this_thread::sleep_for(std::chrono::seconds(3));

			std::string newFoldername;

			while (true) {
				if (XI_OK == xiSetParamInt(deviceHandle, XI_PRM_TRG_SOFTWARE, 1))
				{
					//std::this_thread::sleep_for(std::chrono::seconds(1));
					XI_IMG image;
					memset(&image, 0, sizeof(image));
					image.size = sizeof(XI_IMG);
					if (XI_OK == xiGetImage(deviceHandle, 1000, &image))
					{
						/*std::cout << "Capture an image with width :" << image.width
							<< " height: " << image.height << std::endl; */
							// Convert to opencv image type
						cv::Mat procImage = cvtMatImage(image);

						cv::imshow("Currently captured image", resizeImage(procImage, 0.15));
						cv::waitKey(10);
						//std::cout << "c duration " << ((double)(std::clock()) - (double)(cCheckpoint)) / (double)CLOCKS_PER_SEC << std::endl;
						// 67 is ASCII for 'c'
						if (keyCheck(cCheckpoint, 0.5, 67)) {
							if (numTimesAF != 0) {
								// Save the opencv image
								numImagesSaved++;
								//std::string filename = (std::string)FOLDERNAME + "/ximea_AFround" + std::to_string(numTimesAF) + '_' + std::to_string(numImagesSaved) + ".tif";
								std::string filename = newFoldername + "/ximea" + std::to_string(numImagesSaved) + ".tif";
								cv::imwrite(filename, procImage);
								std::cout << "Saving image: " << filename << std::endl;
							}
							else {
								std::cout << "Please call AF code (by pressing 'f') first" << std::endl;
							}
						}

						//std::cout << "Current focus motor position: " << focusMotorPosition << std::endl;

						// Press 'g' (ASCII 71) to get the focus value of the current image
						if (keyCheck(gCheckpoint, 0.5, 71)) {
							double focusValue = context.autoFocusing().getFocusValue(procImage);
							std::cout << "Current focus value (sharpness): " << focusValue << std::endl;
						}

					}
				}
				// Press 'q' (ASCII 81) to quit, breaks out of this loop
				if (keyCheck(qCheckpoint, 0.5, 81)) {
					cv::destroyAllWindows();
					std::cout << "Destroyed OpenCV windows" << std::endl;
					break;
				}
				
				// Press 'p' (ASCII 80) to get the current focus motor position
				if (keyCheck(pCheckpoint, 0.5, 80)) {
					std::cout << "Current focus motor position: " << focusMotorPosition << std::endl;
				}

				// Press 'm' (ASCII 77) to trigger focus motor movement
				if (keyCheck(mCheckpoint, 2, 77)) {
					std::cout << "Please type in the desired position to move to " << std::endl;
					int desiredPos;
					std::cin >> desiredPos;
					moveToFocusPos(deviceHandle, focusMotorPosition, desiredPos);
				}

				// Press 'f' to run full search autofocus and 'n' to run a full search but limit the bounds to nearby
				// 70 is ASCII for 'f' 78 is ASCII for 'n'

				// Use same checkpoint
				bool pressedF = keyCheck(fCheckpoint, 3, 70);
				bool pressedN = keyCheck(fCheckpoint, 3, 78);

				if (pressedF || pressedN) {
					if (pressedF) {
						std::cout << "Trying to full AF " << std::endl;
					}
					else if (pressedN) {
						std::cout << "Trying to neighbor AF " << std::endl;
					}

					// Temporarily stop acquisition and unlock so that can do AF stuff
					xiStopAcquisition(deviceHandle);
					context.autoFocusing().unlockDeviceMutex();

					try
					{
						// Calling perform() with no args will use the command line args above (serialnumber, searchtype=1, rangestart, rangeend, partition)
						// Alternatively these can be provided directly using this signature
						// context.autoFocusing().perform(const char* serialNumber, int searchType, int rangeStart, int rangeEnd, int partition)

						numTimesAF++;

						newFoldername = (std::string)FOLDERNAME + "/AFround" + std::to_string(numTimesAF);
						_mkdir(newFoldername.c_str());

						std::fstream file;
						//file.open((std::string)FOLDERNAME + "/AF" + std::to_string(numTimesAF) + ".txt", std::ios::out);

						file.open(newFoldername + "/AF" + std::to_string(numTimesAF) + ".txt", std::ios::out);

						// Backup streambuffers of cout, cin
						std::streambuf* stream_buffer_cout = std::cout.rdbuf();
						std::streambuf* stream_buffer_cin = std::cin.rdbuf();

						// Get the streambuffer of the file 
						std::streambuf* stream_buffer_file = file.rdbuf();

						// Redirect cout to file
						std::cout.rdbuf(stream_buffer_file);

						double bestFocusValue;
						// Noticed that sometimes more thorough search is needed to get good results, does not seem to slow down
						// too much either. So, perhaps consider setting partition size to 8.
						//if (((std::string)"BJCAB1841028").compare(std::string(serialNumber)) == 0) {
						//	std::cout << "Doing the more thorough search" << std::endl;
						//	bestFocusValue = context.autoFocusing().perform(serialNumber, 1, 0, 2000, 8);
						//}
						//else {
						//	bestFocusValue = context.autoFocusing().perform();
						//}
						if (pressedF) {
							bestFocusValue = context.autoFocusing().perform(serialNumber, 1, 0, 1100, 6);
						}
						// TODO: Change this so does a neighbor search instead
						else if (pressedN) {
							bestFocusValue = context.autoFocusing().perform(serialNumber, 1, focusMotorPosition - 100, focusMotorPosition + 100, 6);
						}

						// Sleep so don't cut anything off
						std::this_thread::sleep_for(std::chrono::seconds(1));

						file.close();

						// Redirect cout back to screen 
						std::cout.rdbuf(stream_buffer_cout);

						// Read the file that was just written to extract the focus motor position just set
						focusMotorPosition = searchForFocus(newFoldername + "/AF" + std::to_string(numTimesAF) + ".txt");

						// Reset image number count
						numImagesSaved = 0;

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

	std::cout << "Reached here " << std::endl;

	// Generate output file with camera parameters used for collection


	/*
	// Demonstration of calling the focus method for a full range search
	std::cout << "Autofocus full range search " << std::endl;
	try
	{
		// Calling perform() with no args will use the command line args above (serialnumber, searchtype=1, rangestart, rangeend, partition)
		// Alternatively these can be provided directly using this signature
		// context.autoFocusing().perform(const char* serialNumber, int searchType, int rangeStart, int rangeEnd, int partition)
		double bestFocusValue = context.autoFocusing().perform();
		if (bestFocusValue <= 0.0)
		{
			std::cout << "Autofocus full range search " << " failed" << std::endl;
		}
		else
		{
			std::cout << "Autofocus full range search complete, best focus value is "
				<< bestFocusValue << " sleeping for 3 seconds" << std::endl;
		}
	}
	catch (const std::exception& e)
	{
		std::cout << "Autofocus full range search exception: " << e.what() << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::seconds(3));
	// Demonstration of calling the focus method  for a neighbouring range search multiple times
	//with a 3 seconds delay b/w each

	for (auto i = 0; i < 3; ++i)
	{
		std::cout << "Autofocus neighbouring search round " << i + 1 << "/3" << std::endl;
		try
		{
			//false means using neighbouring search
			double bestFocusValue = context.autoFocusing().perform(false);
			if (bestFocusValue <= 0.0)
			{
				std::cout << "Autofocus neighbouring search round " << i + 1 << " failed" << std::endl;
			}
			else
			{
				std::cout << "Autofocus neighbouring search round " << i + 1 << " complete, best focus value is "
					<< bestFocusValue << " sleeping for 3 seconds" << std::endl;
			}
		}
		catch (const std::exception& e)
		{
			std::cout << "Autofocus neighbouring search exception: " << e.what() << std::endl;
		}

		std::this_thread::sleep_for(std::chrono::seconds(3));
	} */
	std::fstream saveFile;
	std::string saveTo = (std::string)FOLDERNAME + "/CameraParameter.txt";
	saveFile.open(saveTo, std::ios::out);

	std::cout << "Was able to open " << saveTo << std::endl;

	// current date/time based on current system
	time_t now = time(0);

	// convert now to string form
	char* dt = ctime(&now);

	saveFile << "The local date and time is: " << dt << std::endl;
	saveFile << "Aperture Level: " << APERTURE_LEVEL << '\n';
	saveFile << "Exposure time: " << EXPOSURE_TIME << '\n';
	saveFile << "Gain: " << GAIN << '\n';
	saveFile << "Automatic white balance: " << WHITE_BALANCE << '\n';
	float lensLength;
	XI_RETURN check = xiGetParamFloat(deviceHandle, XI_PRM_LENS_FOCAL_LENGTH, &lensLength);
	HandleResult(check, XI_PRM_LENS_FOCAL_LENGTH);
	saveFile <<  "Lens Focal Length (zoom): " << lensLength << std::endl;
	saveFile.close();

	std::cout << "Was able to work with and close file " << std::endl;

	// Cannot seem to get code to exit gracefully after running AF code, if don't call the AF code will exit gracefully

	return 0;
}
