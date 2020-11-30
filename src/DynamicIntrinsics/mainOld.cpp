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
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


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
	
	/*::string filename = "C:\\Users\\aofeldman\\Desktop\\testImage.jpg";
	std::cout << filename << std::endl;
	cv::Mat testImage = cv::imread(filename);
	if (testImage.empty()) {
		std::cout << "Why is it failing though? " << filename << std::endl;
		std::cout << "BAD" << std::endl;
		std::cout << filename.length() << std::endl;
	}*/

	// Demonstration of getting the focus value for and existing image
    cv::Mat matOut = cv::imread("C:/Users/aofeldman/Desktop/ClosedLoopMetrology/Code/src/BoeingCode/LensControlDemo/Sample/FocusingSample.png", CV_LOAD_IMAGE_UNCHANGED);
    if (matOut.empty())
    {
        std::cout << "Aaron Testing: Invalid image for calculating focusing value." << std::endl;
    }
    else
    {
        double focusValue = context.autoFocusing().getFocusValue(matOut);
        std::cout << "Focusing value for image is " << focusValue << std::endl;
    }

    // Demonstration of retrieving the the device handle from GenICamSDKContext to make direct Ximea API calls in client code
    HANDLE deviceHandle;
    if (context.autoFocusing().getDeviceHandler(deviceHandle))
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

			// Print out version of the API
			char version[200] = "";
			xiGetParamString(deviceHandle, XI_PRM_API_VERSION, &version, sizeof(version));
			std::cout << "Version of API: " << version << std::endl;

			// Turn on lens mode to enable shifting of aperture and playing with focus distance
			xiSetParamInt(deviceHandle, XI_PRM_LENS_MODE, XI_ON);
			int value = 0;
			if (XI_OK != xiGetParamInt(deviceHandle, XI_PRM_LENS_MODE, &value)) {
				std::cout << "Failed to get value for lens mode " << std::endl;
			}
			else {
				std::cout << "Lens mode is on? " << value << std::endl;
			}

			xiSetParamInt(deviceHandle, XI_PRM_LENS_FEATURE_SELECTOR, XI_LENS_FEATURE_MOTORIZED_FOCUS_SWITCH);
			/*int errorCode = xiSetParamInt(deviceHandle, XI_PRM_LENS_FEATURE, 1);
			if (XI_OK != errorCode) {
				std::cout << "Unable to flip switch on: " << errorCode << std::endl;
			} */

			int switch_status = 0;
			if (XI_OK == xiGetParamInt(deviceHandle, XI_PRM_LENS_FEATURE, &switch_status)) {
				if (switch_status > 0) {
					std::cout << "The motorized focus switch on the lens is switched on" << std::endl;
				}
				else {
					std::cout << "The motorized focus switch on the lens is switched off" << std::endl;
				}
			}
			else {
				std::cout << "The check of motorized focus switch itself is not working" << std::endl;
			}

			// TODO: Fix this portion up, should roughly match above
			/* xiSetParamInt(deviceHandle, XI_PRM_LENS_FEATURE_SELECTOR, XI_LENS_FEATURE_MOTORIZED_FOCUS_SWITCH);
			int zoom_status = 0;
			xiGetParamInt(deviceHandle, XI_PRM_LENS_FEATURE, &zoom_status);
			if (zoom_status > 0) {
				std::cout << "The zoom is supported " << std::endl;
			}
			else {
				int errorCode = xiSetParamInt(deviceHandle, XI_PRM_LENS_FEATURE, 1);
				if (XI_OK != errorCode) {
					std::cout << "Tried to change zoom support but failed " << errorCode << std::endl;
				}
				else {
					int zoom_status = 0;
					xiGetParamInt(deviceHandle, XI_PRM_LENS_FEATURE, &zoom_status);
					if (zoom_status > 0) {
						std::cout << "Successfully changed zoom support " << std::endl;
					}
				}
			} */

			// Try setting aperture to different levels and observe result
			//float apertureLevels[5] = {2.8, 4, 5.6, 8, 11 };
			float apertureLevels[1] = { 2.8 };
			float aperture = 0.0;
			int exposureTimes[3] = { 10000, 50000, 100000 }; // microseconds
			int exposure = 0;

			//for (int i = 0; i < 5; i++)
			//for (int i = 0; i < 3; i++)

			xiSetParamInt(deviceHandle, XI_PRM_EXPOSURE, 50000);

			// Try white balancing
			if (XI_OK != xiSetParamInt(deviceHandle, XI_PRM_AUTO_WB, XI_ON)) {
				std::cout << "Unable to turn on auto white balance " << std::endl;
			}
			else {
				std::cout << "Successfully turned on auto white balance " << std::endl;
			}

			// Let's try changing the focus distance
			int temp = 0;
			if (XI_OK != xiGetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVEMENT_VALUE, &temp)) {
				std::cout << "Unable to retrieve focus movement value " << std::endl;
			}
			else {
				std::cout << "Focus movement value: " << temp << std::endl;
			}
			
			int moveVal = 0;
			if (XI_OK != xiSetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVEMENT_VALUE, moveVal)) {
				std::cout << "Failed to set movement value to " + std::to_string(moveVal) << std::endl;
			}
			else {
				std::cout << "Set movement value to " + std::to_string(moveVal) << std::endl;
			}

			if (XI_OK != xiSetParamInt(deviceHandle, XI_PRM_LENS_FOCUS_MOVE, 0)) {
				std::cout << "Failed to move focus distance" << std::endl;
			}
			else {
				std::cout << "Successfully moved focus distance" << std::endl;
			}

			// Verify that lens mode is still on
			int stillOn = 0;
			xiGetParamInt(deviceHandle, XI_PRM_LENS_MODE, &stillOn);
			if (stillOn > 0) {
				std::cout << "Lens mode still on " << std::endl;
			}
			else {
				std::cout << "Lens mode switched off " << std::endl;
			}

			// Have tried both int distance_cm and float distance_cm (and ParamInt or ParamFloat respectively). Neither seems to work.
			float distance_cm = 0.0;
			//int errorCode = xiGetParamFloat(deviceHandle, XI_PRM_LENS_FOCUS_DISTANCE, &distance_cm);
			DWORD temp1;
			temp1 = sizeof(DWORD);
			XI_PRM_TYPE temp2;
			temp2 = xiTypeFloat;

			int errorCode = xiGetParam(deviceHandle, XI_PRM_LENS_FOCUS_DISTANCE, &distance_cm, &temp1, &temp2);
			if (XI_OK != errorCode) {
				std::cout << "Failed to find focus distance. " << "Error Code: " << errorCode << std::endl;
			}
			else {
				std::cout << "THE FOCUS DISTANCE IS: " << distance_cm << std::endl;
			}

			for (int i = 0; i < 1; i++)
			{
				if (XI_OK != xiSetParamFloat(deviceHandle, XI_PRM_LENS_APERTURE_VALUE, apertureLevels[i]))
				{
					std::cout << "Failed to update aperture" << std::endl;
				}

				if (XI_OK != xiGetParamFloat(deviceHandle, XI_PRM_LENS_APERTURE_VALUE, &aperture)) {
					std::cout << "Failed to find aperture level" << std::endl;
				}
				else
				{
					std::cout << "Current aperture level " << aperture << std::endl;
				}


				/*if (XI_OK != xiSetParamFloat(deviceHandle, XI_PRM_EXPOSURE, exposureTimes[i]))
				{
					std::cout << "Failed to update exposure time" << std::endl;
				}

				if (XI_OK != xiGetParamInt(deviceHandle, XI_PRM_EXPOSURE, &exposure)) {
					std::cout << "Failed to find exposure time" << std::endl;
				}
				else
				{
					std::cout << "Current exposure time " << exposure << std::endl;
				}*/

				if (XI_OK == xiSetParamInt(deviceHandle, XI_PRM_TRG_SOFTWARE, 1))
				{
					std::this_thread::sleep_for(std::chrono::seconds(1));
					
					XI_IMG image;
					memset(&image, 0, sizeof(image));
					image.size = sizeof(XI_IMG);
					if (XI_OK == xiGetImage(deviceHandle, 1000, &image))
					{
						std::cout << "Capture an image with width :" << image.width
							<< " height: " << image.height << std::endl;
						cv::Mat img_raw = cvtMatImage(image);
						
						//cv::imshow("Saved image " + std::to_string(exposure), resizeImage(img_raw, 0.15));
						cv::imshow("Saved image " + std::to_string(aperture), resizeImage(img_raw, 0.15));
						
						/*std::cout << "Shape (row, col, channels) of img_raw " << img_raw.rows << ' ' << img_raw.cols << ' ' << img_raw.channels() << std::endl;
						std::cout << "Type of img_raw " << type2str(img_raw.type()) << std::endl; */
						cv::waitKey(0);
						
						//cv::imwrite("C:/Users/aofeldman/Desktop/labImage" + std::to_string(exposure) + ".tif", img_raw);
						cv::imwrite("C:/Users/aofeldman/Desktop/labImageAF" + std::to_string(aperture) + ".tif", img_raw);
						
						/*cv::waitKey(3000);
						cv::Mat checkImage = cv::imread("C:/Users/aofeldman/Desktop/labImage.tif", cv::IMREAD_UNCHANGED);
						std::cout << "Shape (row, col, channels) of checkImage " << checkImage.rows << ' ' << checkImage.cols << ' ' << checkImage.channels() << std::endl;
						cv::imshow("Checking image", resizeImage(checkImage, 0.15));
						cv::waitKey(0);*/

						float zoom_distance = 15.0;
						if (XI_OK == xiGetParamFloat(deviceHandle, XI_PRM_LENS_FOCUS_DISTANCE, &zoom_distance)) {
							std::cout << "The zoom distance is: " << zoom_distance << std::endl;
						}
						else {
							std::cout << "Unable to retrieve zoom distance " << std::endl;
						}
					}
					else {
						std::cout << "Failed to receive image trigger " << std::endl;
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
    }

    return 0;
}
