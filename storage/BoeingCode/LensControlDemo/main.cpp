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

    // Demonstration of getting the focus value for and existing image
    cv::Mat matOut = cv::imread("..\\appdata\\FocusingSample.png", CV_LOAD_IMAGE_UNCHANGED);
    if (matOut.empty())
    {
        std::cout << "Invalid image for calculating focusing value." << std::endl;
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
            if (XI_OK == xiSetParamInt(deviceHandle, XI_PRM_TRG_SOFTWARE, 1))
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                XI_IMG image;
                memset(&image, 0, sizeof(image));
                image.size = sizeof(XI_IMG);
                if (XI_OK == xiGetImage(deviceHandle, 1000, &image))
                {
                    std::cout << "Captrue an image with width :" << image.width
                        << " height: " << image.height << std::endl;
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
