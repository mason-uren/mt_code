#include <vector>
#include <sstream>
#include <mutex>
#include <cstring>
#include <chrono>
#include "../GenICamInterface/GenICamAdapterInterface.h"
#include "XimeaAdapter.h"
#include "xiApi.h"

using namespace std;

XimeaAdapter::XimeaAdapter() : deviceHandle(nullptr)
{
    adapterName = "Ximea";
	initialized = false;
}

XimeaAdapter::~XimeaAdapter()
{
    if (isAcquiring())
    {
        endAcquire();
    }
    releaseResources();
}

void XimeaAdapter::releaseResources()
{
	xiCloseDevice(deviceHandle);
	deviceHandle = nullptr;
}

bool XimeaAdapter::initializeDevice(string deviceSerialNumber)
{
    //capture serial number
    serialNumber = deviceSerialNumber;
	//turn on new API features
	xiSetParamInt(0, XI_PRM_NEW_PROCESS_CHAIN_ENABLE, XI_ON);
	// Retrieving a handle to the camera device 
	XI_RETURN retVal = xiOpenDeviceBy(XI_OPEN_BY_SN, deviceSerialNumber.c_str(), &deviceHandle);
	initialized = (retVal == XI_OK);
	if (!isInitialized())
	{
		stringstream errStream;
		errStream << "Initialize failed! Error code: " << retVal;
		logMessage(GenICamAdapterInterface::ERROR_MSG, errStream.str());
	}
    return isInitialized();
}

void XimeaAdapter::setIntegerParamVal(string name, int64_t val)
{
    stringstream logStream;
    logStream << "Setting " << name << " to value : " << val;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//set the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiSetParamInt(deviceHandle, name.c_str(), static_cast<int>(val));
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Set failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}    
}

void XimeaAdapter::getIntegerParamVal(string name, int64_t &val)
{
    stringstream logStream;
    logStream << "Getting " << name;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//get the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiGetParamInt(deviceHandle, name.c_str(), reinterpret_cast<int*>(&val));
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Get failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::setFloatParamVal(string name, double val)
{
    stringstream logStream;
    logStream << "Setting " << name << " to value : " << val;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//set the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiSetParamFloat(deviceHandle, name.c_str(), static_cast<float>(val));
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Set failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::getFloatParamVal(string name, double &val)
{
    stringstream logStream;
    logStream << "Getting " << name;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//get the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiGetParamFloat(deviceHandle, name.c_str(), reinterpret_cast<float*>(&val));
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Get failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::setBoolParamVal(string name, bool val)
{
    stringstream logStream;
    logStream << "Setting " << name << " to value : " << val;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//set the value
	lock_guard<mutex> guard(deviceMutex);
	//translate to integer value
	int32_t intVal = 0;
	if (val) intVal = 1;
	XI_RETURN retVal = xiSetParamInt(deviceHandle, name.c_str(), val);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Set failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}	
}

void XimeaAdapter::getBoolParamVal(string name, bool &val)
{
    stringstream logStream;
    logStream << "Getting " << name;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//get the value
	lock_guard<mutex> guard(deviceMutex);
	//translate to integer value
	int32_t intVal = 0;
	XI_RETURN retVal = xiGetParamInt(deviceHandle, name.c_str(), &intVal);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Get failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
	else
	{
		//populate return value
		val = (intVal == 1);
	}
}

void XimeaAdapter::setEnumParamVal(string name, string val)
{
    stringstream logStream;
    logStream << "Setting " << name << " to value : " << val;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//set the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiSetParam(deviceHandle, name.c_str(), (void*)val.c_str(), (DWORD)val.length(), xiTypeEnum);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Set failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::setEnumParamVal(string name, int64_t val)
{
    stringstream logStream;
    logStream << "Setting " << name << " to value : " << val;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//set the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiSetParam(deviceHandle, name.c_str(), (void*)&val, (DWORD)sizeof(val), xiTypeEnum);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Set failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::getEnumParamVal(string name, string &val)
{
    stringstream logStream;
    logStream << "Getting " << name;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	DWORD valSize = 0;
	XI_PRM_TYPE valType;
	//get the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiGetParam(deviceHandle, name.c_str(), (void*)&val, &valSize, &valType);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Get failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
	if (valType != xiTypeEnum)
	{
		stringstream errStream;
		errStream << "Invalid type returned, returned type: " << valType;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::getEnumParamVal(string name, int64_t &val)
{
    stringstream logStream;
    logStream << "Getting " << name;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	DWORD valSize = 0;
	XI_PRM_TYPE valType;
	//get the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiGetParam(deviceHandle, name.c_str(), (void*)&val, &valSize, &valType);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Get failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
	if (valType != xiTypeEnum)
	{
		stringstream errStream;
		errStream << "Invalid type returned, returned type: " << valType;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::setStringParamVal(string name, string val)
{
    stringstream logStream;
    logStream << "Setting " << name << " to value : " << val;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	//set the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiSetParamString(deviceHandle, name.c_str(), (void*)val.c_str(), (DWORD)val.length() + 1);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Set failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::getStringParamVal(string name, string &val)
{
    stringstream logStream;
    logStream << "Getting " << name;
    logMessage(GenICamAdapterInterface::INFO_MSG, logStream.str());
	char buffer[50];
	//set the value
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiGetParamString(deviceHandle, name.c_str(), (void*)buffer, (DWORD)val.length() + 1);
	val = string(buffer, val.length() + 1);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Get failed, returned code: " << retVal;
		logMessage(GenICamAdapterInterface::WARNING_MSG, errStream.str());
	}
}

void XimeaAdapter::executeCommand(string name)
{    
    logMessage(GenICamAdapterInterface::ERROR_MSG, "Ximea doesn't implement a dedicated command interface!");    
}

void XimeaAdapter::beginAcquire()
{
	lock_guard<mutex> guard(deviceMutex);
    logMessage(GenICamAdapterInterface::INFO_MSG, "Begin acquisition...");
	XI_RETURN retVal = xiStartAcquisition(deviceHandle);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Begin acquisition failed! Error Code: " << retVal;
		logMessage(GenICamAdapterInterface::ERROR_MSG, errStream.str());
	}
}

bool XimeaAdapter::trigger()
{
	logMessage(GenICamAdapterInterface::INFO_MSG, "Sending Software Trigger...");
	setIntegerParamVal(XI_PRM_TRG_SOFTWARE, 1);
	return true;
}

void XimeaAdapter::collect()
{
	// TODO - DEBUG remove
	//std::chrono::steady_clock::time_point start;
	//std::chrono::duration<double, std::milli> delta;
	lock_guard<mutex> guard(deviceMutex);
	// image buffer
	XI_IMG image;
	memset(&image, 0, sizeof(image));
	image.size = sizeof(XI_IMG);
	//start = std::chrono::high_resolution_clock::now();
	XI_RETURN retVal = xiGetImage(this->deviceHandle, 1000, &image); // 1000
	//delta = std::chrono::high_resolution_clock::now() - start;
	//std::cout << "Ximea Driver: xiGeImage() ( " << delta.count() << ")" << std::endl;

	if (retVal == XI_OK)
	{
		GenICamAdapterInterface::RawImageInfoType imageInfo;
		imageInfo.frameID = image.nframe;
		imageInfo.timestamp = image.tsSec;
		imageInfo.width = image.width;
		imageInfo.height = image.height;
		uint32_t bytesPerPixel = 0;
		switch (image.frm)
		{
			case XI_MONO8:
			default:
				imageInfo.pixelSize = 8;
				bytesPerPixel = 1;
				break;
			case XI_MONO16:
				imageInfo.pixelSize = 16;
				bytesPerPixel = 2;
				break;
		}
		//determine row size
		imageInfo.rowSize = bytesPerPixel * imageInfo.width + image.padding_x;
		imageInfo.numChannels = 1;// 1;
		imageInfo.isPixelSigned = false;

		// At CSIRO Clayton, the Ximea cameras are returning correct images, so no need to flip.
#ifdef DONT_FLIP_XIMEA
		imageInfo.flipVertical = false;
		imageInfo.flipHorizontal = false;
#else
		//flip the image vertically and horizontally
		imageInfo.flipVertical = true;
		imageInfo.flipHorizontal = true;
#endif
		//mark as valid
		imageInfo.isValid = true;
		//add new image to current image queue
		createImage(imageInfo, (char*)image.bp, image.bp_size);
	}
	else if ((retVal != XI_OK) && (retVal != XI_TIMEOUT))
	{
		stringstream errStream;
		errStream << "Collection failed! Error Code: " << retVal;
		logMessage(GenICamAdapterInterface::ERROR_MSG, errStream.str());
	}
}

void XimeaAdapter::endAcquire()
{
    logMessage(GenICamAdapterInterface::INFO_MSG, "End acquisition...");
	lock_guard<mutex> guard(deviceMutex);
	XI_RETURN retVal = xiStopAcquisition(deviceHandle);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "End acquisition failed! Error Code: " << retVal;
		logMessage(GenICamAdapterInterface::ERROR_MSG, errStream.str());
	}    
}

void XimeaAdapter::reset()
{
    logMessage(GenICamAdapterInterface::INFO_MSG, "Resetting device...");
	lock_guard<mutex> guard(deviceMutex);
    releaseResources();
	XI_RETURN retVal = xiSetParamInt(deviceHandle, XI_PRM_DEVICE_RESET, 1);
	if (retVal != XI_OK)
	{
		stringstream errStream;
		errStream << "Reset failed! Error Code: " << retVal;
		logMessage(GenICamAdapterInterface::ERROR_MSG, errStream.str());
	}
}

bool XimeaAdapter::isAcquiring()
{
	bool status = false;
	getBoolParamVal(XI_PRM_ACQUISITION_STATUS, status);
	return status;
}

void XimeaAdapter::setSerialNumber(const std::string &deviceId) {
	static char sn[256];
	XI_RETURN retVal = xiGetDeviceInfoString(std::stoi(deviceId), XI_PRM_DEVICE_SN, sn, sizeof(sn));
	if (retVal != XI_OK) {
		std::stringstream errStream;
		errStream << "Get serial number failed! Error Code: " << retVal << std::endl;
		logMessage(GenICamAdapterInterface::ERROR_MSG, errStream.str());
	}
	this->serialNumber = sn;
}