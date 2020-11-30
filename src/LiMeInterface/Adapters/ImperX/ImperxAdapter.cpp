#define INC_GEN_TL_ERROR_CODES

#include <system_error>

#include "ImperxAdapter.h"

using namespace std;

const uint64_t ImperxAdapter::BUFFER_TIMEOUT_MS = 1000;

map<IpxCamErr, string> ImperxAdapter::errorMap = { { IPX_CAM_GENICAM_GENERIC_ERROR, "Generic Error" }, { IPX_CAM_GENICAM_TREE_ERROR, "Tree Error" }, { IPX_CAM_GENICAM_ACCESS_ERROR, "Access Error" },
                                                   { IPX_CAM_GENICAM_TYPE_ERROR, "Type Error" }, { IPX_CAM_GENICAM_OUT_OF_RANGE, "Out of Range" }, { IPX_CAM_GENICAM_UNKNOWN_PARAM, "Unknown Param" },
                                                   { IPX_CAM_GENICAM_INVALID_ARGUMENT, "Invalid Argument" } };

ImperxAdapter::ImperxAdapter() : system(nullptr), device(nullptr), stream(nullptr), acquisitionThreadHandle(nullptr), isModelC4080(false)
{
    adapterName = "ImperX";
    isAcquisitionThreadRunning = false;
    //isAcquisition = true;
	isAcquisition = false;
}

ImperxAdapter::~ImperxAdapter()
{
    //end acquisition if started
    if (running())
    {
        endAcquire();
    }
    release();
}

void ImperxAdapter::release()
{
    if (stream)
    {
        stream->ReleaseBufferQueue();
        stream->Release();
        stream = nullptr;
    }
    //release the device
    if (device)
    {
        device->Release();
        device = nullptr;
    }
    //release the system
    if (system)
    {
        system->Release();
        system = nullptr;
    }
}

bool ImperxAdapter::initializeDevice(string deviceSerialNumber)
{
	bool isMapped = false;
    //release any previously allocated resources
    release();
    //get system
    try
    {
        system = IpxCam::IpxCam_GetSystem();
        if (system)
        {
            IpxCam::InterfaceList *interfaceListPtr = system->GetInterfaceList();
            //iterate our interfaces
            for (IpxCam::Interface *interfacePtr = interfaceListPtr->GetFirst(); interfacePtr; interfacePtr = interfaceListPtr->GetNext())
            {
                IpxCam::DeviceInfoList *deviceInfoListPtr = interfacePtr->GetDeviceInfoList();
                //iterate the devices attached to each interface
                for (IpxCam::DeviceInfo *deviceInfoPtr = deviceInfoListPtr->GetFirst(); deviceInfoPtr; deviceInfoPtr = deviceInfoListPtr->GetNext())
                {
                    string imperxSerialNumber(deviceInfoPtr->GetSerialNumber());
					//std::cout << "ImperX SN: " << imperxSerialNumber << std::endl;
                    //see if it matches this device
                    if (imperxSerialNumber == deviceSerialNumber)
                    {
                        if (deviceInfoPtr->GetAccessStatus() != IpxCam::DeviceInfo::AccessStatusNoAccess)
                        {
							// Connect to the USB3 Vision device
                            device = IpxCam::IpxCam_CreateDevice(deviceInfoPtr);

							// HRL - addition
							// Found there were cases when device failed to allocate and was left unchecked null_ptr
							if (!device) {
								throw std::system_error(EAGAIN, std::generic_category(), "Failed to connect IpxCamera");
							}

                            serialNumber = imperxSerialNumber;
                            //we found what we needed
                            initialized = true;
                            isMapped = true;

							if (std::string("U3V-C4080C-RF0000") == deviceInfoPtr->GetModel())
							{
								isModelC4080 = true;
							}
                            break;
                        }
                    }
                }
            }
        }
        else
        {
            // log error
            logMessage(GenICamAdapterInterface::ERROR_MSG, "Could not get system!");
        }
    }
	// HRL - addition
	// Found there were cases when device failed to allocate and was left unchecked null_ptr
	catch (const std::system_error & error) {
		logMessage(GenICamAdapterInterface::ERROR_MSG, std::string("ImperxAdapter::initializeDevice exception: ") + error.what());
	} 
	catch (std::exception & e) {
        // log error
        logMessage(GenICamAdapterInterface::ERROR_MSG, std::string("ImperxAdapter::initializeDevice exception: ") + e.what());
    }

    return isMapped;
}

void ImperxAdapter::setIntegerParamVal(string name, int64_t val)
{
    // The default Model of ImperxAdapter is C5180. For Model C4080 used in Clayton, some options are slightly different.
    if (isModelC4080)
    {
        if (name == "Frame_DigitalGain" || name == "Frame_DigitalOffset" || name == "Frame_ExposureTime" || name == "Frame_BlackLevelOffset")
        {
            stringstream ss;
            ss << "Ignore: " << name << " setting on Model: C4080";
            logMessage(GenICamAdapterInterface::WARNING_MSG, ss.str());
            return;
        }
        else if (name == "Height")
        {
            if (val > 3016)
            {
                val = 3016;
                logMessage(GenICamAdapterInterface::WARNING_MSG, "Clamp Height to 3016 on Model: C4080");
            }
        }
        else if (name == "Width")
        {
            if (val > 4016)
            {
                val = 4016;
                logMessage(GenICamAdapterInterface::WARNING_MSG, "Clamp Width to 4016 on Model: C4080");
            }
        }
        else if (name == "Frame_FixedFramePeriodValue")
        {
            setIntegerParamVal("Frame_LineTime", val);
        }
    }


    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    IpxGenParam::Int *param = paramArray->GetInt(name.c_str(), &errorCode);
    if (errorCode == IPX_CAM_ERR_OK)
    {
        //check range
        if (val <= param->GetMax() && val >= param->GetMin())
        {
            errorCode = param->SetValue(val);
        }
        else
        {
            stringstream ss;
            ss << "Parameter: " << name << " is out of valid range: max->" << param->GetMax() << " min->" << param->GetMin();
            logMessage(GenICamAdapterInterface::WARNING_MSG, ss.str());
        }
    }
    //catch all
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //report error
        logSetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::getIntegerParamVal(string name, int64_t &val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    val = paramArray->GetIntegerValue(name.c_str(), &errorCode);
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        logGetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::setFloatParamVal(string name, double val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    IpxGenParam::Float *param = paramArray->GetFloat(name.c_str(), &errorCode);
    if (errorCode == IPX_CAM_ERR_OK)
    {
        //check range
        if (val <= param->GetMax() && val >= param->GetMin())
        {
            errorCode = param->SetValue(val);
        }
        else
        {
            stringstream ss;
            ss << "Parameter: " << name << " is out of valid range: max->" << param->GetMax() << " min->" << param->GetMin();
            logMessage(GenICamAdapterInterface::WARNING_MSG, ss.str());
        }
    }
    //catch all
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //report error
        logSetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::getFloatParamVal(string name, double &val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    val = paramArray->GetFloatValue(name.c_str(), &errorCode);
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        logGetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::setBoolParamVal(string name, bool val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    IpxGenParam::Boolean *param = paramArray->GetBoolean(name.c_str(), &errorCode);
    if (errorCode == IPX_CAM_ERR_OK)
    {
        errorCode = param->SetValue(val);
    }
    //catch all
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //report error
        logSetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::getBoolParamVal(string name, bool &val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    val = paramArray->GetBooleanValue(name.c_str(), &errorCode);
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        logGetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::setEnumParamVal(string name, string val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();

    // The default Model of ImperxAdapter is C5180. For Model C4080 used in Clayton, some options are slightly different.
    if (isModelC4080)
    {
        if (name == "Frame_AnalogGain")
        {
            static std::map<std::string, std::string> analogGainMap =
            {
                {"Gain_1x0", "Gain_1x"},
                {"Gain_1x26", "Gain_2x"},
                {"Gain_1x87", "Gain_4x"},
                {"Gain_3x17", "Gain_8dB"},
            };
            auto it = analogGainMap.find(val);
            val = it == analogGainMap.end() ? "Gain_1x" : it->second;
        }
        else if (name == "Frame_BlackLevelAutoCalibration")
        {
            name = "WhiteBalanceMode";
            if (val == "On")
            {
                val = "Auto";
            }
        }

    }

    IpxCamErr errorCode = paramArray->SetEnumValueStr(name.c_str(), val.c_str());
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        logSetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::setEnumParamVal(string name, int64_t val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    IpxGenParam::Enum *param = paramArray->GetEnum(name.c_str(), &errorCode);
    if (errorCode == IPX_CAM_ERR_OK)
    {
        errorCode = param->SetValue(val);
    }
    //catch all
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //report error
        logSetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::getEnumParamVal(string name, string &val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    val = paramArray->GetEnumValueStr(name.c_str(), &errorCode);
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        logGetParamMessage(name, val, errorCode);
    }
}
void ImperxAdapter::getEnumParamVal(string name, int64_t &val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    val = paramArray->GetEnumValue(name.c_str(), &errorCode);
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        logGetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::setStringParamVal(string name, string val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode = paramArray->SetStringValue(name.c_str(), val.c_str());
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        logSetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::getStringParamVal(string name, string &val)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode;
    val = paramArray->GetStringValue(name.c_str(), &errorCode);
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        logGetParamMessage(name, val, errorCode);
    }
}

void ImperxAdapter::executeCommand(string name)
{
    IpxGenParam::Array *paramArray = device->GetCameraParameters();
    IpxCamErr errorCode = paramArray->ExecuteCommand(name.c_str());
    //see if we succeeded
    if (errorCode != IPX_CAM_ERR_OK)
    {
        //throw error
        stringstream ss;
        ss << "Could not execute command: " << name << "(" << errorCode << ")";
        logMessage(GenICamAdapterInterface::ERROR_MSG, ss.str());
    }
}

void ImperxAdapter::beginAcquire()
{
    //reset acquisition
    acquisitionStop();
    acquisitionStart();
    //start our acquisition thread
    startAcquisitionThread();
    
}

bool ImperxAdapter::trigger()
{
    logMessage(GenICamAdapterInterface::DEBUG_MSG, "Performing Software Trigger ");
    //send the trigger
    executeCommand("TriggerSoftware");
	return true;
}

void ImperxAdapter::collect()
{
	//not implemented yet
}

void ImperxAdapter::endAcquire()
{
    //stop our acquisition thread
    stopAcquisitionThread();
    //stop acquisition from device
    acquisitionStop();
}

void ImperxAdapter::reset()
{
    executeCommand("DeviceReset");
    //wait for device to cycle
    Sleep(5000);
}

void ImperxAdapter::acquisitionStart()
{
    //open stream (always use first stream)
    stream = device->GetStreamByIndex(0);
    if (!stream)
    {
        //log error
        stringstream ss;
        ss << "Could not get stream!";
        logMessage(GenICamAdapterInterface::ERROR_MSG, ss.str());
        release();
        return;
    }
    //allocate buffers
    size_t minNumBuffers = stream->GetMinNumBuffers();
    stream->AllocBufferQueue(nullptr, minNumBuffers);


	if (device->GetCameraParameters()) {
		//lock down transport layer
		setIntegerParamVal("TLParamsLocked", 1);
	}
    
    //enable stream and send Start Acquisition command
	auto result = stream->StartAcquisition();
    if (result != IPX_CAM_ERR_OK)
    {
        //throw error
        stringstream ss;
        ss << "Could not start stream!";
        logMessage(GenICamAdapterInterface::ERROR_MSG, ss.str());
        release();
        return;
    }
    executeCommand("AcquisitionStart");
    //signal that we started acquisition
    isAcquisition = true;
}

void ImperxAdapter::acquisitionStop()
{
	IpxGenParam::Array *params = device->GetCameraParameters();

	// Added! Acquisition is occuring, so quit
	if (!params) {
		return;
	}


    if (stream)
    {
        // Cancel I/O for current buffer
        stream->CancelBuffer();
    }
    // stop acquisition
    if (IPX_CAM_ERR_OK != params->ExecuteCommand("AcquisitionStop"))
    {
        executeCommand("AcquisitionAbort");
    }
    if (stream)
    {
        IpxCamErr error = stream->StopAcquisition(1);
        if ((error != IPX_CAM_ERR_OK) && (error != GC_ERR_RESOURCE_IN_USE))
        {
            //throw error
            stringstream ss;
            ss << "Could not stop stream!";
            logMessage(GenICamAdapterInterface::ERROR_MSG, ss.str());
            release();
            return;
        }
        //flush all buffers
        stream->FlushBuffers(IpxCam::Flush_AllDiscard);
        //release stream
        stream->ReleaseBufferQueue();
        stream->Release();
        stream = nullptr;
    }
    setIntegerParamVal("TLParamsLocked", 0);
    //signal that we've stopped acquisition
    isAcquisition = false;
}

void ImperxAdapter::startAcquisitionThread()
{
    //kick off acquisition thread
    isAcquisitionThreadRunning = true;
    DWORD mID;
    acquisitionThreadHandle = CreateThread(NULL,
                                           0,
                                           (LPTHREAD_START_ROUTINE) acquisitionLink,
                                           this,
                                           0,
                                           &mID);
    //set thread priority
    SetThreadPriority(acquisitionThreadHandle, THREAD_PRIORITY_HIGHEST);
}

void ImperxAdapter::stopAcquisitionThread()
{
    //indicate to thread to stop and wait for it to complete
    isAcquisitionThreadRunning = false;
    WaitForSingleObject(acquisitionThreadHandle, INFINITE);

    CloseHandle(acquisitionThreadHandle);
    acquisitionThreadHandle = INVALID_HANDLE_VALUE;
}
// WINAPI
unsigned long ImperxAdapter::acquisitionLink(void *adapterObject)
{
    ImperxAdapter *thisAdapter = reinterpret_cast<ImperxAdapter *>(adapterObject);
    bool restartAcquisition = false;
    do
    {
        //verify that the thread didn't signal to restart acquisition
        restartAcquisition = thisAdapter->acquisitionLoop();
        if (restartAcquisition)
        {
            //restart our acquisition
            thisAdapter->acquisitionStop();
            thisAdapter->acquisitionStart();
        }
    } while (restartAcquisition);
    return 0;
}

bool ImperxAdapter::acquisitionLoop()
{
    //flag used to indicate if acquisition needs to be restarted (error recovery)
    bool doRestart = false;
    // Working cycle
    while (running())
    {
        //get a buffer for the incoming image
        IpxCamErr err;
        IpxCam::Buffer* buffer = nullptr;
        try
        {
            buffer = stream->GetBuffer(BUFFER_TIMEOUT_MS, &err);
        }
        catch (...)
        {
            //no buffer available yet
        }

        if (err == GC_ERR_TIMEOUT)
        {
            //no buffer available yet
            continue;
        }
        if (err != IPX_CAM_ERR_OK)
        {
            //log error
            stringstream ss;
            ss << "Could not get buffer, returned code: " << err;
            logMessage(GenICamAdapterInterface::WARNING_MSG, ss.str());
            //signal a restart upon return
            doRestart = true;
            break;
        }
        if (buffer)
        {
            if (buffer->IsIncomplete())
            {
                //log error
                stringstream ss;
                ss << "Obtained incomplete buffer!";
                logMessage(GenICamAdapterInterface::WARNING_MSG, ss.str());
                //signal a restart upon return
                doRestart = true;
                break;
            }
            else
            {
                //get pointer to new image
                IpxImage *imagePtr = buffer->GetImage();
                if (imagePtr)
                {
                    RawImageInfoType imageInfo;
                    //populate our local image info structure
                    imageInfo.height = imagePtr->height;
                    imageInfo.width = imagePtr->width;
                    imageInfo.rowSize = imagePtr->rowSize;
                    imageInfo.pixelSize = imagePtr->pixelTypeDescr.pixSize;
                    imageInfo.isPixelSigned = imagePtr->pixelTypeDescr.pixSigned;
                    imageInfo.numChannels = imagePtr->pixelTypeDescr.channels;
                    imageInfo.flipHorizontal = false;
                    imageInfo.flipVertical = false;
                    //determine validity of image last
                    imageInfo.isValid = ((imageInfo.height > 0) && (imageInfo.width > 0) && (imageInfo.pixelSize > 0));
                    createImage(imageInfo, imagePtr->imageData, imagePtr->imageSize);
                    // Re-queue the buffer in the stream object
                    if (stream->QueueBuffer(buffer) != IPX_CAM_ERR_OK)
                    {
                        //log error
                        stringstream ss;
                        ss << "Could not re-queue buffer after image acquisition";
                        logMessage(GenICamAdapterInterface::ERROR_MSG, ss.str());
                    }
                }
                else
                {
                    //log error
                    stringstream ss;
                    ss << "Could not get image from buffer!";
                    logMessage(GenICamAdapterInterface::ERROR_MSG, ss.str());
                    release();
                    break;
                }
            }
        }
        else
        {
            //log error
            stringstream ss;
            ss << "Obtained bad buffer!";
            logMessage(GenICamAdapterInterface::ERROR_MSG, ss.str());
            release();
            break;
        }
    }
    return doRestart;
}

bool ImperxAdapter::running()
{
    return isAcquisitionThreadRunning;
}

bool ImperxAdapter::isAcquiring()
{
    return isAcquisition;
}
