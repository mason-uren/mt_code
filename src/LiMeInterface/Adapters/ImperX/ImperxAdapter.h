#pragma once
#include <vector>
#include <map>
#include <cstdint>
#include <string>
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>

//#include <windows.h>//TODO: need to rid dependency for cross-platform dependency
// From HRL
#include <Shared/OSDefines.h>

#include "IpxCameraApi.h"
#include "IpxCameraErr.h"
#include "IpxImage.h"
#include "IpxPixelType.h"

#include "../GenICamInterface/GenICamAdapterInterface.h"

using namespace std;

/*! \class ImperxAdapter
    \brief A derived class used for communicating with ImperX devices
    \implements GenICamAdapterInterface

    This class implements the GenICamAdapterInterface and provides the means for
    interacting with an ImperX device. For additional information, see GenICamAdapterInterface.
*/
class ImperxAdapter : public GenICamAdapterInterface
{
public:
    ImperxAdapter();
    ~ImperxAdapter();

    virtual bool initializeDevice(string deviceSerialNumber);

    virtual void setIntegerParamVal(string name, int64_t val);
    virtual void getIntegerParamVal(string name, int64_t &val);

    virtual void setFloatParamVal(string name, double val);
    virtual void getFloatParamVal(string name, double &val);

    virtual void setBoolParamVal(string name, bool val);
    virtual void getBoolParamVal(string name, bool &val);

    virtual void setEnumParamVal(string name, string val);
    virtual void setEnumParamVal(string name, int64_t val);
    virtual void getEnumParamVal(string name, string &val);
    virtual void getEnumParamVal(string name, int64_t &val);

    virtual void setStringParamVal(string name, string val);
    virtual void getStringParamVal(string name, string &val);

    virtual void executeCommand(string name);
    virtual void beginAcquire();
    virtual bool trigger();
	virtual void collect() override;
    virtual void endAcquire();
    virtual void reset();

    bool isAcquiring() override;

    virtual void acquisitionStart();
    virtual void acquisitionStop();

protected:

    static map<IpxCamErr, string> errorMap;

    template <typename T>
    void logSetParamMessage(string name, T val, IpxCamErr errorCode)
    {
        string errorCodeString = "Unknown Error";
        
        //retrieve the error string
        if (errorMap.count(errorCode) > 0)
        {
            errorCodeString = errorMap[errorCode];
        }
        //build the message
        stringstream ss;
        ss << "Could not set parameter: " << name << " to value: " << val << " (" << errorCodeString << ")";
        logMessage(GenICamAdapterInterface::WARNING_MSG, ss.str());
    }

    template <typename T>
    void logGetParamMessage(string name, T val, IpxCamErr errorCode)
    {
        string errorCodeString = "Unknown Error";
        
        //retrieve the error string
        if (errorMap.count(errorCode) > 0)
        {
            errorCodeString = errorMap[errorCode];
        }
        //build the message
        stringstream ss;
        ss << "Could not get parameter: " << name << " to value: " << val << " (" << errorCodeString << ")";
        logMessage(GenICamAdapterInterface::WARNING_MSG, ss.str());
    }

    void startAcquisitionThread();
    void stopAcquisitionThread();
    bool acquisitionLoop();
    bool running();
    void release();

    static unsigned long acquisitionLink(void *adapterObject);

    IpxCam::System *system;
    IpxCam::Device *device;
    IpxCam::Stream *stream;

    HANDLE acquisitionThreadHandle;
    bool isAcquisitionThreadRunning;
    bool isAcquisition;

    bool isModelC4080;

    static const uint64_t BUFFER_TIMEOUT_MS;
};


