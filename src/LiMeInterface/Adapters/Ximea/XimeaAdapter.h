#pragma once
#include <vector>
#include <cstdint>
#include <mutex>
#include <iostream>
#include "../GenICamInterface/GenICamAdapterInterface.h"

#include <xiApi.h>
using namespace std;

/*! \class XimeaAdapter
    \brief A derived class used for communicating with Ximea devices
    \implements GenICamAdapterInterface

    This class implements the GenICamAdapterInterface and provides the means for
    interacting with a Ximea device. For additional information, see GenICamAdapterInterface.
*/
class XimeaAdapter : public GenICamAdapterInterface
{
public:
    XimeaAdapter();
    ~XimeaAdapter();

    virtual bool initializeDevice(string deviceSerialNumber);

    virtual void setIntegerParamVal(string name, int64_t val) override;
    virtual void getIntegerParamVal(string name, int64_t &val) override;

    virtual void setFloatParamVal(string name, double val) override;
    virtual void getFloatParamVal(string name, double &val) override;

    virtual void setBoolParamVal(string name, bool val) override;
    virtual void getBoolParamVal(string name, bool &val) override;

    virtual void setEnumParamVal(string name, string val) override;
    virtual void setEnumParamVal(string name, int64_t val) override;
    virtual void getEnumParamVal(string name, string &val) override;
    virtual void getEnumParamVal(string name, int64_t &val) override;

    virtual void setStringParamVal(string name, string val) override;
    virtual void getStringParamVal(string name, string &val) override;

    virtual void executeCommand(string name) override;
    virtual void beginAcquire() override;
    virtual bool trigger() override;
	virtual void collect() override;
    virtual void endAcquire() override;
    virtual void reset() override;
	virtual bool isAcquiring() override;

	void setSerialNumber(const std::string &deviceId) override;

protected:

    void releaseResources();

	HANDLE deviceHandle;
	mutex deviceMutex;
};


