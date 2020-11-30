//#pragma once
#include <time.h>
#include <iomanip>
#include <string>
#include <mutex>
#include <iostream>
#include "GenICamAdapterInterface.h"
#include "../ImperX/ImperxAdapter.h"
// #include "BaumerAdapter.h"
#include "../Ximea/XimeaAdapter.h"
// #include "ExoTracerAdapter.h"
// #include "LineaAdapter.h"

using namespace std;

GenICamAdapterInterface::GenICamAdapterInterface() : initialized(false)
{

}

GenICamAdapterInterface::~GenICamAdapterInterface() = default;


GenICamAdapterInterface* GenICamAdapterInterface::createAdapter(CameraEnumType cameraType)
{
    GenICamAdapterInterface *newAdapter = nullptr;
    switch (cameraType)
    {
        case IMPERX:
            newAdapter = new ImperxAdapter();
            break;
        //case BAUMER:
        //    newAdapter = new BaumerAdapter();
        //    break;
        case XIMEA:
            newAdapter = new XimeaAdapter();
            break;
        //case LINEA:
        //    newAdapter = new LineaAdapter();
        //    break;
        //case EXO_TRACER:
        //    newAdapter = new ExoTracerAdapter();
        //    break;
        case JAI:
        case BASLER:
        default:
            //unsupported adapter (not implemented)
            break;
    }
    return newAdapter;
}

bool GenICamAdapterInterface::isInitialized()
{
    return initialized;
}

string GenICamAdapterInterface::getAdapterName()
{
    return adapterName;
}

string GenICamAdapterInterface::getSerialNumber()
{
    return this->serialNumber;
}

void GenICamAdapterInterface::logMessage(SeverityEnumType severity, string message)
{
    lock_guard<mutex> guard(logMutex);
    //prepend adapter name to messages
    stringstream ss;
    ss << adapterName << ":" << serialNumber << " " << message;
    LogMessageType genICamMessage = { severity, ss.str() };
    logs.push_back(genICamMessage);
}

bool GenICamAdapterInterface::getNextLog(LogMessageType &log)
{
    lock_guard<mutex> guard(logMutex);
    bool retVal = false;
    if (!logs.empty())
    {
        //get next message
        log = logs.front();
        retVal = true;
    }
    return retVal;
}

void GenICamAdapterInterface::popNextLog()
{
    lock_guard<mutex> guard(logMutex);
    if (!logs.empty())
    {
        logs.pop_front();
    }
}

void GenICamAdapterInterface::createImage(RawImageInfoType imageInfo, char *imageData, uint64_t imageSize)
{
    lock_guard<mutex> guard(imageQueueMutex);
    if (imageQueue.size() < MAX_IMAGE_QUEUE_SIZE)
    {
        //create new image (in place)
        imageQueue.emplace_back();

        //get reference to new image
        RawImageType &newImage = imageQueue.back();
        //populate the image
        newImage.info = imageInfo;
		newImage.buffer = std::shared_ptr<RawImageBufferType>(new RawImageBufferType());
        newImage.buffer->insert(newImage.buffer->begin(), imageData, imageData + imageSize);
    }
    else
    {
		// FIXME - should report but only spams console
        // logMessage(GenICamAdapterInterface::WARNING_MSG, "Image overflow!");
    }
}

bool GenICamAdapterInterface::getNextImage(RawImageType &image)
{
    lock_guard<mutex> guard(imageQueueMutex);
    bool retVal = false;
    if (!imageQueue.empty())
    {
        image = imageQueue.front();
        retVal = true;
    }
    return retVal;
}

void GenICamAdapterInterface::popNextImage()
{
    lock_guard<mutex> guard(imageQueueMutex);
    if (!imageQueue.empty())
    {
        //pop from queue
        imageQueue.pop_front();
    }
}

void GenICamAdapterInterface::setSerialNumber(const std::string &serial) {
	this->serialNumber = serial;
}

bool GenICamAdapterInterface::isAcquiring()
{
    bool status = false;
    getBoolParamVal("AcquisitionStatus", status);
    return status;
}

std::string GenICamAdapterInterface::zeroPadNumber(const int & src, const int & dstWidth)
{
    std::ostringstream ss;
    ss << std::setw(dstWidth) << std::setfill('0') << src;
    return ss.str();
}


std::string GenICamAdapterInterface::getCurrentTimeInSeconds()
{
    time_t t = time(0);
    struct tm now;
#ifdef _MSC_VER
        localtime_s(&now, &t);
#else
    // https://en.cppreference.com/w/c/chrono/localtime
    now = *localtime(&t);
#endif
    std::string dateStr = std::to_string(now.tm_year + 1900) + zeroPadNumber(now.tm_mon + 1, 2) +
        zeroPadNumber(now.tm_mday, 2) + "_" + zeroPadNumber(now.tm_hour, 2) +
        zeroPadNumber(now.tm_min, 2) + zeroPadNumber(now.tm_sec, 2);

    return dateStr;
}


