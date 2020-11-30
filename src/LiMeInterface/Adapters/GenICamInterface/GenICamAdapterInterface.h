#pragma once
#include <string>
#include <cstdint>
#include <deque>
#include <vector>
#include <sstream>
#include <mutex>
#include <memory>

#include "GenICam_API.h"
using namespace std;

/*! \class GenICamAdapterInterface
    \brief An abstract class (interface) meant to serve as a generic entry point to controlling
           GenICam capable devices.

    This abstract class allows clients to construct GenICam device adapters and use its interface
    to communicate with its mapped device. Design allows for additional adapters to be added to support more
    GenICam capable devices.
*/
// class GENICAM_API GenICamAdapterInterface
class GenICamAdapterInterface
{

public:

    /*! \brief Pure virtual destructor

        Derived adapter destructors should override this
    */
    virtual ~GenICamAdapterInterface() = 0;

    /*! \brief Map device to this adapter object given device's serial number
        \param[in] deviceSerialNumber Device serial number
        \return Returns true if the device mapping succeeded

        This method serves as the first entry into an adapter object. This must be called
        before anything else.
    */
    virtual bool initializeDevice(string deviceSerialNumber) = 0;

    /*! \brief Determine if adapter is initialized
    */
    virtual bool isInitialized();

    /*! \brief Mutator for a device's integer setting
        \param[in] name Name of setting
        \param[in] val Value to change setting to

        This method allows a client to alter an integer setting
    */
    virtual void setIntegerParamVal(string name, int64_t val) = 0;

    /*! \brief Accessor for a device's integer setting
        \param[in] name Name of setting
        \param[out] val Value to populate

        This method allows a client to retrieve an integer setting
    */
    virtual void getIntegerParamVal(string name, int64_t &val) = 0;

    /*! \brief Mutator for a device's floating point setting
        \param[in] name Name of setting
        \param[in] val Value to change setting to

        This method allows a client to alter a floating point setting
    */
    virtual void setFloatParamVal(string name, double val) = 0;

    /*! \brief Accessor for a device's floating point setting
        \param[in] name Name of setting
        \param[out] val Value to populate

        This method allows a client to retrieve a floating point setting
    */
    virtual void getFloatParamVal(string name, double &val) = 0;

    /*! \brief Mutator for a device's boolean setting
        \param[in] name Name of setting
        \param[in] val Value to change setting to

        This method allows a client to alter a boolean setting
    */
    virtual void setBoolParamVal(string name, bool val) = 0;

    /*! \brief Accessor for a device's boolean setting
        \param[in] name Name of setting
        \param[out] val Value to populate

        This method allows a client to retrieve a boolean setting
    */
    virtual void getBoolParamVal(string name, bool &val) = 0;

    /*! \brief Mutator for a device's enumerated string setting
        \param[in] name Name of setting
        \param[in] val Value to change setting to

        This method allows a client to alter a enumerated string setting
    */
    virtual void setEnumParamVal(string name, string val) = 0;

    /*! \brief Mutator for a device's enumerated integer setting
        \param[in] name Name of setting
        \param[in] val Value to change setting to

        This method allows a client to alter a enumerated integer setting
    */
    virtual void setEnumParamVal(string name, int64_t val) = 0;

    /*! \brief Accessor for a device's enumerated string setting
        \param[in] name Name of setting
        \param[out] val Value to populate

        This method allows a client to retrieve a enumerated string setting
    */
    virtual void getEnumParamVal(string name, string &val) = 0;

    /*! \brief Accessor for a device's enumerated integer setting
        \param[in] name Name of setting
        \param[out] val Value to populate

        This method allows a client to retrieve a enumerated integer setting
    */
    virtual void getEnumParamVal(string name, int64_t &val) = 0;

    /*! \brief Mutator for a device's string setting
        \param[in] name Name of setting
        \param[in] val Value to change setting to

        This method allows a client to alter a string setting
    */
    virtual void setStringParamVal(string name, string val) = 0;

    /*! \brief Accessor for a device's string setting
        \param[in] name Name of setting
        \param[out] val Value to populate

        This method allows a client to retrieve a string setting
    */
    virtual void getStringParamVal(string name, string &val) = 0;

    /*! \brief Sends a command to the device
        \param[in] name Name of command

        This method allows a client to send a command to a GenICam device for execution
    */
    virtual void executeCommand(string name) = 0;

    /*! \brief Resets the device

        This method resets the device. Note: Client must consider time it takes for device
        to become active after it is reset.
    */
    virtual void reset() = 0;

    /*! \brief Begin acquisition of an image from the device

        This method is used to begin acquisition of images from the GenICam device.
    */
    virtual void beginAcquire() = 0;

    /*! \brief Trigger the capture of an image from the device

        This method is used to trigger a capture via software from the GenICam device.
    */
    virtual bool trigger() = 0;

    /*! \brief Collect an image from the device

    This method is used to collect an image from the GenICam device.
    */
    virtual void collect() = 0;

    /*! \brief End acquisition of an image from the device

        This method is used to end acquisition of images from the GenICam device.
    */
    virtual void endAcquire() = 0;

    /*! \brief A enum type that serves as the means for retrieving the correct adapter object from createAdapter().

        This can be expanded as new adapters are implemented.
    */
    enum CameraEnumType
    {
        IMPERX,
        BAUMER,
        XIMEA,
        JAI,
        BASLER,
        EXO_TRACER,
        LINEA
    };

    /*! \brief Retrieve polymorphic pointer to adapter object
        \param[in] cameraType Type of camera that the client an adapter instance for

        This static method is used to construct an adapter object based on the specified camera type.
    */
    static GenICamAdapterInterface* createAdapter(CameraEnumType cameraType);

    /*! \brief An enum type that serves as a message severity type */
    enum SeverityEnumType
    {
        DEBUG_MSG,
        INFO_MSG,
        ERROR_MSG,
        WARNING_MSG,
        NUM_MSG_TYPES
    };

    /*! \brief A struct type that serves as a container for a log message */
    struct LogMessageType
    {
        SeverityEnumType severity;
        string what;
    };
    typedef deque<LogMessageType> LogMessageQueueType;

    /*! \brief Retrieve a message
    \param[out] severity Severity of message
    \param[out] what Message content
    \return If there are any messages left to retrieve

    This method allows a client retrieve any logged messages
    */
    virtual bool getNextLog(LogMessageType &logMessage);

    /*! \brief Remove oldest log from queue
    */
    virtual void popNextLog();

    /*! \brief Determine acquisition state
    \return True if the device is acquiring
    */
    virtual bool isAcquiring();

    /*! \brief Accessor for getting a string representation of the adapter object
        \return Adapter name
    */
    virtual string getAdapterName();

    /*! \brief Accessor for getting a string representation of the GenICam device's serial number
        \return Serial number
    */
    virtual string getSerialNumber();

    /*! \brief A type alias that serves as a container for an acquired image */
    typedef vector<uint8_t> RawImageBufferType;

    typedef struct
    {
        bool isValid;
        uint64_t frameID;
        uint64_t timestamp;
        uint32_t height;
        uint32_t width;
        uint32_t rowSize;
        uint32_t pixelSize;//in bits
        bool isPixelSigned;
        uint32_t numChannels;
        bool flipVertical;
        bool flipHorizontal;
    } RawImageInfoType;

    /*! \brief A struct type that serves as a container for information associated with an acquired image */
    typedef struct
    {
        RawImageInfoType info;
		
		// HRL - modified to shared_ptr to inherit destructor (removed memory leak)
		std::shared_ptr<RawImageBufferType> buffer;
    } RawImageType;

    /*! \brief Constructs image at front of queue
    */
    virtual void createImage(RawImageInfoType imageInfo, char *imageData, uint64_t imageSize);

    /*! \brief Accessor for getting a reference to a raw image  that is populated during image acquisition
        \return Reference to image
    */
    virtual bool getNextImage(RawImageType &image);

    /*! \brief Remove oldest image from queue
    */
    virtual void popNextImage();

	virtual void setSerialNumber(const std::string &serial);

protected:
    /*! \brief Constructor
    */
    GenICamAdapterInterface();

    // Int to string with zero-padding
    static std::string zeroPadNumber(const int & src, const int & dstWidth);

    // YYYYMMDD_HHMMSS
    static std::string getCurrentTimeInSeconds();


    /*! \brief A type alias for the container of raw images */
    typedef deque<RawImageType> RawImageQueueType;

    /*! \brief Log a message
    \param[in] severity Severity of message
    \param[in] message Message content

    This method allows a client log a message during execution of the associated adapter object
    */
    virtual void logMessage(SeverityEnumType severity, string message);

    /*! GenICam adapter's state */
    bool initialized;
    /*! GenICam device's serial number */
    string serialNumber;
    /*! GenICam adapter name */
    string adapterName;
    /*! Container for raw images retrieved during image acquisition */
    static const uint8_t MAX_IMAGE_QUEUE_SIZE = 20;
    mutex imageQueueMutex;
    RawImageQueueType imageQueue;
    /*! Container for logged messages */
    mutex logMutex;
    LogMessageQueueType logs;
};