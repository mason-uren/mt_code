#include "GrabbingThread.h"
#include "IpxCameraApi.h"
#include "IpxImage.h"
#include "IpxDisplay.h"
#include <iostream>
#include <zmq.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lockfree/spsc_queue.hpp>

using namespace std;

boost::lockfree::spsc_queue<IpxImage *> q{100};

#define within(num) (int) ((float) num * random () / (RAND_MAX + 1.0))


void sendData()
{
	void *context = zmq_ctx_new();
	void *publisher = zmq_socket(context, ZMQ_PUB);
	int HWM = 1;
   	zmq_setsockopt(publisher,ZMQ_SNDHWM,&HWM, sizeof(HWM));
   	zmq_setsockopt(publisher,ZMQ_RCVHWM,&HWM, sizeof(HWM));
	int bind = zmq_bind(publisher, "tcp://*:9000");

	//char *arr1 = (char *)malloc ((sizeof(char)) * 5120 * 5120);
	//double balance[] = {1000.0, 2.0, 3.4, 17.0, 50.0};
	static IpxImage * img;

	while (1) {
		if (q.pop(img)){
			//memcpy(arr1,img->imageDataOrigin,sizeof(char) * img->imageSize);
			zmq_send(publisher, img->imageDataOrigin, sizeof(char) * img->imageSize, ZMQ_NOBLOCK);
		}
	}
}


//===================================================================
// GrabbingThread::GrabbingThread():  constructor
//===================================================================
GrabbingThread::GrabbingThread(IpxCam::Stream* stream, IpxDisplay *display)
    : m_bStart(false)
	, m_bStopAck(false)
	, m_bTerminate(false)
    , m_strm(stream)
    , m_pDisplay(display)
    , m_width(0)
    , m_height(0)
    , m_pixelType(0)
{
	// Create the thread object
    m_pThread = std::thread(&GrabbingThread::Work, this); 
}

//===================================================================
// GrabbingThread::Terminate(): Terminates the thread
//===================================================================
void GrabbingThread::Terminate()
{
	// Terminate the grabbing
	m_bTerminate=true;

	// Dummy Start to unfreeze the thread or Stop the Writing
	if( !m_bStart )
		Start();
	else
		Stop();

	// Wait for thread finished
    m_pThread.join();
}

//===================================================================
// GrabbingThread::Work(): Main function of the Grabbing Thread 
//===================================================================
void GrabbingThread::Work()
{ 
	boost::thread sendThread(sendData);
	// Working cycle
	while(true) 
	{
		// Wait the Working Start signal
        {
            std::unique_lock<std::mutex> lk_start(m_mutex);
            m_cvStart.wait(lk_start, [this]{ return m_bStart; });
        }
		
		// Writing cycle, check if we're terminating
		if(!m_bTerminate)
		{
			while(m_bStart) 
			{


				// Grab new buffer
				uint32_t err = GrabNewBuffer(-1);

				if(err)
				{
					// ERROR: 
					// log the error here
				}
			}
		}

		// Notify them, that Working Cycle is stopped
        m_mutex.lock();
        m_bStopAck = true;
        m_mutex.unlock();
		m_cvStopAck.notify_all();
		
		// Quit the working cycle, if we're terminating
		if(m_bTerminate)
			break;
	};
}

//===================================================================
// GrabbingThread::Start(): Starts the video acquisition on the Stream
//===================================================================
bool GrabbingThread::Start()
{
	// Check the thread state
	if( m_bStart )
		return false;

	// Notify the Working Cycle to Start 
    m_mutex.lock();
    m_bStart = true;
    m_bStopAck = false;
    m_mutex.unlock();
    m_cvStart.notify_all();

    return true;
}

//===================================================================
// GrabbingThread::Stop(): Stops the video acquisition
//===================================================================
bool GrabbingThread::Stop()
{
	// Check the thread state
    if( !m_bStart )
		return false;

	// Stop the working cycle
    m_mutex.lock();
    m_bStart = false;
    m_mutex.unlock();
    m_cvStart.notify_all();

	// Cancel I/O for current buffer
    if (m_strm)
        m_strm->CancelBuffer();
	
	// Wait for Stop notification
    {
        std::unique_lock<std::mutex> lk_stop(m_mutex);
        m_cvStopAck.wait(lk_stop, [this]{ return m_bStopAck; });
    }

    return true;
}

//===================================================================
// Acquires the new buffer form the Device and displays using IpxDisplay
//===================================================================
bool GrabbingThread::GrabNewBuffer(uint64_t grabTimeout) 
{
	// Check the thread state
    if (!m_bStart || !m_strm)
		return false;

	// Acquire new buffer
 


	IpxCam::Buffer *pbuff=nullptr; 
	IpxCamErr err = IPX_CAM_ERR_OK;

	pbuff = m_strm->GetBuffer(grabTimeout, &err);
    if((IPX_CAM_ERR_OK == err) && pbuff)
	{
		// Return the incompleted buffer back to stream queue, if specified
        if(pbuff->IsIncomplete())
		{
            m_strm->QueueBuffer(pbuff);
			pbuff=nullptr;
            return false; // ERROR: GetBuffer() failed
		}

		// Get the acquired image and display it
        if(m_pDisplay)
		{
			IpxImage *img = pbuff->GetImage();

            // check if SetVideoMode needs to be called
            if (img->width != m_width || img->height != m_height
                || img->pixelTypeDescr.pixelType != m_pixelType)
            {
                m_width = img->width;
                m_height = img->height;
                m_pixelType = img->pixelTypeDescr.pixelType;
                m_pDisplay->SetVideoMode(img);
            }
			q.push(img);
			m_pDisplay->DisplayVideo(img);
		}

		// Return the buffer back to stream queue
        if(pbuff)
			m_strm->QueueBuffer(pbuff);

        pbuff = nullptr;
	}

	return true;
}
