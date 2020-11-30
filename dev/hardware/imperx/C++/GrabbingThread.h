#pragma once

#include <mutex>
#include <thread>
#include <condition_variable>
#include <zmq.hpp>

//----------------------- forward decloration -----------------------//
namespace IpxCam
{
    class Stream;
}

class IpxDisplay;
//-------------------------------------------------------------------//

class GrabbingThread
{
public:

    GrabbingThread(IpxCam::Stream* stream, IpxDisplay *display);
    ~GrabbingThread(){ Terminate(); }
	
	bool Start();
	bool Stop();
	void Work();
    bool IsStarted(){ return m_bStart; }

protected:
	void Terminate();
	bool GrabNewBuffer(uint64_t grabTimeout); // Acquires new image from Stream and shows it on Display

private:
    bool m_bStart;
	bool m_bStopAck;
	bool m_bTerminate;

	IpxCam::Stream* m_strm;  // Pointer to Stream object, where the image data to be read from. 
	IpxDisplay *m_pDisplay;  // Pointer to Display object, where we show the acquired images

    // for display to remember image params
    uint32_t m_width;
    uint32_t m_height;
    uint32_t m_pixelType;

    // thread stuff
    std::mutex m_mutex;
    std::thread m_pThread;
    std::condition_variable m_cvStart;
    std::condition_variable m_cvStopAck;
};
