#include <iostream>
#include "VideoRecorder.h"

VideoRecorder::VideoRecorder(const std::string &videoOut, bool debug) :
    videoOutputFile(videoOut), doneRecording(false), frame_count(0), debug(debug)
{

}

VideoRecorder::~VideoRecorder()
{
    if (thread_hdl.joinable())
    {
        doneRecording = true;
        fifoCv.notify_all(); // signal to  processes waiting on fifoCv
        thread_hdl.join();
    }
    
    if (videoOut.isOpened())
    {
        videoOut.release();
        if(debug)
        {
            std::cout << "VideoRecorder: Video file written to " << videoOutputFile << std::endl;
            std::cout << "VideoRecorder: total frames written: "<< frame_count << std::endl;
        }
    }
}

void VideoRecorder::configure(const std::string & videoOutFile, bool Debug)
{
    videoOutputFile=videoOutFile;
    debug = Debug;
}

void VideoRecorder::recordingThread()
{
    while(!doneRecording)
    {
        std::unique_lock<std::mutex> lock(fifoMtx);
        fifoCv.wait(lock, [this] { return !frameBufferFIFO.empty() || doneRecording;}  );
        // now we have lock on fifoMtx. and exclusive write access to frameBufferFIFO:
        if(doneRecording) // flush the buffer and exit
        {
            while(!frameBufferFIFO.empty())
            {
                videoOut.write(frameBufferFIFO.front());
                frameBufferFIFO.erase(frameBufferFIFO.begin()); //pop it off
                frame_count++;
            }
        }
        else // dump one frame and continue the loop
        {
            videoOut.write(frameBufferFIFO.front());
            frameBufferFIFO.erase(frameBufferFIFO.begin()); //pop it off
            frame_count++;
        }
    }
}

bool VideoRecorder::writeVideoFrame(const cv::Mat & frame)
{
    // Start the recording thread the first time this function is called
    if (!videoOut.isOpened() && !videoOutputFile.empty())
    {
        bool color_video;
        if (frame.channels()==3)
            color_video = true;
        else if (frame.channels()==1)
            color_video = false;
        else // cannot handle
        {
            std::cerr << "VideoRecorder::writeVideoFrame: Error: image frame must contain either 1 or 3 channels." << std::endl;
            return false;
        }

        videoOut.open(videoOutputFile, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,//fps
                      cv::Size(frame.cols, frame.rows), color_video);
        if (!videoOut.isOpened() )
        {
            std::cerr <<"VideoRecorder: Error opening video output file for write: " << videoOutputFile << std::endl;
            return false;
        }
        if(debug) std::cout << "VideoRecorder: Video output file opened for write: " << videoOutputFile << std::endl;
        
        // start video recording thread:
        thread_hdl = std::thread(&VideoRecorder::recordingThread, this);
        
        if(debug) std::cout <<"VideoRecorder: recorder thread started (id: "<< thread_hdl.get_id() << ")"<< std::endl;
    }
    // For successive calls, push the display frame to the frame buffer FIFO
    if (videoOut.isOpened())
    {
        { //fifo mutex block:
            std::lock_guard<std::mutex> lock(fifoMtx);
            frameBufferFIFO.push_back(frame);
        }
        fifoCv.notify_all(); // signal to  processes waiting on fifoCv
    }
    
    return true;
}
