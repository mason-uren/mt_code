#ifndef _VIDEO_RECORDER_H
#define _VIDEO_RECORDER_H

#include <string>
#include <thread>
#include <condition_variable>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

class VideoRecorder
{
public:
    VideoRecorder(const std::string & videoOut=std::string("video_recorder_output.avi"), bool debug=false);
    virtual ~VideoRecorder();
    bool writeVideoFrame(const cv::Mat &);
    void configure(const std::string & videoOut=std::string("video_recorder_output.avi"), bool debug=false);
   
protected:

private:
    void recordingThread();
 
    cv::VideoWriter videoOut;
    std::string videoOutputFile;
    bool doneRecording, debug;
    std::thread thread_hdl;
    std::vector<cv::Mat> frameBufferFIFO;
    std::mutex fifoMtx;
    std::condition_variable fifoCv;
    int frame_count;
};
#endif
