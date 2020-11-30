#ifndef _VIDEO_OBJECT_H
#define _VIDEO_OBJECT_H

#include <string>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/opencv.hpp>


#include "VideoRecorder.h"

//######################################################################
// VideoObject is intended for displaying images using OpenCV functions
// in a multi-thread environment, where cv::imshow() and cv::waitKey()
// must be called from the main program or thread.
// VideoObject can be used as follows:
// - create a VideoObject instance (vObj) and pass it to client/thread code
//   or have the client/thread create one;
// - in the client/thread code, call VideoObject::createDisplayFrame()
//   to prepare the image frames to be displayed. This is usually carried
//   out in the worker function of the client in a thread.
// - back in the main, we do the following:
//   while(!VideoObject::isDone())
//   {
//       VideoObject::updateAllDisplayWindows();
//       usleep(10000); //sleep 10 ms --> max 100 fps, adjust when needed
//   }
// - In the client/thread, you can call VideoObject::getUserKey() to check
//   if user has typed some special key (e.q. 'Q'), and react accordingly.
//   When the client is done, call VideoObject::setDone() on the vObj instance.
// - At this time, the while loop in the main thread will quit.
// - Proceed with any regular handling to end program and/or client/thread joining.
//######################################################################
class VideoObject: public VideoRecorder
{
	typedef std::map<std::string, VideoObject*> VOMap;

public:
    VideoObject();
    VideoObject(const std::string dispWinTitle, double dispFrameScale=0.25, 
                bool interactive=false, const std::string videoOutputFile="", bool debug=false);
    ~VideoObject();

    void createDisplayFrame(const cv::Mat &);
    static int getUserKey();
    static bool isDone() {return done;}
    static void setDone();
    void setInteractive(bool val) { interactive = val; }
    bool writeVideoFrame();
    void configure(const std::string dispWinTitle, double dispFrameScale, bool isInteractive,
                   const std::string videoOutputFile="", bool debug=false);
    static int updateAllDisplayWindows();

// Alternative API & usage:
    static void imshow(const std::string& winTitle, const cv::Mat & image);

protected:
    void updateDisplayWindow(double scale=1.0);
    std::string winTitle;
    double dispFrameScale; //scale factor applied to input image to generate display image
    cv::Mat dispFrame;
    bool displayNeedsUpdate, debug;
    std::mutex dispFrameMutex;
    static bool done;
    static int userKey;
    static bool interactive; // stop after displaying an image frame and ask for press any key

private:
    void registerClassInstance(VideoObject *);
    static VOMap vObjMap;
    static std::mutex allVOMutex;
};


#endif
