#ifndef _FT_VIDEO_OBJECT_H
#define _FT_VIDEO_OBJECT_H

#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "SixDOF.h"
#include "OpenCVHelpers/Video/VideoObject.h"

// VideoObject for fiducial tracker
class FTVideoObject: public VideoObject
{
public:
    FTVideoObject() {};
    FTVideoObject(const std::string & dispWinTitle, double frameBufferScale=0.25, 
                  bool showPoseEstimates=true, bool interactive=false, 
                  std::string videoOut="", bool debug=false) :
        showPoseEstimates(showPoseEstimates),
        VideoObject(dispWinTitle, frameBufferScale, interactive, videoOut, debug)
    {}
    void createDisplayFrame(const cv::Mat &, size_t frameId, 
                            const SixDOF & sixdof, bool showGreenDot, cv::Point dotCenter, const double & dotScale = 1);
    void configure(const std::string dispWinTitle, double dispFrameScale, bool showPoseEstimates, 
                   bool isInteractive, const std::string videoOutputFile="", bool debug=false);

private:
    bool showPoseEstimates;
};

#endif
