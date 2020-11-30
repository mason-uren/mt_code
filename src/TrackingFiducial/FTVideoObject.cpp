#include "FTVideoObject.h"

void FTVideoObject::configure(const std::string dispWinTitle, double dispScale, bool showPose, 
                              bool interactive, const std::string videoOutputFile, bool debug)
{
    showPoseEstimates = showPose;
    VideoObject::configure(dispWinTitle, dispScale, interactive, videoOutputFile, debug);
}

// Input:
//   image: either color or gray-scale image frame
//   frameId: 0-indexed frame id
//   sixdof: fiducial pose in SixDOF structure
//   showGreenDot: if true, put a greed dot in the image frame at location "dotCenter"
//   dotCenter: location to show the green dot if showGreenDot is true;
//              dotCenter is the location in original image before resizing by dispFrameScale
void FTVideoObject::createDisplayFrame(const cv::Mat & image, size_t frameId, 
                                     const SixDOF & sixdof, bool showGreenDot, cv::Point dotCenter, const double & dotScale)
{
    // get exclusive access to dispFrame
    std::lock_guard<std::mutex> guard(dispFrameMutex);

    cv::resize(image, dispFrame, cv::Size(), dispFrameScale, dispFrameScale, cv::INTER_LINEAR);  // downsample the frame to give a reasonably-sized video
    if (dispFrame.type() == CV_8UC1)
        cv::cvtColor(dispFrame, dispFrame, cv::COLOR_GRAY2BGR);
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    char label[128];
    int spacing = 35, startpos = 60;
    int start_col = dispFrame.cols - 250;

    if (showPoseEstimates)
    {
        // Draw a filled rectangle to obscure the old scores and write the new ones on top
        cv::rectangle(dispFrame, cv::Point(start_col-45, 27), cv::Point(dispFrame.cols-30, 250), cv::Scalar(80, 80, 80), -1);

        /*
        // This is for ChArUco testing - show the number of ChArUco squares that the algorithm detected
        snprintf(label, sizeof(label), "Frame: %d", frame_num);
        cv::putText(frame, label, cv::Point(730, startpos), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);

        snprintf(label, sizeof(label), "ChArUco: %d", charuco_sq_found);
        cv::putText(frame, label, cv::Point(730, startpos + spacing * 1), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);
        */

        snprintf(label, sizeof(label), "X: %6.3f", sixdof.x);
        cv::putText(dispFrame, label, cv::Point(start_col, startpos), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);      // x position

        snprintf(label, sizeof(label), "Y: %6.3f", sixdof.y);
        cv::putText(dispFrame, label, cv::Point(start_col, startpos + spacing * 1), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);      // y position

        snprintf(label, sizeof(label), "Z: %6.3f", sixdof.z);
        cv::putText(dispFrame, label, cv::Point(start_col, startpos + spacing * 2), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);     // z position

        snprintf(label, sizeof(label), "Pitch: %8.3f", sixdof.pitch);
        cv::putText(dispFrame, label, cv::Point(start_col-40, startpos + spacing * 3), fontFace, fontScale, cv::Scalar(0, 255, 0), thickness);     // pitch (rx)

        snprintf(label, sizeof(label), " Yaw: %8.3f", sixdof.yaw);
        cv::putText(dispFrame, label, cv::Point(start_col-40, startpos + spacing * 4), fontFace, fontScale, cv::Scalar(0, 255, 0), thickness);     // yaw (ry)

        snprintf(label, sizeof(label), " Roll: %8.3f", sixdof.roll);
        cv::putText(dispFrame, label, cv::Point(start_col-40, startpos + spacing * 5), fontFace, fontScale, cv::Scalar(0, 255, 0), thickness);     // roll (rz)
    }

    snprintf(label, sizeof(label), "Pan: %5.1f", sixdof.pan);
    cv::putText(dispFrame, label, cv::Point(30,  50), fontFace, fontScale, cv::Scalar(200, 200, 64), thickness);     // frame id

    snprintf(label, sizeof(label), "Tilt:  %5.1f", sixdof.tilt);
    cv::putText(dispFrame, label, cv::Point(30, 50 + spacing ), fontFace, fontScale, cv::Scalar(200, 200, 64), thickness);     // frame id

    snprintf(label, sizeof(label), "Frame: %5zu", frameId);
    cv::putText(dispFrame, label, cv::Point(30, dispFrame.rows - 50), fontFace, fontScale, cv::Scalar(0, 200, 128), thickness);     // frame id

    // Draw a green disc to signal that a new pose is successfully estimated
    if(showGreenDot)
        cv::circle(dispFrame, dotCenter*dispFrameScale, 20 * dotScale, cv::Scalar(0, 200,0), -1);
    displayNeedsUpdate = true;
}
