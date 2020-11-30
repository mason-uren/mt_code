#include "VideoObject.h"
#include "Shared/OSDefines.h"

//Initialize the static member
int VideoObject::userKey = -1;
bool VideoObject::done = false;
bool VideoObject::interactive = false;
VideoObject::VOMap VideoObject::vObjMap{};
std::mutex VideoObject::allVOMutex;

void VideoObject::registerClassInstance(VideoObject * vobj)
{
    std::lock_guard<std::mutex> guard(allVOMutex);
    std::string title=vobj->winTitle;
    bool changed=false;
    while(vObjMap.count(title)>0) // winTitle has been used already
    {
        if(debug) std::cout << "VideoObject:: Warning: VideoObject exist with the same winTitle: "<< title << std::endl;
        title +=".";
        changed = true;
    }
    if(changed)
    {
        if(debug) std::cout << "VideoObject:: changed winTitle to: "<< title << std::endl;
        vobj->winTitle = title;
    }
    vObjMap[vobj->winTitle] = vobj;
    if(debug)
        std::cout << "VideoObject:: Registering instance with winTitle = '"<<vobj->winTitle <<"'" << std::endl;
}

VideoObject::VideoObject() :
    winTitle("VideoObject"),dispFrameScale(1.0),
    displayNeedsUpdate(false)
{
    registerClassInstance(this);
}

VideoObject::VideoObject(const std::string dispWinTitle, double dispFrameScale,
            bool interactive, const std::string videoOutputFile, bool debug) :
    VideoRecorder(videoOutputFile, debug), winTitle(dispWinTitle),
    dispFrameScale(dispFrameScale), displayNeedsUpdate(false), debug(debug)
{
    registerClassInstance(this);
    VideoObject::interactive = VideoObject::interactive || interactive; // once interactive, forever interactive :-)
}

VideoObject::~VideoObject()
{

    std::lock_guard<std::mutex> guard(allVOMutex);
    if (vObjMap.erase(this->winTitle)<=0)
    {
        std::cout << "VideoObject: WARNING: object not found ("<< this->winTitle<< ") in registry while attempting to delete it on destruction." << std::endl;
    }

}

void VideoObject::configure(const std::string dispWinTitle, double dispScale, bool interactiveMode, 
                            const std::string videoOutputFile, bool debug)
{
    winTitle = dispWinTitle;
    dispFrameScale = dispScale;
    interactive = interactiveMode;
	displayNeedsUpdate = false;
    if(!videoOutputFile.empty())
        VideoRecorder::configure(videoOutputFile, debug);

	//  register
}

bool VideoObject::writeVideoFrame()
{
    return VideoRecorder::writeVideoFrame(dispFrame);
}

void VideoObject::createDisplayFrame(const cv::Mat & image)
{
    // get exclusive access to dispFrame
    std::lock_guard<std::mutex> guard(dispFrameMutex);

    if (dispFrameScale==1.0)
        dispFrame = image.clone();
    else
        cv::resize(image, dispFrame, cv::Size(), dispFrameScale, dispFrameScale, cv::INTER_LINEAR);  // downsample the frame to give a reasonably-sized video
    displayNeedsUpdate = true;
}


// Display the dispFrame
// This function is no longer public, and should not be called from an instance of VideoObject
// because it no longer calls cv::waitKey().  Call the class function updateAllDisplayWindows() instead.
// The following argument will be deprecated soon.
// scale: default=1.0, scale applied to dispFrame (this scale does not affect the saved video
//        and is good when working remotely and you want to scale the dispFrame down further)
void VideoObject::updateDisplayWindow(double scale)
{
    if (!displayNeedsUpdate) //no (new) image to show yet
    {
        //std::cout << "VideoObject::updateDisplayWindow() - no need to update" << std::endl;
        return;
    }
    else
    {
        //std::cout << "VideoObject::updateDisplayWindow() - update needed" << std::endl;

        // get exclusive access to dispFrame
        std::lock_guard<std::mutex> guard(dispFrameMutex);

        // resize to gain some speed for remote session for testing
        if (scale != 1.0)
        {
            cv::Mat dispFrame2;
            cv::resize(dispFrame, dispFrame2, cv::Size(), scale, scale, cv::INTER_LANCZOS4);
            cv::imshow(winTitle, dispFrame2);
        }
        else 
            cv::imshow(winTitle, dispFrame);
        // turn off the flag since we just updated the display window
        displayNeedsUpdate = false;
    }
}

int VideoObject::updateAllDisplayWindows()
{
    static int userKeyCountDown{0};
    {
        std::lock_guard<std::mutex> guard(allVOMutex);
		VideoObject::VOMap::iterator iter=vObjMap.begin();
        for (; iter != vObjMap.end(); iter++)
        {
            (iter->second)->updateDisplayWindow();
        }
    }

    if(interactive)
    {
        if(!done)
        {
            std::cout << "Press any key in the image window to continue, Q to quit ... " << std::endl;
            userKey=cv::waitKey(1000); // wait for 1 sec., not infinitely
        }
    }
    else
    {
        int key = cv::waitKey(1);      // minimum wait between frames (only for display purposes)

        // Let userKey persist for 50 iterations of this updateAllDisplayWindow() call, so that clients
        // that has longer display cycles can still catch userKey before it is overwritten by the
        // next waitKey() value

        //std::cout << "key, userKey, count=" <<key <<", " << userKey <<", "<< userKeyCountDown<< std::endl;

        if( key>0 && userKey < 0 )
        {
            if(userKeyCountDown<=0 ) // user input detected & not in count-down
            {
                userKey = key;
                userKeyCountDown = 50;
                //std::cout << "count-down STARTED -----------------------" << std::endl;
            }

        }
        // If we are in count-down and reached 0, then update userKey (whatever key is)
        if(userKeyCountDown<=0 || --userKeyCountDown==0)
        {
            userKey = key;
            //std::cout << "count-down ENDED -----------------------" << std::endl;
        }

    }
    return userKey;

}

// Force to wait for the user's input
int VideoObject::getUserKey()
{
    if(interactive)
    {
        while(userKey<=0)
            Sleep(20);//ms
    }
    return userKey;
}

void VideoObject::setDone()
{
    // This function may need to be reimplemented after we made
    // the main-thread API calls static functions.

    int timeOut = 150; //time out in 3000ms=3sec
    // Wait for someone to call VideoObject::updateDisplayWindow
    // so the pending image can be displayed before declaring "done".
    //while(displayNeedsUpdate && (timeOut-- > 0 ))
    //    Sleep(20);//ms
    done = true;
}

void VideoObject::imshow(const std::string& winTitle, const cv::Mat & image)
{
    VideoObject * vobj;
    if(vObjMap.count(winTitle)>0)
    {
        vobj = vObjMap[winTitle];
    }
    else
    {
        vobj = new VideoObject(winTitle, 1.0); //FIXME: memory leak
    }
    vobj->createDisplayFrame(image);
}
