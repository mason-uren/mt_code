#ifndef __FiducialTracker_h
#define __FiducialTracker_h

#include <fstream>
#include <thread>
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// HRL
#include <Shared/OSDefines.h>
#include <Shared/ModelsConfig.h>

// Utilities
#include "ThreadInterface/ThreadInterface.h"

// Interface
#include "Threads/Ximea/XimeaThread.h"
#include "Threads/ImperX/ImperxThread.h"
#include "Threads/PanTilt/PanTiltThread.h"

// Models
#include "Camera/CameraModel.h"
#include "Board/FiducialModel.h"

// Pose Estimator

#include "PoseEstimator.h"
#include "FusedPoses/FusedPoses.h"

#include "ConfigParser/TrackingJSONParser.h"
#include "FTVideoObject.h"

class CSVRecorder
{
public:
    CSVRecorder(const std::string outputFile):
        outputFile(outputFile) {}

    ~CSVRecorder()
    {
        if (file_stream.is_open())
            file_stream.close();
    }
    void write(const std::string & str)
    {
        file_stream << str;
    }
    bool open()
    {
        if( !file_stream.is_open() && !outputFile.empty() )
        { // open on first write:
            file_stream.open(outputFile, std::ios_base::out);
        }
        return this->is_open();
    }
    bool is_open()
    {
        return file_stream.is_open();
    }

private:
    std::ofstream file_stream;
    std::string outputFile;
};

class PoseRecorder: public CSVRecorder
{
public:
    PoseRecorder(const std::string posePathname="./tracked_fiducial_pose.txt")
    : CSVRecorder(posePathname)
    {}

    void write(size_t frameId, const SixDOF & sixdof)
    {
        std::stringstream strbuf;
        if( !this->is_open() )
        { // open on first write:
            this->open();
        }
        if(is_open()) //only attempt to write when the file is open; ignore errors
        {
            strbuf.clear();
            strbuf << frameId << " " << sixdof.x << " " << sixdof.y << " " << sixdof.z
                   << " " << sixdof.pitch << " " << sixdof.yaw << " " << sixdof.roll << " " << sixdof.reprojection_error << std::endl;
            CSVRecorder::write(strbuf.str());
        }
    }
};


class PanTiltRecorder: public CSVRecorder
{
public:
    PanTiltRecorder(const std::string ptFilePathname="./pantilt_recorded.txt")
    : CSVRecorder(ptFilePathname)
    {}

    void write(size_t frameId, double pan, double tilt)
    {
        if( !is_open()  )
        { // open on first write:
            open();
        }
        if(is_open()) //only attempt to write when the file is open; ignore errors
        {
            std::stringstream strbuf;
            strbuf << frameId << " " << pan << " " << tilt << std::endl;
            CSVRecorder::write(strbuf.str());
        }
    }
};

class PanTiltReader
{
public:
    PanTiltReader() {}
    ~PanTiltReader()
    {
        if (file_stream.is_open())
            file_stream.close();
    }
    bool open(const std::string & filePath)
    {
        pantiltFile = filePath;
        bool ret=true;
        if(!pantiltFile.empty())
        {
            file_stream.open(pantiltFile, std::ios_base::in);
            if(!file_stream.good())
            {
                std::cout <<"PanTileReader: Warning: error opening file for read:" <<pantiltFile << std::endl;
                ret = false;
            }
        }
        else
        {
            ret = false;
        }
        return ret;
    }
    bool readNext(size_t *frameId, double *pan, double *tilt)
    {
        if( file_stream.is_open() && file_stream.good())
        {
            file_stream >> *frameId >> *pan >> *tilt;
            return true;
        }
        else
        {
            return false;
        }
    }
    bool good()
    {
        return file_stream.good();
    }
private:
    std::ifstream file_stream;
    std::string pantiltFile;
};

class FiducialTracker: public ThreadInterface
{
public:
    FiducialTracker(TrackingJSONParser * parser,
                    const CameraModel<double> & intrinsics,
                    const FiducialModel & fiducialModel,
                    bool altBoard, const std::string & name = CLASS);
    FiducialTracker(const std::string & cadModelJsonPath,
					const CameraModel<double> & intrinsics,
					const FiducialModel & fiducialModel,
                    bool debug=false,
                    double markerDetectionScale=0.2,
                    bool showResults=true,
                    bool recordResults=false,
                    bool recordInput=false,
                    const std::string & name = CLASS);
    ~FiducialTracker() override = default;

    bool setup(void * params = nullptr) override;
    void run() override;

    bool set_video_input(const std::string & path);
    bool set_image_input(const std::vector<std::string> & paths);
    bool set_pantilt_reader_input(const std::string & path);
    bool configure_interface_devices(DeviceInterface * cameraThread, DeviceInterface * panTiltThread = nullptr);

    // Static
    static bool sourceInstances(TrackingJSONParser * parser, 
								const FiducialModel & fiducialModel, 
								const bool hasWFOV = true);
    static bool startInstances();
    static void allWait();
    static auto numberOfInstances() -> decltype(int{}) {
        return static_cast<int>(instances.size());
    }

protected:
    // Functions
    bool cleanAndExit() override;
    void print_parameters();

    // Variables
    std::shared_ptr<FTVideoObject> videoDispObj{};

private:
    // Functions
    bool    visitStagingArea();
    bool    isLive();
    void	setRunningMode(const std::string & mode = std::string{});

    // Variables
    bool        done, liveDemo, doDisplay, writeVideoFile;
    bool        bShowEstimatedPose, altCharucoBoard, recordInput, debug;
    bool        videoMode; //for liveDemo=false only
    double      dispScale; // = 0.125; display Ximea image at 1/8 of the original size in both x & y dimensions
    double      scale4MarkerDetection; //to be passed on to PoseEstimator(World)
    double      angleThresholdDeg; // threshold to determine if pan/tilt change is needed to re-center the fiducial in image

    // depending on videoMode true/false, one of the following will be used
    cv::VideoCapture videoCapture;
    std::vector<std::string> inputImagePaths;
    std::vector<std::string>::const_iterator inputImagePathsIter;

    std::string trackingVideoOutputFile;

    PanTiltReader ptReader;
    std::shared_ptr<VideoRecorder> input_video_recorder{};
    std::shared_ptr<PanTiltRecorder> pantilt_recorder{};
    std::shared_ptr<PoseRecorder> pose_recorder{};

    std::string jsonModelFile;
    CameraModel<double> cameraModel;
    FiducialModel fiducialModel{};
//    ,
//    cameraMatrixPath, cameraDistCoeffsPath;

    std::shared_ptr<DeviceInterface> cameraDevice{};
    std::shared_ptr<DeviceInterface> panTiltDevice{};
    std::shared_ptr<PoseEstimator> poseEstimator{};

    SixDOF sixdof;
    std::string ID{};

    // Static declarations
    static const std::string CLASS;
    static std::string UUID() {
        static int uuidGenerator{};
        return std::to_string(uuidGenerator++);
    }
    static std::unordered_map<std::string, FiducialTracker *> instances;

    // NOTE: singletons (don't mix with shared_ptr)
    FusedPoses * fusedPoseDistributor;
};

#endif // __FiducialTracker_h
