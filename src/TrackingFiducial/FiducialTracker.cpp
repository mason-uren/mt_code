#include "FiducialTracker.h"

using namespace std;

// Static Variable Initialization
std::unordered_map<std::string, FiducialTracker *> FiducialTracker::instances{};
const std::string FiducialTracker::CLASS{ "FiducialTracker" };

// Static Function Defs
bool FiducialTracker::sourceInstances(TrackingJSONParser * parser, const FiducialModel & fiducialModel, const bool hasWFOV) {
    // Load Config
    if (!parser->loadTrackingConfig()) {
		ErrorHandler::getInstance()->report("Failed to load tracking config!", Shared::Error::Severity::KILL_AND_EXIT);
        return false;
    }

    auto pairs{ DeviceInterface::pairs };
    if (false) { // hasWFOV) {
        pairs.insert({
            Presets::Tracking::Context::WORLD_FRAME,
            System::Pair{DeviceInterface::wFrame, "", ""}
        });
    }

    // Get pair
    for (auto & pair : pairs) { // DeviceInterface::pairs) {
        try {
            auto intrinsics{ (pair.first != Presets::Tracking::Context::WORLD_FRAME) ? DeviceInterface::ximea.at(pair.second.camera.intrinsics) : DeviceInterface::imperx.at(pair.second.camera.intrinsics) };
            FiducialTracker::instances.insert({ pair.first, new FiducialTracker(parser, intrinsics, fiducialModel, false) });
        }
        catch (const std::bad_alloc & error) {
            ErrorHandler::getInstance()->report(
                error.what(),
                Shared::Error::Severity::WARN
            );
        }

        // Sanity check!
        if (FiducialTracker::instances.find(pair.first) == FiducialTracker::instances.end()) {
            ErrorHandler::getInstance()->report(
                "Tracker instance failed to initialize. Expect exit...",
                Shared::Error::Severity::KILL_AND_EXIT
            );
            return false;
        }

        // Assign ID
        FiducialTracker::instances.at(pair.first)->ID = pair.first;
    }

    return true;
}

bool FiducialTracker::startInstances() {
    for (auto & instance : FiducialTracker::instances) {
        auto trackerPtr{const_cast<std::string *>(&instance.first)};
        if (!instance.second->start(static_cast<void *>(trackerPtr))) {
            instance.second->eHandler->report(
                instance.second->threadID() + ": Failed to start tracking instance.",
                Shared::Error::Severity::KILL_AND_EXIT
            );
            return false;
        }
    }
    return ErrorHandler::getInstance()->shouldContinue();
}

void FiducialTracker::allWait() {
    for (const auto & instance : FiducialTracker::instances) {
        instance.second->wait();
    }
}

// END Static functions

//=========================================================================
FiducialTracker::FiducialTracker(TrackingJSONParser * parser,
                                 const CameraModel<double> & intrinsics,
                                 const FiducialModel & fiducialModel,
                                 bool altBoard, const std::string & name) :
    ThreadInterface(name + "[ " + UUID() + "]"),
    videoDispObj(nullptr),
    done(false),
    liveDemo(false),
    doDisplay(parser->display.visible),
    writeVideoFile(parser->output.record),
    bShowEstimatedPose(true),
    altCharucoBoard(altBoard),
    recordInput(false),
    debug(false),
    videoMode(false),
    dispScale(parser->display.scale),
    scale4MarkerDetection(0.2),
    angleThresholdDeg(0.5),
    trackingVideoOutputFile(""),
    jsonModelFile(""),
    cameraModel(intrinsics),
    fiducialModel(fiducialModel),
    fusedPoseDistributor(FusedPoses::getInstance())
{
    ThreadInterface::loadConfig(parser);

    // Create separate logging instance
    logger->addInstance(ConfigInterface::name);
    eHandler->report("Welcome to tracking instance ID: " + ConfigInterface::name, Shared::Error::INFO, ConfigInterface::name);

    setRunningMode(); // parser->mode);

    // Output video file
    if (writeVideoFile) {
        trackingVideoOutputFile = "./" + ThreadInterface::name + "_liveDemo_output.avi";
		//trackingVideoOutputFile = 	ThreadInterface::Config<TrackingJSONParser>()->output.video.directory + FILE_SEP +
		//	ThreadInterface::name + ThreadInterface::Config<TrackingJSONParser>()->output.video.file;
    }

    // Input video file
    if (recordInput) {
        input_video_recorder = std::make_shared<VideoRecorder>(ConfigInterface::name + "_liveDemo_input.avi"); //  parser->input.video.file);
        pantilt_recorder = std::make_shared<PanTiltRecorder>(ConfigInterface::name + "_pantilt_input.txt");
    }

    // Pose Recorder
    if (liveDemo || videoMode) {
        pose_recorder = std::make_shared<PoseRecorder>(ConfigInterface::name + "_charuco_poses.txt");
    }
}

FiducialTracker::FiducialTracker(const std::string & cadModelJsonPath,
                                const CameraModel<double> & intrinsics,
                                const FiducialModel & fiducialModel,
                                bool debug,
                                double markerDetectionScale,
                                bool showResults,
                                bool recordResults,
                                bool recordInput,
                                const std::string & name) :
    ThreadInterface(name + "_" + UUID()),
    videoDispObj(nullptr), done(false), liveDemo(isLive()), doDisplay(showResults),
    writeVideoFile(recordResults), bShowEstimatedPose(true),
    // altCharucoBoard(altBoard),
    recordInput(recordInput), debug(debug),
    videoMode(false), dispScale(0.125),
    scale4MarkerDetection(markerDetectionScale),angleThresholdDeg(0.5),
    trackingVideoOutputFile(""), jsonModelFile(cadModelJsonPath),
    cameraModel(intrinsics),
    fiducialModel(fiducialModel),
    fusedPoseDistributor(FusedPoses::getInstance())
{
    ThreadInterface::loadConfig<TrackingJSONParser>();

    // Create separate logging instance
    logger->addInstance(ConfigInterface::name);
    eHandler->report("Welcome to tracking instance ID: " + ConfigInterface::name, Shared::Error::INFO, ConfigInterface::name);

    // Output video file
    if(writeVideoFile)
        trackingVideoOutputFile = "./" + ConfigInterface::name + "_liveDemo_output.avi";

    // Input video file
    if (recordInput) {
        input_video_recorder = std::make_shared<VideoRecorder>(ConfigInterface::name + "_liveDemo_input.avi");
        pantilt_recorder = std::make_shared<PanTiltRecorder>(ConfigInterface::name + "_pantilt_input.txt");
    }

}

// List all class parameters for debugging
void FiducialTracker::print_parameters()
{
    std::cout << "Parameters for "<< this->getName() << ":"<<std::endl;
    std::cout << "  Is live demo: "<< liveDemo << std::endl;
    std::cout << "  Display results: "<< doDisplay << std::endl;
    std::cout << "  Save results video: "<< writeVideoFile << std::endl;
    std::cout << "  Use alt. ChArUco board: "<< altCharucoBoard << std::endl;
    std::cout << "  Record input video: "<< recordInput << std::endl;
    std::cout << "  Video disp scale: "<< dispScale << std::endl;
    std::cout << "  Marker detection scale: "<< scale4MarkerDetection << std::endl;
    std::cout << "  Pan/tilt angle threshold (deg): "<< angleThresholdDeg << std::endl;
}

bool FiducialTracker::isLive() {
#ifdef WIN32
    return true;
#else
    return false;
#endif
}

void FiducialTracker::setRunningMode(const std::string & mode) {
    // Lambda functions
    auto boolToStr = [](const bool val) {
        return val ? "true" : "false";
    };
    // END lambda

    if (this->isLive()) {
        this->liveDemo = true;
        this->videoMode = false;
    }
    else if (mode == Presets::Tracking::Mode::VIDEO_STREAM) {
        this->liveDemo = false;
        this->videoMode = true;
    }
    else {
        this->liveDemo = false;
        this->videoMode = false;
    }

    this->eHandler->report(
        this->name + ": Tracking-Mode => Live (" + boolToStr(this->liveDemo) + "), " +
                        "VideoMode (" + boolToStr(this->videoMode) + "), " +
                        "Images (" + boolToStr(!(this->liveDemo && this->videoMode)) + ")",
        Shared::Error::INFO,
        this->name
    );
}

bool FiducialTracker::configure_interface_devices(DeviceInterface * ximeaThread, DeviceInterface * panTiltThread) {
    this->cameraDevice = std::shared_ptr<DeviceInterface>(ximeaThread);
    this->panTiltDevice = std::shared_ptr<DeviceInterface>(panTiltThread);

    if (debug)
        std::cout << "Interface devices configuration done." << std::endl;
    return true;
}

bool FiducialTracker::setup(void * params) {

    // Setup visualizer and video recorder
    this->videoDispObj = std::shared_ptr<FTVideoObject>(new FTVideoObject(this->name, dispScale, bShowEstimatedPose,
                                                                          (!liveDemo)&&(!videoMode), trackingVideoOutputFile, this->debug));
    // Moved here from the constructors because we won't know the true state of videoMode
    // until we run the tracker in stand-alone mode
    if (liveDemo || videoMode) {
        pose_recorder = std::make_shared<PoseRecorder>(getName() + "_charuco_poses.txt");
    }
    this->eHandler->report(
        this->name + ": FiducialTracker: Display Object initialized",
        Shared::Error::Severity::DEBUG, this->getName()
    );

    auto cameraID{ std::string{} };
    auto ptuID{ std::string{} };
    double inputImageScale{ 1 };
    /**
     * Cases:
     * 1) No JSON
     * 2) Default JSON
     */
    if (!params) {
        cameraID = Presets::Tracking::DefaultDeviceKeys::XIMEA_ID;
        ptuID = Presets::Tracking::DefaultDeviceKeys::PTU_ID;
        if (jsonModelFile.empty())
        {
            this->poseEstimator = std::make_shared<PoseEstimator>(
                this->cameraModel, this->fiducialModel,
                this->debug, this->scale4MarkerDetection, inputImageScale, this->getName());
                if(debug) std::cout <<"FiducialTracker: PoseEstimator initialized" << std::endl;
        }
        else
        {
            this->poseEstimator = std::make_shared<PoseEstimatorWorld>(
                this->jsonModelFile, this->cameraModel, this->fiducialModel,
                this->debug, this->scale4MarkerDetection, inputImageScale, this->getName());
                if(debug) std::cout <<"FiducialTracker: PoseEstimatorWorld initialized" << std::endl;
        }

        // NOTE: uncomment (below) and comment (above) - Imperx instance runs without PTU
        //imperxID = Presets::Tracking::DefaultDeviceKeys::IMPERX_ID;
    }
    else {
        auto key{ std::string{*static_cast<std::string *>(params)} };
        System::Pair pair{ key == Presets::Tracking::Context::WORLD_FRAME  ?
            System::Pair{ DeviceInterface::wFrame, "", "" } :
            DeviceInterface::pairs.at(key)
        };

        switch (DeviceInterface::components.at(pair.camera.id)->getType()) {
            case Presets::Device::Type::XIMEA: {
				
                this->jsonModelFile = static_cast<TrackingJSONParser *>(this->config)->models.at(pair.extrinsics).path;
                this->scale4MarkerDetection = static_cast<TrackingJSONParser *>(this->config)->markerDetectionScales.at(Presets::Tracking::Context::XIMEA_FRAME);
                this->poseEstimator = std::make_shared<PoseEstimatorWorld>(
                    this->jsonModelFile,
                    //this->cameraMatrixPath, this->cameraDistCoeffsPath,
                    this->cameraModel, this->fiducialModel,
                    this->debug, this->scale4MarkerDetection, inputImageScale, this->getName());
                this->poseEstimator->set_board_detection_threshold(35);
                break;
            }
            case Presets::Device::Type::IMPERX: {
                this->scale4MarkerDetection = static_cast<TrackingJSONParser *>(this->config)->markerDetectionScales.at(Presets::Tracking::Context::IMPERX_FRAME);
                this->poseEstimator = std::make_shared<PoseEstimator>(
//                    this->cameraMatrixPath, this->cameraDistCoeffsPath,
                    this->cameraModel, this->fiducialModel,
                    this->debug, this->scale4MarkerDetection, inputImageScale, this->getName());
				this->poseEstimator->set_board_detection_threshold(35);
                break;
            }
            default:
                this->eHandler->report(
                    "Unknown camera type <" + std::to_string(static_cast<int>(DeviceInterface::components.at(pair.camera.id)->getType())) + ">",
                    Shared::Error::Severity::KILL_AND_EXIT,
                    this->getName()
                );
                break;
        }

        cameraID = pair.camera.id;
        ptuID = pair.ptu;
    }

    auto context{FusedPoses::Context()};
    if (!this->liveDemo) {
        // If not running live...
    }
    else {
        try {
            auto result{ ptuID.empty() ?
                this->configure_interface_devices(DeviceInterface::components.at(cameraID)) :
                this->configure_interface_devices(DeviceInterface::components.at(cameraID),
                                                  DeviceInterface::components.at(ptuID))
            };
            if (!result) {
                return false;
            }
        }
        catch (const std::out_of_range & error) {
            this->eHandler->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT);
            return false;
        }

        context = FusedPoses::Context(
                this->cameraDevice->getType(), // Camera Type (ximea, imperx, etc)
                SixDOF{},					   // SixDOF
                this->getName(),			   // Logger ID - instance
                false						   // Has fiducial position been set?
        );
    }

    // Add instance to be tracked by FusedPoses shared object
    this->fusedPoseDistributor->addInstance(this->getName(), context);

    // Visit staging area (Will only attempt in Windows OS)
    if (!this->visitStagingArea()) {
        this->eHandler->report(
            "Failed to send <" + this->panTiltDevice->getName() + "> to staging position.",
            Shared::Error::Severity::KILL_AND_EXIT,
            this->getName()
        );
        return false;
    }

    return true;
}

// either of the following two function calls will cause the liveDemo to turn off
bool FiducialTracker::set_video_input(const std::string & path)
{
    liveDemo = false;
    videoMode = true;
    // caller to prepare to catch cv::Exception
    bool ret = videoCapture.open(path);

    // turn off interactive mode of VideoObject when input is video (interactive would be true when liveDemo is false)
    //if(videoDispObj!=nullptr)
    //    videoDispObj->setInteractive(false);  // no longer works since the VideoObject is not created until setup() time
    return ret;
}

bool FiducialTracker::set_image_input(const std::vector<std::string> & paths)
{
    liveDemo = false;
    inputImagePaths = paths;
    inputImagePathsIter = inputImagePaths.begin();
    videoMode = false;
    return true;
}

bool FiducialTracker::set_pantilt_reader_input(const std::string & path)
{
    return ptReader.open(path);
}

bool FiducialTracker::cleanAndExit() {
    std::cout << "Tracker trying to clean up" << std::endl;
    this->running = false;
    this->done = false;

    if (this->initialized) {
        // Close all OpenCV windows...
        for (auto & instance : FiducialTracker::instances) {
            if (instance.second->videoCapture.isOpened()) {
                videoCapture.release();
            }
        }
    }
    ErrorHandler::getInstance()->report("Cleaned up Tracker: " + this->getName());
    return !this->running;
}

void FiducialTracker::run()
{
    // From ThreadInterface...
    this->running = this->initialized;

    if (debug) {
        std::cout << "Entering FiducialTracker::run() (id: " << this->threadID() << ")" << std::endl;
    }
    this->eHandler->report("Entering FiducialTracker::run() (id: " + this->threadID() + ")", Shared::Error::Severity::DEBUG, this->getName());

    //VideoRecorder input_video_recorder(this->getName() + "_liveDemo_input.avi");
    //PanTiltRecorder pantilt_recorder(this->getName() + "_pantilt_input.txt");
    //PoseRecorder pose_recorder(this->getName() + "_charuco_poses.txt");

    size_t frame_num = 0;
    char buffer[256];
    cv::Mat frame;
    cv::Vec2d pan_tilt(0,0);
    cv::Vec2d panTiltTarget(nan(""), nan(""));
    //bool new_pose_needed = true;
    //bool new_pose_once = false;
    cv::VideoWriter vwrite;

    // Set up the timer for the processing speed...
    auto start_time = chrono::high_resolution_clock::now();

    double pan_deg=0.0, tilt_deg=0.0;
    double my_pan, my_tilt;
    const double pan_offset = 0.0; //88.0;
    const double tilt_offset = 0.0; //3.0;
    cv::Point2f centerCorner, greenDotLoc;

    // hasReachedTarget() becomes true, how many images should we wait before we start
    // to processing marker detection to ensure the image is "new" after hasReachedTarget()?
    //#ifdef WIN32
    int num_cached_images_to_pop = 3;
    int num_frames_to_skip = -1; // -1 means we are not in skip mode yet (hasReachedTarget() not turned true yet)
    auto ptu_step_start_time = chrono::high_resolution_clock::now();
    bool pantilt_has_reached_target = true;
    bool pantilt_has_changed = true;
    //#endif

    stringstream strbuf; // scratchpad for Logging msg

    if(debug)
    {
        print_parameters();
        std::cout << std::endl << "FiducialTracker: ready to enter the main loop ..."<< std::endl;
    }
    // The tracker loop consists of several main steps:
    // 1) Acquire image from live camera
    // 2) Execute any pending P/T target command;
    //    Wait for P/T to arrive at its target position (triggered by the tracker, see step 4 below)
    //    Even after P/T has reached the target position we keep waiting for several image frame cycles
    //    to ensure the image acquisition buffer are cleared, and we get fresh images after PTU has stopped.
    // 1a) off-line image acquisition
    // 3) Detect the aruco markers in the fiducial (as a way for detecting the charuco board)
    // 4) If fiducial is detected, try estimate the fiducial pose;
    //    At this time fiducial pose hand-off logic is activated:
    //    if fiducial detection at step 3 succeeds & pose estimation at step 4 succeeds
    //      use the estimated pose as the source of hand-off;
    //    else fetch shared fiducial pose && if successful
    //      use shared fiducial pose as the source of hand-off;
    //    else no hand-off, fiducial lost
    // 5) Calculate needed pan-tilt based on hand-off fiducial pose (if applicable)
    // Note: the above steps are a simple description, and the implementation contains other
    // considerations.
    while (!done && this->eHandler->shouldContinue())
    {
        auto iter_start_time = chrono::high_resolution_clock::now();
        if (liveDemo)
        {
            auto capture_start_time = chrono::high_resolution_clock::now();

            //=================Steps 1 start here=================
            if (!this->cameraDevice->listen(&frame))
            {
                std::cerr << "Error: Failed to get image for < " << cameraDevice->getName() << ">" << std::endl;
                break;
            }
            double capture_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - capture_start_time).count();
            if (debug) std::cout << "Captured " + this->cameraDevice->getName() + " image, frame " << frame_num << ", size = "
                << frame.cols << "(W) x" << frame.rows << "(H)" << " x"<<frame.channels() <<"(C)  (" << capture_processing_time << " ms)" << std::endl;

            //=================Steps 2 start here=================

            if (this->panTiltDevice) {

                // If there is pending pan/tilt target that need to be executed (from
                // previous iteration of the loop), we do it here before we check on whether
                // pan/tilt has reached target
                if (!pantilt_has_reached_target)
                {
                    if (!pantilt_has_changed)
                    {
                        if (debug)
                            std::cout << "Executing pan/tilt command for pan,tilt =" << panTiltTarget[0] << ", "
                            << panTiltTarget[1] << std::endl;

                        // If target is accessable
						// FIXME - remove me!!
//						auto temp{ std::vector<double>{panTiltTarget[0], panTiltTarget[1]} };
						// END

						std::static_pointer_cast<PanTiltThread>(panTiltDevice)->step(panTiltTarget); //&temp);
						pantilt_has_changed = true;
                    }
                    else
                        if (debug) std::cout << "Waiting for PTU to reach target pan/tilt from a previous command ..." << std::endl;
                }

                // check to see whether target pan/tilt has been reached & if so, whether we have
                // waited long enough (# of image have been skipped) to begin processing
                if (pantilt_has_changed)
                {
                    if ((static_cast<PanTiltController*> (panTiltDevice->getDevice()))->hasReachedTarget())
                    {
                        // We have to assume that during the entire time of skipping input frames
                        // hasReachedTarget() will always return true.
                        if (num_frames_to_skip < 0) // skip count not yet started, so start it
                        {
                            num_frames_to_skip = num_cached_images_to_pop;
                            if (debug)
                            {
                                double ptu_step_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ptu_step_start_time).count();
                                std::cout << "P/T has reached target (" << ptu_step_processing_time << " ms), count-down starts:" << std::endl;
                            }
                        }
                        if (num_frames_to_skip > 0)
                        {
                            if (debug)
                            {    std::cout << num_frames_to_skip << " frames to skip before starting useful marker detection begins " << std::endl;}
                            num_frames_to_skip--;
                        }
                        else // after the specified num of frames are skipped we finally turn on the flags allowing marker detection
                            // and pose estimation to happen.
                        {
                            if (debug)
                            {    std::cout << "Ready to process markers and pose " << std::endl; }
                            pantilt_has_reached_target = true;
                            pantilt_has_changed = false;
                            num_frames_to_skip = -1;
                        }
                    }
                }
                // read PT current angle positions:
                auto ptu_start_time = chrono::high_resolution_clock::now();

//				// FIXME - remove me!!
//				auto temp{ std::vector<double>{pan_tilt[0], pan_tilt[1]} };
//				// END
//                panTiltDevice->listen(&temp);
//				pan_tilt = cv::Vec2d{temp[0], temp[1]};

                std::static_pointer_cast<PanTiltThread>(panTiltDevice)->listen(pan_tilt);
                double ptu_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ptu_start_time).count();

                pan_deg = pan_tilt[size_t(Interface::PanTilt::Axis::PAN)];
                tilt_deg = pan_tilt[size_t(Interface::PanTilt::Axis::TILT)];
                if (debug) std::cout << "Current PT position: pan=" << pan_deg << ", tilt=" << tilt_deg << " (top of the loop, " << ptu_processing_time << " ms)" << std::endl;

            }
        }

        //=================Step 1a (image acquisition, off-line mode)=================
        else //  not liveDemo
        {
            if (videoMode)
            {
                videoCapture >> frame;
                // if the PanTiltReader is good and provides good data, then use them; otherwise skip
                if(this->ptReader.good())
                {
                    size_t frameId;
                    double pan, tilt;
                    if(this->ptReader.readNext(&frameId, &pan, &tilt))
                    {
                        if(debug)
                          cout << "PanTiltReader::readNext(): frameId, pan, tilt = "<< frameId <<", "<< pan <<", " << tilt << endl;
                        if(frameId==frame_num)
                        {
                            pan_deg = pan; tilt_deg = tilt;
                        }
                        else
                        {
                            cerr <<"Warning: PanTiltReader frame ID out of sync, pan & tilt ignored" << endl;
                        }
                    }
                }
            }
            else
            {
                if(inputImagePathsIter != inputImagePaths.end())
                {
                    if(debug) std::cout << "Reading still image for testing: " << (*inputImagePathsIter) << std::endl;
                    frame = cv::imread((*inputImagePathsIter));
                    inputImagePathsIter++;
                    // Testing frame 200 from liveDemo281:
                    //pan_deg = -75.838;
                    //tilt_deg = 5.84235;
                }
                else
                {
                    frame.release();
                }
            }
            if (frame.empty()) {
                cerr << "Frame/image reader has returned an empty frame (possible end of file/images)" << std::endl;
                break;
            }
        }

        if(recordInput)
        {
            this->input_video_recorder->writeVideoFrame(frame);
            // not meaningful outside of Windows where we currently do not have interface to the PTU setup
            if(this->panTiltDevice)
            { this->pantilt_recorder->write(frame_num, pan_deg, tilt_deg);}
        }
        if(debug)
        {
            double image_acqusition_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - iter_start_time).count();
            std::cout << "Image acquisition: " << frame.cols << "(W) x " << frame.rows << "(H) x "<<frame.channels() <<"(C)  (" << image_acqusition_time
                      <<"ms, recordInput = "<<recordInput << ")"<< std::endl;
        }

        //=================Step 3 detect the fiducial  =================
        auto marker_start_time = chrono::high_resolution_clock::now();
        bool marker_detected = false;
        cv::Point2f imageCenter(frame.cols / 2.0f, frame.rows / 2.0f);

        if ( (pantilt_has_reached_target || !liveDemo) && (marker_detected = this->poseEstimator->detect_aruco_markers(frame, centerCorner)))
        {
            // Debug - record markers for each instance
            this->eHandler->report(
                "Frame: (" + std::to_string(frame_num) + ") # of Markers: " + std::to_string(this->poseEstimator->observedMarkerCount()),
                Shared::Error::Severity::DEBUG,
                this->getName()
            );

            // NOTE: returns delta, not target
            //cv::Point2f delta_pantilt = this->poseEstimator->estimate_needed_pantilt(imageCenter, centerCorner);
            //if (debug) std::cout << "Required Delta(Pan,Tilt) = " << delta_pantilt << " (deg)" << std::endl;
            double marker_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - marker_start_time).count();
            snprintf(buffer, sizeof(buffer), "Frame %05d (pan = %.3f, tilt = %.3f): {ChaRuCo board detected }  (%.3f ms)",
                (int)frame_num, pan_deg, tilt_deg, marker_processing_time);
            this->eHandler->report(buffer, Shared::Error::Severity::DEBUG, this->getName());
            if (debug)
            {
                std::cout << buffer << std::endl;
            }
        }
        else //did not detect the markers
        {
            //new_pose_needed = true;
            double marker_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - marker_start_time).count();
            snprintf(buffer, sizeof(buffer), "Frame %05d (pan = %.3f, tilt = %.3f): {FAILED: ChaRuCo board detection}  (%.3f ms)",
                (int)frame_num, pan_deg, tilt_deg, marker_processing_time);
            this->eHandler->report(buffer, Shared::Error::Severity::WARN, this->getName());
            if (debug)
            {

                std::cout << buffer << std::endl;
            }
        }
        //=================Step 4 fiducial pose estimation & estimation of needed pan/tilt =================
        // Compute the 6 DOF of the fiducial in the video frame
        bool pose_estimate_ok = false;
        my_pan = pan_deg + pan_offset;
        my_tilt = tilt_deg + tilt_offset;
        // These values will be overwritten by estimate_pose_charucoboard() if the estimation is successful.
        // If not successful, the pan & tilt values will be used for display and they are updated in real-time
        // in synch with thee PTU even if the pose estimation is not updated.
        this->sixdof.pan = my_pan;
        this->sixdof.tilt = my_tilt;

        SixDOF fiducialPose{};
        bool hasFiducialPose{false};

        // Inform "FusedPoses" of missing detection
        this->fusedPoseDistributor->updateFiducialState(this->getName(), marker_detected);
        if (pantilt_has_reached_target || !liveDemo) {
			if (marker_detected)
			{

				auto pose_start_time = chrono::high_resolution_clock::now();

				pose_estimate_ok = this->poseEstimator->estimate_pose_charucoboard(frame, my_pan, my_tilt, &this->sixdof);
				double iter_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - pose_start_time).count();

				if (!pose_estimate_ok)
				{
					// If we are viewing or saving the video, we have to process the frame anyway
					snprintf(buffer, sizeof(buffer), "Frame %05d (pan = %.3f, tilt = %.3f): { Pose estimation failed } (%.3f ms)", (int)frame_num, pan_deg, tilt_deg, iter_processing_time);
					this->eHandler->report(buffer, Shared::Error::WARN, this->getName());
					if (debug)
					{
						std::cout << buffer << std::endl;
					}
					this->sixdof.reset();
				}
				else
				{
					fiducialPose = this->sixdof;
					hasFiducialPose = true;

					// Inform "FusedPoses" of new sixDOF
					this->fusedPoseDistributor->recordEstimatedPose(this->getName(), this->sixdof);

					snprintf(buffer, sizeof(buffer), "Frame %05d (pan = %.3f, tilt = %.3f): { x: %.3f, y: %.3f, z: %.3f, yaw: %.3f, pitch: %.3f, roll: %.3f } (%.3f ms)",
						(int)frame_num, this->sixdof.pan, this->sixdof.tilt, this->sixdof.x, this->sixdof.y, this->sixdof.z, this->sixdof.yaw, this->sixdof.pitch, this->sixdof.roll, iter_processing_time);
					this->eHandler->report(buffer, Shared::Error::DEBUG, this->getName());
					if (debug)
					{
						std::cout << buffer << std::endl;
					}
					// For display overlay on the charucoboard
					greenDotLoc = this->poseEstimator->get_center_corner_uv_charucoboard();

					if (debug)
					{
						std::cout << "Updated fiducial center coordinates (green dot coords): " << greenDotLoc << std::endl;
					}
					//new_pose_needed = false;
					// record the fiducial pose to an disk file for analysis / display:
					if (liveDemo || videoMode)
					{
						this->pose_recorder->write(frame_num, this->sixdof);
					}
				}
			}
			else if (this->fusedPoseDistributor->fetchPose(this->getName(), fiducialPose)) {
				// Acquired shared pose
				this->sixdof.reset();
				hasFiducialPose = true;
			}
			else {
				// Fiducial not detected, nor shared pose acquired
				this->sixdof.reset();
			}
		}
        // ================= Step 5 calculate the needed pan/tilt to point the camera at the fiducial=================
        if (hasFiducialPose && ( (liveDemo && this->panTiltDevice) || debug ) ) {

            this->eHandler->report("Frame: (" + std::to_string(frame_num) + ") fiducialPose: " + fiducialPose.toString(), Shared::Error::Severity::DEBUG, this->getName());

            auto solver_start_time = chrono::high_resolution_clock::now();
            //auto point{ std::static_pointer_cast<PoseEstimatorWorld>(this->poseEstimator)->estimate_needed_pantilt(fiducialPose, cv::Point2d{my_pan, my_tilt}) };
            auto pan_tilt_other{cv::Vec2d{my_pan, my_tilt}}; // set initial value
            if ( this->poseEstimator->estimate_needed_pantilt(fiducialPose, pan_tilt_other))
            {
                double solver_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - solver_start_time).count();

                if (fabs(pan_tilt_other[0] - my_pan) > this->angleThresholdDeg || fabs(pan_tilt_other[1] - my_tilt) > this->angleThresholdDeg) {
                    // Set pan/tilt target
                    panTiltTarget = pan_tilt_other;

                    // Set flags that will trigger execution of pan/tilt movement
                    // TODO - fix naming convention of flags
                    pantilt_has_reached_target = false;
                    pantilt_has_changed = false;

                    this->eHandler->report("Frame: (" + std::to_string(frame_num) + ") Set new PT target based on fiducial pose. PT (" + std::to_string(panTiltTarget[0]) + ", " + std::to_string(panTiltTarget[1]) + ")",
                        Shared::Error::Severity::DEBUG, this->getName());
                    if (debug) std::cout << "Estimated pan/tilt target: " << panTiltTarget[0] << ", "<< panTiltTarget[1] <<" (" <<solver_processing_time<< " ms)"<< std::endl;
                }
                else {
                    if (debug) std::cout << "Current pan/tilt within 0.5 deg of target, not moving PTU (" <<solver_processing_time<< " ms)"<< std::endl;
                }
            }
            else {
                this->eHandler->report("Frame: (" + std::to_string(frame_num) + ") WARNING: estimate_needed_pantilt() failed or not applicable",
                    Shared::Error::Severity::DEBUG, this->getName());
                if (debug) std::cout << "WARNING: estimate_needed_pantilt() failed or not applicable" << std::endl;
            }
        }
        //else do nothing since we lost track of the fiducial

        auto disp_start_time = chrono::high_resolution_clock::now();
        double disp_processing_time = 0.0,  video_write_time= 0.0;
        if (doDisplay) {
            // When do we want to show the greendot?
            bool showGreenDot = pose_estimate_ok && (!liveDemo || pantilt_has_reached_target);

            if (liveDemo && this->cameraDevice && this->cameraDevice->getType() == Presets::Device::Type::IMPERX) {
                videoDispObj->createDisplayFrame(frame, frame_num, this->sixdof, showGreenDot, greenDotLoc, 0.5);
            } else {
                videoDispObj->createDisplayFrame(frame, frame_num, this->sixdof, showGreenDot, greenDotLoc);
            }
            disp_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now()
                                                                                       - disp_start_time).count();
            int key = videoDispObj->getUserKey();
            if (key == 'q' || key == 'Q' || key == 27)  // 27 is the escape key
            {
                done = true;
                if (debug)
                    cout << "FiducialTracker: User has requested to quit! (key=" << key << ")" << endl;
            }
        }

        if ((liveDemo || videoMode) && doDisplay && writeVideoFile)
        {
            strbuf.str(""); strbuf.clear();
            if (!videoDispObj->writeVideoFrame())
            {
                strbuf << "WARNING: writeVideoFrame() to video file failed at frame " << frame_num ;
                std::cout << strbuf.str() << std::endl;
                this->eHandler->report(strbuf.str(), Shared::Error::WARN, this->getName());

                video_write_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now()
                                                                                - disp_start_time).count() - disp_processing_time;
            }
        }
        if (debug)
        {
            std::cout << "Overhead for video display setup + imshow, video-write (ms): " << disp_processing_time << ", " << video_write_time << std::endl;
        }

        // =============== Step 5a execute P/T movement if needed================
        // Code has been moved to the top of the while() loop so the PTU code is in one chunck

        double frame_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - iter_start_time).count();
        strbuf.str(""); strbuf.clear();
        strbuf << "================End of frame "<<frame_num<<" (time="<<frame_processing_time<<"ms) =======================";
        std::cout << strbuf.str() << std::endl;
        this->eHandler->report(strbuf.str(), Shared::Error::INFO, this->getName());
        ++frame_num;
    }  // end of while(!done) loop

    this->running = false; //the thread has ended

    double full_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start_time).count();

    {
        // Compute the processing time and frames per second of the operation (won't be accurate if we killed the program early)
        double average_FPS = frame_num / (full_processing_time / 1000.0);
        cout << frame_num << " frames processed in " << full_processing_time << " milliseconds (" << average_FPS << " frames per second)" << endl;
    }

    videoDispObj->setDone(); // signal to the "master" that we are done with video display

    if (debug) {
        //std::cout <<"Exiting FiducialTracker::run() (id: "<< thread_hdl.get_id() << ")"<< std::endl;
        std::cout << "Exiting FiducialTracker::run() (id: " << this->threadID() << ")" << std::endl;
    }

}

bool FiducialTracker::visitStagingArea() {
#ifdef WIN32
    if (!this->panTiltDevice) {
        this->eHandler->report("No attached PTU. Returning success.", Shared::Error::Severity::DEBUG, this->getName());
        return true;
    }

    bool hasReachedStaging{};
    bool readyToSend{ true };
    auto sstream{ std::stringstream{} };
    std::vector<double> ptPosition(0, 2);
    std::vector<double> target{ ptPosition };
    PanTiltController * panTiltController;

    // Lambda Functions
    // TODO - should probably be moved to higher class (used by DataCollector/PanTiltController)
    auto displayStatus = [&](const std::vector<double> position, const bool atHome) {
        sstream.str("");
        try {
            sstream << "\rSend to Staging [ID: " + this->panTiltDevice->getName() + "] => P/T ( " <<
                position.at((int)Interface::PanTilt::Axis::PAN) << ", " <<
                position.at((int)Interface::PanTilt::Axis::TILT) << ")"; // << std::endl;
            std::cout << sstream.str() << std::flush;

        }
        catch (const std::out_of_range & error) {
            // FIXME - add ERRORHANDLER
            std::cout << error.what() << std::endl;
        }
    };
    // END Lambda Functions

    size_t result{};
    if ((result = static_cast<TrackingJSONParser *>(this->config)->stagingPositions.size()) < 1) {
        this->eHandler->report("No staging position passed. Default: [0, 0].", Shared::Error::Severity::INFO, this->getName());
        return true;
    }

    try {
        target =  (result < 2) ? static_cast<TrackingJSONParser *>(this->config)->stagingPositions.begin()->second :
								 static_cast<TrackingJSONParser *>(this->config)->stagingPositions.at(this->panTiltDevice->getSensorID());
    }
    catch (const std::out_of_range & error) {
        this->eHandler->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT, this->getName());
        return false;
    }

    while (!hasReachedStaging) {
        panTiltController = static_cast<PanTiltController *>(this->panTiltDevice->getDevice());
        if (!this->panTiltDevice->listen(&ptPosition)) {
            this->eHandler->report(
                "Failed to acquire PTU poses for unit ( " + this->panTiltDevice->getName() + ")",
                Shared::Error::Severity::KILL_AND_EXIT,
                this->getName()
            );
            return false;
        }

        // If current and staging points are not equivalent (within EPSILON)
        hasReachedStaging = panTiltController->angleEquivalence(ptPosition, target);

        // Display current position...
        displayStatus(ptPosition, hasReachedStaging);

        if (readyToSend && !hasReachedStaging) {
            panTiltController->ignoreSoftLimits = true;
            this->panTiltDevice->step(static_cast<void *>(&target));
            this->eHandler->report("Sent PTU <" + this->panTiltDevice->getName() + ">. Staging: PT (" +
                std::to_string(target[0]) + "," +
                std::to_string(target[1]) + ")", Shared::Error::Severity::INFO, this->getName());
            readyToSend = false;
        }
        else if (hasReachedStaging) {
            this->eHandler->report(this->panTiltDevice->getName() + ": Reached staging area!", Shared::Error::Severity::INFO, this->getName());
        }
        else {
            auto deltaPose{ std::vector<double>{
                std::abs(target[(int)Interface::PanTilt::Axis::PAN] - ptPosition[(int)Interface::PanTilt::Axis::PAN]),
                std::abs(target[(int)Interface::PanTilt::Axis::TILT] - ptPosition[(int)Interface::PanTilt::Axis::TILT])
            } };
            auto sstream{ std::stringstream{} };
            sstream << this->panTiltDevice->getName() << " Pan/Tilt: (" << ptPosition.at((int)Interface::PanTilt::Axis::PAN) << ", " << ptPosition.at((int)Interface::PanTilt::Axis::TILT) <<
                ") Delta: ( " << deltaPose.at((int)Interface::PanTilt::Axis::PAN) << ", " << deltaPose.at((int)Interface::PanTilt::Axis::TILT) << ")";

            // Log Current position (DEBUG)
            this->eHandler->report(sstream.str(), Shared::Error::Severity::INFO, this->getName());
        }

        Sleep(50);
    }
    panTiltController->ignoreSoftLimits = false;
#endif
    return true;
}

