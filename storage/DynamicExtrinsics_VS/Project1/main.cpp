#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#ifdef WIN32 
#define _USE_MATH_DEFINES // for C++
#include <cmath>
// the above does not work
#define	M_PI 3.141592653589793238462643383279502884197169399375105820974944592307
#else
#include <math.h>
#endif

#include "PoseEstimator.h"

#ifdef WIN32
// HRL Metrology2020 interface code:
// GenICam
#include "Adapters/GenICamInterface/GenICamAdapterInterface.h"

// HRL
#include "Threads/Interface/ThreadInterface.h"
#include "Threads/Ximea/XimeaThread.h"
#include "CommonIncludes/Shared/SharedStructs.h"
#include "Utilities/Logger/Logger.h"
#include "Utilities/ErrorHandler/ErrorHandler.h"
//#include "Threads/ImperX/ImperxThread.h"
#include "Threads/PanTilt/PanTiltThread.h"
#endif

#ifdef WIN32
#include <Windows.h>
#define sleep(x) Sleep((x*1000))
#else //Assuming Posix
#include <unistd.h>
// Use this: sleep(seconds)
#endif


// Compiler options to turn off the display and video writing (these are optional features that slow down the algorithm)
// In a real system, they would be placed in a separate thread, so turning them both off is representative of a "live" system.
#define DO_DISPLAY 1
#define WRITE_VIDFILE 1


bool 
#ifdef WIN32
   liveDemo = true; // if true, then capture from camera & control PTU
//#define STILL_IMAGE_TEST
#else
// we can only exercise part of the code for image/video inputs
   liveDemo = false;
// still image vs. video input can both be done in non-live mode
#define STILL_IMAGE_TEST
#endif

using namespace std;

const bool bOverwriteOldResults = true;
const bool Debug = true;

string vidfile = "../resources/Video_1000x1000.mp4";       const double scaleFactor = 6.0;
string outfilename = "../resources/resultfile1k.avi";
//string vidfile = "../resources/Video_2000x2000.mp4";     const double scaleFactor = 3.0;
//string outfilename = "../resources/resultfile2k.avi";

string modelfile = "../resources/cad_model_4_z0_angle_imp_5.txt";
string jsonModelFile = "../resources/2019-09-04-cad-model.json";
string ximea_distCoeffs = "../resources/ximea_intrinsics/ximea_distCoeffs.npy";
string ximea_intrinsics = "../resources/ximea_intrinsics/ximea_intrinsics.npy";
string ptPositionFile = "../resources/PTpositions-milestone.csv";

// Sample still images for testing ChaRuCo board detection:
const char * ximeaSampleImage = "../resources/XimeaImages/zoom200mm/ximean200_09.jpg";

//#######################################################################################
bool readPTPositionFile(string filename, vector<vector<string> > *filedata)
{
    // Read the pan and tilt values from a csv-formatted file
    // The format of the data is: "frame_num, pan, tilt\n"
    // https://www.geeksforgeeks.org/csv-file-management-using-c/

    filedata->clear();
    ifstream fin(filename, ios::in);

    vector<string> rowdata;
    string line, word, temp;

    getline(fin, line);  // immediately read the first line and throw it away; it contains the header information
    while(1)
    {
        // Pull a line from the file and store it in a stringstream object for parsing
        getline(fin, line);
        //cout << line << endl;
        if(line.empty()) {
            break;
        }
        stringstream s(line);

        // Parse the line in 's' and split on the comma character, storing the elements in 'rowdata'
        rowdata.clear();
        while (getline(s, word, ',')) {
            rowdata.push_back(word);
        }
        filedata->push_back(rowdata);

    }
    fin.close();

    // Check the integrity of the data; all elements in the return object should have 3 parts: frame_num, pan, and tilt
    for (size_t i = 0; i < filedata->size(); ++i)
    {
        vector<string> framedata = filedata->at(i);
        if(framedata.size() != 3) {
            cout << "ERROR! There aren't 3 elements in the PT vector for element " << i << ". Found " << framedata.size() << endl;
            for(size_t j = 0; j < framedata.size(); ++j) {
                cout << framedata[j] << endl;
            }
        return false;
        }
    }
    return true;
}

//#######################################################################################
int main(int argc, char **argv)
{
    double imageScaleFactor = 0.0;
    cv::VideoCapture cap;
    int frame_width, frame_height, num_frames=64000;
    #ifdef WIN32
    XimeaThread *ximeaCameraDevice;
    #endif
    if (!liveDemo)
    {
        // Load the video file
        cap.open(vidfile);
        if(!cap.isOpened()) {
            cout << "ERROR! Cannot open stream or video file: " << vidfile << endl;
            sleep(5);
            return EXIT_FAILURE;
        }
        frame_width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
        frame_height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        num_frames = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
        double fps = cap.get(cv::CAP_PROP_FPS);
        cout << "Video file '" << vidfile << "' contains " << num_frames << " frames (" << frame_height << "x" << frame_width << ")" << endl;
        
        #ifdef STILL_IMAGE_TEST
        //testing processing with still image input
        if (argc >= 2) 
        {
            ximeaSampleImage = argv[1]; 
            cout <<"Testing image given on command-line: "<< ximeaSampleImage <<endl;
        }
        imageScaleFactor = 1.0;
        #else
        imageScaleFactor = scaleFactor;
        #endif
    }
    else
    {    
        /*  some windows-specific stuff here that need to be cleaned up
        // Initialize Logger & ErrorHandler
        Logger::getInstance();
        ErrorHandler::getInstance()->setListeningLevel(Shared::Error::Severity::NO_ERR);	

        // Register Exit Handler
        if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE) winExitHandler, true)) {
            return EXIT_FAILURE;
        }
        */
        imageScaleFactor = 1.0; // original scale
        int sensorID = 0;
        #ifdef WIN32
        ximeaCameraDevice = new XimeaThread(GenICamAdapterInterface::createAdapter(GenICamAdapterInterface::CameraEnumType::XIMEA), sensorID);
        ximeaCameraDevice->initDevice();
        ximeaCameraDevice->startThread();
        ximeaCameraDevice->deviceInfo(true);
        cout << "________********* Need to set frame_width and frame_height ***************______________"<< endl;
        #endif
        outfilename = "./liveDemo_output.avi";

    }

    vector<vector<string> > PTvalues;  // for when liveDemo==false

    #ifdef WIN32
    // Read the file containing the pan-tilt values and ensure that it is the correct length;
    Network * network;
    PanTiltThread * panTiltThread;
    #endif
    if (liveDemo)
    {
        #ifdef WIN32
        network = new Network(ConnectionInterface{"192.168.0.104", "3001"}, ConnectionInterface{"192.168.0.110", "4000"});
        panTiltThread = new PanTiltThread(new PanTiltController(*network), 3);
        panTiltThread->initDevice();
        panTiltThread->startThread();
        panTiltThread->deviceInfo(true);
        /* Code set the initial pan/tilt angles */
        std::vector<double> target{ -4.0, 7.0 };
        if (panTiltThread->step(&target)) {
            //std::cout << "Setting initial PTU @ " << target[0] << " " << target[1] << std::endl;
        }
        
        #endif
    }
    else
    {
        if (!readPTPositionFile(ptPositionFile, &PTvalues))
         {
            cout << "Failure reading pan-tilt value file" << endl;
            sleep(5);
            return EXIT_FAILURE;
        }

        if(PTvalues.size() != cap.get(cv::CAP_PROP_FRAME_COUNT) )
        {
            cout << "WARNING: Number of PT pairs and frames in video do not match!" << endl;
            cout << "frames in video: " << cap.get(cv::CAP_PROP_FRAME_COUNT) << endl;
            cout << "PT pairs: " << PTvalues.size() << endl;
            sleep(5);
            //return EXIT_FAILURE;
        }
    }
    PoseEstimator pose_estim;
    if(!pose_estim.init(modelfile, ximea_distCoeffs, ximea_intrinsics, imageScaleFactor, Debug)) {
        cout << "ERROR! Unable to initialize the PoseEstimator" << endl;
        sleep(5);
        return EXIT_FAILURE;
    }
    
    double focal_length = pose_estim.get_focal_length()*1.0; // 1.3 is a fudge factor because the intrinsics are not correct
    // Instantaneous field-of-view, the FOV of each pixel (at the ceter of the image)
    float ifov = 1.0/focal_length/M_PI*180.0; //in degrees
    // This is for using the new format JSON cad-model from Jacob's dynamic extrinsics code
    // that Mason converted to C++. "true" forces PoseEstimator::calc() to use this model instead
    // of the model implemented in PoseEstimator to do dynamic extrinsics.
    pose_estim.init2(jsonModelFile, true);

#if(DO_DISPLAY)
    cv::namedWindow("PTVideo", cv::WINDOW_AUTOSIZE);
#endif


    // Run the main loop. Pull an image, process it for the 6 DOF, compute the FPS,
    // and then prepare some type of output for it (e.g., write to file, display, or nothing)
    double pan_deg, tilt_deg;
    size_t frame_num = 0;
    char buffer[256];
    bool bKilled = false;
    cv::Mat frame;
    std::vector<double>pan_tilt(2,0.0);
    std::vector<double> panTiltTarget(2);
    bool pantilt_has_changed = false;
	bool pantilt_has_reached_target = false;
    bool new_pose_needed = true;
    cv::VideoWriter vwrite;

    // Set up the timer for the processing speed...
    auto start_time = chrono::high_resolution_clock::now();

    double my_pan, my_tilt;
    const double pan_offset = 0.0; //88.0;
    const double tilt_offset = 0.0; //3.0;
	auto ptu_start_time = chrono::high_resolution_clock::now();

    // ...and GO!
    while (1)
    {
        auto iter_start_time = chrono::high_resolution_clock::now();
        if (liveDemo)
        {
#ifdef WIN32
            if (pantilt_has_changed)
            {
                if( (static_cast<PanTiltController*> (panTiltThread->getDevice()))->hasReachedTarget())
				{
					if (Debug)
					{
						double ptu_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ptu_start_time).count();
						std::cout << "PTU has reached target position, elapsed time = " << ptu_processing_time << " (ms)" << std::endl;
					}
					if (ximeaCameraDevice->listen(&frame))
					{
						if (Debug) std::cout << "Clearing out the capture buffer of staled image ..." << std::endl;
					}
					pantilt_has_reached_target = true;
					pantilt_has_changed = false;
				}
            }
            // read PT current angle positions:
            panTiltThread->listen(&pan_tilt);
            //pan_tilt = static_cast<PanTiltController*> (panTiltThread->getDevice())->getCurrentAngle();
            pan_deg = pan_tilt[size_t(Shared::PanTilt::Axis::PAN)];
            tilt_deg = pan_tilt[size_t(Shared::PanTilt::Axis::TILT)];
            if (Debug) std::cout << "Current PT position: pan=" << pan_deg << ", tilt=" << tilt_deg << " (top of the loop)" << std::endl;

            if (!ximeaCameraDevice->listen(&frame))
            {
                std::cerr << "Error: Failed to get image for < " << ximeaCameraDevice->getName() << ">" << std::endl;
                break;
            }
            frame_width = frame.cols;
            frame_height = frame.rows;
            if(Debug) std::cout << "Captured Ximea image, frame "<< frame_num << ", size = " << frame_width << "(W) x" << frame_height << "(H)" << std::endl;

#endif
        }
        else
        {
            cap >> frame;  // Capture the source and transfer into 'img';

#ifdef STILL_IMAGE_TEST
            std::cout << "Reading still image instead for testing: " << ximeaSampleImage << std::endl;
            frame = cv::imread(string(ximeaSampleImage));
            // Try resize to 50% each dim since Ximea images are too big
            //cv::resize(frame, frame, cv::Size(), 0.5, 0.5, cv::INTER_LANCZOS4);
            std::cout << "Still image size = " << frame.cols << "(W) x" << frame.rows << "(H)" << std::endl;
            frame_width = frame.cols;
            frame_height = frame.rows;
            // alternatively try Gaussian Blur:
            // cv::GaussianBlur(frame, frame, cv::Size(3,3), 0.5)
#endif

            if (frame.empty()) {
                cerr << "Frame reader has returned an empty frame (possible end of file)" << std::endl;
                break;
            }
            // Look up the correct pan and tilt values for the current frame in the video
            assert(frame_num == atoi(PTvalues[frame_num][0].c_str()));
            pan_deg = stod(PTvalues[frame_num][1]);
            tilt_deg = stod(PTvalues[frame_num][2]);

        }

        // Compute the 6 DOF of the fiducial in the video frame and measure the processing time
        SixDOF sixdof, new6dof;

        double scale4MarkerDetection = 0.25; // to increase processing speed
        cv::Point2f markerCorner;
        cv::Mat markerFrame{};
        if (scale4MarkerDetection==1.0)
        {
            markerFrame = frame;
        }
        else
        {
            cv::resize(frame, markerFrame, cv::Size(), scale4MarkerDetection, scale4MarkerDetection,cv::INTER_LANCZOS4);
        }
        auto marker_start_time = chrono::high_resolution_clock::now();
        bool marker_detected = false;
        if (pose_estim.detect_aruco_markers(markerFrame, markerCorner))
        {
            double marker_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - marker_start_time).count();
            if (Debug)
            {
                snprintf(buffer, sizeof(buffer), "Frame %05d (pan = %.3f, tilt = %.3f): {ChaRuCo board detected }  (%.3f ms)",
                    (int)frame_num, pan_deg, tilt_deg, marker_processing_time);
                std::cout << buffer << std::endl;
            }
            marker_detected = true;
            if (Debug) std::cout << "Center marker corner: " << markerCorner << std::endl;
            //cv::imshow("Aruco Markers", frame_copy);
            cv::Point2f imageCenter(markerFrame.cols / 2.0, markerFrame.rows / 2.0);
            cv::Point2f offCenterPixels = imageCenter - markerCorner;
            if (Debug) std::cout << "Marker off center by: " << offCenterPixels << std::endl;
            if (Debug) std::cout << "IFOV (deg) = " << ifov << std::endl;
            cv::Point2f delta_pantilt = offCenterPixels * ifov / scale4MarkerDetection;
            if (Debug) std::cout << "Required Delta(Pan,Tilt) = " << delta_pantilt << std::endl;
            #ifndef STILL_IMAGE_TEST
            // with EPSILON set to 0.1 currently, that's too small for our rough estimate of delta_pantilt
            if (fabs(delta_pantilt.x) > 0.5 || fabs(delta_pantilt.y) > 0.5)
            {
                panTiltTarget[size_t(Shared::PanTilt::Axis::PAN)] = pan_deg + delta_pantilt.x;
                panTiltTarget[size_t(Shared::PanTilt::Axis::TILT)] = tilt_deg - delta_pantilt.y;
                if (Debug) std::cout << "Needed pan/tilt target: " << panTiltTarget[size_t(Shared::PanTilt::Axis::PAN)] << ", "
                          << panTiltTarget[size_t(Shared::PanTilt::Axis::TILT)] << std::endl;
				pantilt_has_reached_target = false;
                new_pose_needed = true;
            }
            else // within EPSILON
            {
                if (Debug) std::cout << "Pan & tile positions within 0.5 degrees; not moving PTU." << std::endl;
            }
            #endif
        }
        else 
        {
            if (Debug)
            {
                snprintf(buffer, sizeof(buffer), "Frame %05d (pan = %.3f, tilt = %.3f): {FAILED: ChaRuCo board detection}", (int)frame_num, pan_deg, tilt_deg);
                std::cout << buffer << std::endl;
            }
        }
            //panTiltThread->step(&panTiltTarget); //moved to after waitKey()
        bool pose_estimate_ok = false;
        my_pan = pan_deg + pan_offset;
        my_tilt = tilt_deg + tilt_offset;
        #ifndef STILL_IMAGE_TEST
        if (marker_detected && new_pose_needed && pantilt_has_reached_target)
        {
        #endif
            auto pose_start_time = chrono::high_resolution_clock::now();
			pose_estimate_ok = pose_estim.calc(frame, my_pan, my_tilt, &new6dof);
            double iter_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - pose_start_time).count();
            int charuco_sq_found = new6dof.num_charuco_found;

            if (!pose_estimate_ok)
            {
                if (Debug)
                {
                    // If we are viewing or saving the video, we have to process the frame anyway
                    snprintf(buffer, sizeof(buffer), "Frame %05d (pan = %.3f, tilt = %.3f): { Pose estimation failed } (%.3f ms)", (int)frame_num, pan_deg, tilt_deg, iter_processing_time);
                    std::cout << buffer << std::endl;
                }
            }
            else
            {
                sixdof = new6dof;
                if (Debug)
                {
                    snprintf(buffer, sizeof(buffer), "Frame %05d (pan = %.3f, tilt = %.3f): { x: %.3f, y: %.3f, z: %.3f, yaw: %.3f, pitch: %.3f, roll: %.3f } (%.3f ms)",
                        (int)frame_num, sixdof.pan, sixdof.tilt, sixdof.x, sixdof.y, sixdof.z, sixdof.yaw, sixdof.pitch, sixdof.roll, iter_processing_time);
                    std::cout << buffer << std::endl;
                }
                new_pose_needed = false;
            }
        #ifndef STILL_IMAGE_TEST
        }
        #endif
#if(DO_DISPLAY)
        cv::Mat dispFrame{};
		double dispScale = 0.125;
        cv::resize(frame, dispFrame, cv::Size(), dispScale, dispScale, cv::INTER_LINEAR);  // downsample the frame to give a reasonably-sized video
		if (dispFrame.type() == CV_8UC1)
			cv::cvtColor(dispFrame, dispFrame, cv::COLOR_GRAY2BGR);
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 1.0;
        int thickness = 2;
        char label[128];
        int spacing = 35, startpos = 60;
        if (true) //( bGood)
        {

            // Display the video frame in an onscreen window
            if (bOverwriteOldResults)
            {
                // Draw a filled rectangle to obscure the old scores and write the new ones on top
                cv::rectangle(dispFrame, cv::Point(725, 27), cv::Point(997, 250), cv::Scalar(80, 80, 80), -1);

                /*
                // This is for ChArUco testing - show the number of ChArUco squares that the algorithm detected
                snprintf(label, sizeof(label), "Frame: %d", frame_num);
                cv::putText(frame, label, cv::Point(730, startpos), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);

                snprintf(label, sizeof(label), "ChArUco: %d", charuco_sq_found);
                cv::putText(frame, label, cv::Point(730, startpos + spacing * 1), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);
                */

                snprintf(label, sizeof(label), "X: %.3f", sixdof.x);
                cv::putText(dispFrame, label, cv::Point(730, startpos), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);      // x position

                snprintf(label, sizeof(label), "Y: %.3f", sixdof.y);
                cv::putText(dispFrame, label, cv::Point(730, startpos + spacing * 1), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);      // y position

                snprintf(label, sizeof(label), "Z: %.3f", sixdof.z);
                cv::putText(dispFrame, label, cv::Point(730, startpos + spacing * 2), fontFace, fontScale, cv::Scalar(255, 255, 0), thickness);     // z position

                snprintf(label, sizeof(label), "Pitch: %.3f", sixdof.pitch);
                cv::putText(dispFrame, label, cv::Point(730, startpos + spacing * 3), fontFace, fontScale, cv::Scalar(0, 255, 0), thickness);     // pitch (rx)

                snprintf(label, sizeof(label), "Yaw: %.3f", sixdof.yaw);
                cv::putText(dispFrame, label, cv::Point(730, startpos + spacing * 4), fontFace, fontScale, cv::Scalar(0, 255, 0), thickness);     // yaw (ry)

                snprintf(label, sizeof(label), "Roll: %.3f", sixdof.roll);
                cv::putText(dispFrame, label, cv::Point(730, startpos + spacing * 5), fontFace, fontScale, cv::Scalar(0, 255, 0), thickness);     // roll (rz)
            }

        }

        snprintf(label, sizeof(label), "Pan: %5.1f", my_pan);
        cv::putText(dispFrame, label, cv::Point(30,  50), fontFace, fontScale, cv::Scalar(200, 200, 64), thickness);     // frame id

        snprintf(label, sizeof(label), "Tilt:  %5.1f", my_tilt);
        cv::putText(dispFrame, label, cv::Point(30, 50 + spacing ), fontFace, fontScale, cv::Scalar(200, 200, 64), thickness);     // frame id

        snprintf(label, sizeof(label), "Frame: %5zu", frame_num);
        cv::putText(dispFrame, label, cv::Point(30, dispFrame.rows - 50), fontFace, fontScale, cv::Scalar(0, 200, 128), thickness);     // frame id

		// Draw a green disc to signal that a new pose is successfully estimated
		if(!new_pose_needed)
			cv::circle(dispFrame, markerCorner/scale4MarkerDetection*dispScale, 20, cv::Scalar(0, 200,0), -1);

        // resize to gain some speed for remote session for testing
		//cv::Mat dispFrame2;
		//cv::resize(dispFrame, dispFrame2, cv::Size(), 0.5, 0.5, cv::INTER_LANCZOS4);
		cv::imshow("PTVideo", dispFrame);

#ifdef STILL_IMAGE_TEST
        char key = cv::waitKey(0);      // minimum wait between frames (only for display purposes)
        std::cout << "Press any key to continue ... " << std::endl;
#else
        char key = cv::waitKey(1);      // minimum wait between frames (only for display purposes)
#endif
        if (key == 'q' || key == 'Q' || key == 27) {  // 27 is the escape key
            bKilled = true;
            break;
        }
        #ifndef STILL_IMAGE_TEST
        panTiltThread->listen(&pan_tilt);
        //pan_tilt = static_cast<PanTiltController*> (panTiltThread->getDevice())->getCurrentAngle();
        pan_deg = pan_tilt[size_t(Shared::PanTilt::Axis::PAN)];
        tilt_deg = pan_tilt[size_t(Shared::PanTilt::Axis::TILT)];
        if (Debug) std::cout << "Current PT position: pan=" << pan_deg << ", tilt=" << tilt_deg << " (bottom of loop)" << std::endl;
        if (!pantilt_has_reached_target)
        {
			if (!pantilt_has_changed)
			{
				if (Debug)
					std::cout << "Executing pan/tilt command for pan,tilt =" << panTiltTarget[size_t(Shared::PanTilt::Axis::PAN)] << ", "
					<< panTiltTarget[size_t(Shared::PanTilt::Axis::TILT)] << std::endl;
				ptu_start_time = chrono::high_resolution_clock::now();
				panTiltThread->step(&panTiltTarget);
				pantilt_has_changed = true;
			}
			else
				if (Debug) std::cout << "Waiting for PTU to reach the desired target pan/tilt ..." << std::endl;
        }
        #endif
		
	#if(WRITE_VIDFILE)
        if (!vwrite.isOpened())
        {
            vwrite.open(outfilename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,//fps
                                cv::Size(dispFrame.cols, dispFrame.rows), true);
        }
        // Write the new frame to the output video file
        vwrite.write(dispFrame);
		if (Debug) std::cout << "Wrote frame " << frame_num << " to output video." << std::endl;
	#endif
#endif //DO_DISPLAY

        double frame_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - iter_start_time).count();
        std::cout << "================End of frame "<<frame_num<<" (time="<<frame_processing_time<<"ms) =======================" << std::endl;
        ++frame_num;
    }  // end of while(1) loop

    double full_processing_time = (double)chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start_time).count();

    // Release bindings, free up memory, etc.
#if(WRITE_VIDFILE)
    vwrite.release();
    cout << "Video file written to " << outfilename << endl;
#endif
    cap.release();
    //cv::destroyAllWindows();

    if (bKilled) {
        num_frames = frame_num + 1;
        // Compute the processing time and frames per second of the operation (won't be accurate if we killed the program early)
        double average_FPS = num_frames / (full_processing_time / 1000.0);
        cout << num_frames << " frames processed in " << full_processing_time << " milliseconds (" << average_FPS << " frames per second)" << endl;
        cout << "Program exits in 3 seconds ... " << endl;
        sleep(3);
    }

    return EXIT_SUCCESS;
}
