#include <iostream>

// OpenCV
#include <opencv2/core.hpp>
#include <CLI11/CLI11.hpp>  //CLI11 CL argument parser

// HRL
#include <Shared/OSDefines.h>
#include <Shared/SystemConfig.h>
#include <Shared/Presets.h>

// Utilities
#include "Logger/Logger.h"
#include "ErrorHandler/ErrorHandler.h"

// Models
#include "Board/FiducialModel.h"
#include "Camera/CameraModel.h"

#include "FiducialTracker.h"
#include "ConfigParser/ModelsJSONParser.h"
#include "ConfigParser/TrackingJSONParser.h"

using namespace std;

// Sample still images for testing ChaRuCo board detection: no longer used(
//const char * ximeaSampleImage = "../resources/XimeaImages/zoom200mm/ximean200_09.jpg";

#ifdef WIN32
bool WINAPI winExitHandler(DWORD signal) {
    switch (signal) {
        case CTRL_C_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_SHUTDOWN_EVENT: {
            //cout << "Signal: " << signal << endl;
            DeviceInterface::safeExit();
            break;
        }
    }
    return true;
}
#endif

// The following depends on 2 macros defined in CMakeLists.txt
string jsonModelFile = string(CAD_MODEL_JSON_PATH);

//#######################################################################################
int main(int argc, char **argv)
{
    const string myName{"StandaloneTracker"};
    string ximea_intrinsics_dir = string(XIMEA_INTRINSICS_DIR);
    string jsonModelFile=string(CAD_MODEL_JSON_PATH);
    string jsonConfigFile;
    string inputVideoPath;
    string pantiltFilePath;
    vector<string> inputImagePaths;
    bool debug = false;
    bool altCharucoBoard = false;
    bool recordInput = false;
    bool noDisplay = false;
    bool noRecordResults = false;
    double markerDetectionScale = 0.2;
    // Parse command-line options:
    CLI::App app{"Program for fiducial tracking: \nDefault behaviors (no -v or positional args): Windows: process Ximea images; Linux/Darwin: process hardcoded image(s)"};
    app.add_option("-i,--camera-intrinsics-dir", ximea_intrinsics_dir, "Path to Ximea/Imperx intrinsics directory");
    app.add_option("-m,--cad-model-path", jsonModelFile, "Path of cad-model JSON file; an empty string or 'none' disables this feature");
    app.add_flag("-b,--alt-charucoboard", altCharucoBoard, "Track the alternative ChArUco board (TV3); default is the 4x6 inch fiducial")->default_val(false);
    app.add_option("-v,--video-path", inputVideoPath, "Specify input video path; has precedence over <input-image-paths>; see top of help for default behaviors");
    app.add_option("-f,--pantilt-path", pantiltFilePath, "Specify the file path to read the pan & tilt to go with the video input");
    app.add_flag("-r,--record-input-video", recordInput, "Record the input video and pan-tilt (on Windows) to prespecified output files (liveDemo_input.avi and pantilt_input.txt")->default_val(false);
    app.add_flag("-n,--no-display", noDisplay, "Disable result video display")->default_val(false);
    app.add_flag("-s,--no-save-result", noRecordResults, "Do not save result video")->default_val(false);
    app.add_option("--marker-detection-scale", markerDetectionScale, "Image scale factor for Aruco marker detection ")->default_val(0.2);
    app.add_flag("-d,--debug", debug, "Turn on verbose debug mode")->default_val(false);
    //app.add_option("-c, --config-file", jsonConfigFile, "Path to the config file (JSON) to load the optional parameters for the run");
    app.add_option("input-image-paths", inputImagePaths, "Optional paths to input images")->expected(0,-1);
    CLI11_PARSE(app, argc, argv);
    
    string ximea_distCoeffs = ximea_intrinsics_dir + "/ximea_distCoeffs.npy";
    string ximea_intrinsics = ximea_intrinsics_dir + "/ximea_cameraMatrix.npy";

	// Initialize Logger & ErrorHandler
    ErrorHandler::getInstance()->setListeningLevel( debug ? Shared::Error::Severity::DEBUG : Shared::Error::Severity::INFO);

    // Single tracking instance
    std::shared_ptr<FiducialTracker> tracker{};

#ifdef WIN32
    /*  some windows-specific stuff here that may need to be cleaned up */	

    // Register Exit Handler
    if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE) winExitHandler, true)) {
        return EXIT_FAILURE;
    }
      
    if (inputVideoPath.empty() && inputImagePaths.empty())
    {
		const int ximeaPCIeSlot = 0; // change this for the Ximea camera you want to use, either 0 or 1
		//const Network network = Network(ConnectionInterface{ "192.168.0.104", "3001" }, ConnectionInterface{ "192.168.0.111", "4001" });
		const Network network = Network(ConnectionInterface{ "192.168.0.105", "3001" }, ConnectionInterface{ "192.168.0.111", "4001" });

		// TODO - how to handle imperx camera?
		// Add camera (assumes ximea)
		try {
			DeviceInterface::addComponent(Presets::Tracking::DefaultDeviceKeys::XIMEA_ID, 
				new XimeaThread(GenICamAdapterInterface::createAdapter(GenICamAdapterInterface::CameraEnumType::XIMEA), ximeaPCIeSlot)
			);

			DeviceInterface::addComponent(Presets::Tracking::DefaultDeviceKeys::IMPERX_ID, 
				new ImperxThread(GenICamAdapterInterface::createAdapter(GenICamAdapterInterface::CameraEnumType::IMPERX)));
		}
		catch (const std::out_of_range & error) {
			ErrorHandler::getInstance()->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT);
			DeviceInterface::safeExit();
			return EXIT_FAILURE;
		}

		// Add ptu
		try {
			DeviceInterface::addComponent(Presets::Tracking::DefaultDeviceKeys::PTU_ID, new PanTiltThread(
				new PanTiltController(network, Interface::PanTilt::PID_TYPE::INTERNAL), "0")
			);
		}
		catch (const std::out_of_range & error) {
			ErrorHandler::getInstance()->report(error.what(), Shared::Error::Severity::KILL_AND_EXIT);
			DeviceInterface::safeExit();
			return EXIT_FAILURE;
		}
		

		if (!DeviceInterface::initialize()) {
			DeviceInterface::safeExit();
			return EXIT_FAILURE;
		}

		// Sanity check!
		if (!ErrorHandler::getInstance()->shouldContinue()) {
			ErrorHandler::getInstance()->report("Error: Sanity check failed. Expect exit.");
			DeviceInterface::safeExit();
			return EXIT_FAILURE;
		}
    }
#endif // WIN32
    if (!jsonConfigFile.empty())
    {
        //std::cout << "WARNING: JSON Configure file has not been implemented and will be ignored for this run:"
        //          << endl << jsonConfigFile << std::endl;
        // Ideally, JSON config file provides the default config parameters over hardcoded default values,
        // and then they can be over-ridden by the values specified on the command-line if given.
        // For example, if JSON config file is given on the command-line (-c/--config-file), and
        // it contains value for cad-model path, which we have a default from hardcoding, then the value
        // in JSON config file takes over.  Furthermore, if the user specify -m/--cad-model-path, then
        // the value specified by -m/--cad-model-path will be used.

		// FIXME - ADD IMPL

		//ErrorHandler::getInstance()->report("Separate JSON configuration specified.");

		//// Parse Models JSON Config
		//auto modelsParser{ ModelsJSONParser() };
		//if (!modelsParser.loadModelsConfig()) {
		//	return EXIT_FAILURE;
		//}

		//// Need to match intrinsic IDs specified in ModelsConfig.json
		//const std::string ximeaIntrinsicsID{"xi_I0"};
		////const std::string ximeaIntrinsicsIDs{"xi_I1"};

		//auto parser{ new TrackingJSONParser() };
		//parser->loadTrackingConfig(jsonConfigFile); 
  //      tracker = new FiducialTracker(parser, modelsParser.ximea.at(ximeaIntrinsicsID), altCharucoBoard);
    }
    else
    {
        ErrorHandler::getInstance()->report("No configuration JSON sourced.",Shared::Error::Severity::DEBUG, myName);
        if (jsonModelFile.compare("none")==0) jsonModelFile.clear(); // none is same as empty string

        auto cameraModel{ CameraModel<double>{ximea_intrinsics, ximea_distCoeffs, 1.0, myName} };
		auto fiducialModel{ altCharucoBoard ? 
            FiducialModel{Model::Board{ 40, 20, 0.0235F, 0.01665F }, cv::aruco::DICT_4X4_1000, myName } :
            FiducialModel{Model::Board{ 8, 12, 0.012F, 0.009F }, cv::aruco::DICT_4X4_1000, myName}
		};
        tracker = std::make_shared<FiducialTracker>(jsonModelFile, cameraModel, fiducialModel, debug,
                                                    markerDetectionScale, !noDisplay, !noRecordResults, recordInput , myName);

		tracker->loadConfig<TrackingJSONParser>();
		// Set staging position
		tracker->Config<TrackingJSONParser>()->stagingPositions.insert({ 
			Presets::Tracking::DefaultDeviceKeys::PTU_ID, std::vector<double>{-65, 5} });
    }

    try
    {
        if (inputVideoPath.empty() && inputImagePaths.empty() ) // both video & image paths are empty --> use ximea + ptu
        {
            #ifdef WIN32
            {

            }
            #else //for Linux/Darwin, work in image mode: take image paths default to hardcoded images
            {
                if (altCharucoBoard)
                {
                    inputImagePaths.push_back("./Ximea_TV_Left.jpg");
                }
                else
                {
                    inputImagePaths.push_back("./ximea200_01.jpg");
                    //inputImagePaths.push_back("./liveDemo_input281_frame200.png");
                }
                tracker->set_image_input(inputImagePaths);
            }
            #endif
        }
        // If a video file in given with -v/--video-path, then work in video mode (input images given on command-line are ignored)
        else if (! inputVideoPath.empty() )
        {
            if (! tracker->set_video_input(inputVideoPath) )
            {
                std::cerr << "Error attempting to open video for read: " << inputVideoPath << std::endl;
                return 1;
            }
            if (! pantiltFilePath.empty() )
                tracker->set_pantilt_reader_input(pantiltFilePath);
        }
        else // inputImagePaths is given
        {
            tracker->set_image_input(inputImagePaths);
        }

        tracker->start();
        while(!VideoObject::isDone())
        {
            VideoObject::updateAllDisplayWindows();
            Sleep(10);
        }

        tracker->wait(); // wait until thread is done (through user interaction in the tracking loop
    } 
    catch (cv::Exception & e)
    {
        std::cout << e.what() << std::endl;
		tracker->stop();
        DeviceInterface::safeExit();
    }
    catch (...)
    {
        std::cout << "Exception during tracking processing ... "<< std::endl;
		tracker->stop();
        DeviceInterface::safeExit();
    }

	// Clean up any/remaining threads (will clean up Logger/ErrorHandler as well)
	tracker->stop();
	DeviceInterface::safeExit();

    std::cout << "Program exits now!"<< std::endl;

    return EXIT_SUCCESS;
}
