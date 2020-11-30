
File: TrackingFiducial/README.txt

This is a stand-alone app for fiducial tracking. Full functionality of the code can be built
on Windows, but partial functionality can be realized building on Linux.

==============================
What does it do?

Windows
  It captures image from Ximea, and find the charuco board (fiducial) and move
  the pan-tilt-unit (PTU) on which the Ximea camera is mounted on to point the camera at
  the fiducal so that the fiducial is positioned roughly at the center of FOV of the camera.
  At that point, it'll estimate the pose of the charuco board in the world coordinate frame
  and display the info in the (smaller version) input image display on the screen.

Linux
  On Linux, no camera or PTU interface is included. Instead, the input comes from
  an image (hardcoded, available in here), or from a video (-v command-line option).
  For the image input or video input, it will try to find the charuco board, estimate
  its pose and display related info superimposed on the input image.

==============================
How to build it?
  Build this app from the top-level CMakeLists.txt is recommended. The resulting executible 
  is found in build/bin/trackfiducial

==============================
Dependencies and other parameters
-Pose Estimation
  Fiducial tracking requires PoseEstimation module as dependency. See ../PoseEstimation.

-Camera intrinsics, charuco board parameters & dynamic extrinsic model:
  Camera intrinsics for Ximea: there are defaults, but can be changed by -i or 
         --camera-intrinsics-dir command-line option. Two files are needed in the directory: 
         ximea_intrinsics.npy and ximea_distCoeffs.npy
  charuco board parameters: there are two charuco board configurations, an standard 8x12 with
         12mm marker square (default), and a 40 x 20, with 23.5mm marker square. Use -b, 
         or --alt-charucoboard to select the 40x20 board.
  dynamic extrinsic model: (aka "cad-model") this is a JSON file for the dynamic extrinsic 
         model from off-line training. There is a default model, or use -i, --cad-model-path
         to specify the pathname to the .json file of the model.

==============================
Which camera & PTU?
 Ximea: currently the code will attempt to connect to the first (id=0) Ximea camera in 
      the system
 PTU: currently the code is hardcoded to connect to PTU at IP=192.168.0.110, TCP port=4000
      and the host is at 192.168.0.104 listening at UDP port = 3001 (PTU must be configured 
      to broadcast P/T position to 192.168.0.104 at UDP port 3001)
 
 We are in the process of putting the default parameters into JSON configuration files.

==============================
Auto-Focus/Dynamic Intrinsics:
 Not included in this release.

=========================================
Additional info for "trackfiducial" app (not all options are avalable on all platforms)

% build/trackfiducial --help
Program for fiducial tracking:
Usage: build/trackfiducial [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -i,--camera-intrinsics-dir TEXT
                              Path to Ximea/Imperx intrinsics directory
  -m,--cad-model-path TEXT    Path of cad-model JSON file
  -b,--alt-charucoboard=0     Track the alternative ChArUco board (TV3); default is the 4x6 inch fiducial
  -v,--video-path TEXT        Specify input video path; default is to use image input
  -r,--record-input-video=0   Record the input video and pan-tilt (on Windows w/) to prespecified output files (liveDemo_input.avi and pantilt_input.txt
  -d,--debug=0                Turn on verbose debug mode
