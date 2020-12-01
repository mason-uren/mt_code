Follow the steps to build the demo:
1.	install VS2017
2.	install CMake (prefer the 3.17 64bit version)
3.	install CUDA 10.2 for windows from https://developer.nvidia.com/cuda-downloads?target_os=Windows&target_arch=x86_64&target_version=10&target_type=exelocal
4.	download/extract WS_LIME_SDK package to a folder (e.g. c:\WS_LIME_SDK)
5.	optional, set PATH env: 
	LIME_SDK_INSTALL_AREA=c:\WS_LIME_SDK
	PATH=%LIME_SDK_INSTALL_AREA%bin;%PATH%
	Or run use sdkprompt.bat in the root folder of the sdk for the rest of the command lines:
6.	Start cmake-gui and open source folder c:\WS_LIME_SDK\sdk\LensControlDemo and specify a build folder (could next to it).or run cmake-gui in the prompt started by sdkprompt.bat if env not set.
      6a. Set OpenCV_DIR to to your OpenCV dirctory
      6b. Set XIMEA_DIR to Ximea API directory which contains xiApi.h(usually it's where you install the Ximea package e.g.C:\XIMEA\API)
7.	Open the solution file and build
8.	Run the output executable path/to/lenscontroldemo.exe --help
