# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone

# Include any dependencies generated for this target.
include TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/depend.make

# Include the progress variables for this target.
include TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/progress.make

# Include the compile flags for this target's objects.
include TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/flags.make

TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/main.cpp.o: TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/flags.make
TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/main.cpp.o: ../TrackingFiducial/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/main.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TrackingFiducial_Example.dir/main.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/main.cpp

TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TrackingFiducial_Example.dir/main.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/main.cpp > CMakeFiles/TrackingFiducial_Example.dir/main.cpp.i

TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TrackingFiducial_Example.dir/main.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/main.cpp -o CMakeFiles/TrackingFiducial_Example.dir/main.cpp.s

# Object files for target TrackingFiducial_Example
TrackingFiducial_Example_OBJECTS = \
"CMakeFiles/TrackingFiducial_Example.dir/main.cpp.o"

# External object files for target TrackingFiducial_Example
TrackingFiducial_Example_EXTERNAL_OBJECTS =

bin/TrackingFiducial_Example: TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/main.cpp.o
bin/TrackingFiducial_Example: TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/build.make
bin/TrackingFiducial_Example: lib/libUtilities.a
bin/TrackingFiducial_Example: lib/libPoseEstimation.a
bin/TrackingFiducial_Example: lib/libLiMeInterface.a
bin/TrackingFiducial_Example: lib/libModels.a
bin/TrackingFiducial_Example: lib/libDynamicExtrinsics.a
bin/TrackingFiducial_Example: lib/libTrackingFiducial.a
bin/TrackingFiducial_Example: lib/libPoseEstimation.a
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_aruco.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_calib3d.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_features2d.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_flann.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_imgproc.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_core.so.4.3.0
bin/TrackingFiducial_Example: lib/libLiMeInterface.a
bin/TrackingFiducial_Example: /usr/lib/libm3api.so
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/ImperX/lib/Linux64_x64/libIpxDisplayQt.so
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/ImperX/lib/Linux64_x64/libIpxCameraApi.so
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/ImperX/lib/Linux64_x64/libIpxCameraGuiApi.so
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/ImperX/lib/Linux64_x64/libIpxImageApi.so
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_core.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_highgui.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_videoio.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_imgcodecs.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_aruco.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_calib3d.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_features2d.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_imgproc.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_flann.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_core.so.4.3.0
bin/TrackingFiducial_Example: lib/libDynamicExtrinsics.a
bin/TrackingFiducial_Example: lib/libModels.a
bin/TrackingFiducial_Example: lib/libUtilities.a
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/HDF5/lib/libhdf5_cpp_debug.so.200.0.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/HDF5/lib/libhdf5_debug.so.200.0.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_highgui.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_videoio.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_imgcodecs.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_imgproc.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_core.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_aruco.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_calib3d.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_features2d.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_flann.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_imgproc.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_core.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_aruco.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_calib3d.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_features2d.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_flann.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_imgproc.so.4.3.0
bin/TrackingFiducial_Example: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/lib/OpenCV/lib/libopencv_core.so.4.3.0
bin/TrackingFiducial_Example: TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/TrackingFiducial_Example"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/TrackingFiducial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TrackingFiducial_Example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/build: bin/TrackingFiducial_Example

.PHONY : TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/build

TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/clean:
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/TrackingFiducial && $(CMAKE_COMMAND) -P CMakeFiles/TrackingFiducial_Example.dir/cmake_clean.cmake
.PHONY : TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/clean

TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/depend:
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/TrackingFiducial /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : TrackingFiducial/CMakeFiles/TrackingFiducial_Example.dir/depend

