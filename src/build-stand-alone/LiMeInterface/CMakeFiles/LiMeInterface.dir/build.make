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
include LiMeInterface/CMakeFiles/LiMeInterface.dir/depend.make

# Include the progress variables for this target.
include LiMeInterface/CMakeFiles/LiMeInterface.dir/progress.make

# Include the compile flags for this target's objects.
include LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make

LiMeInterface/CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.o: ../LiMeInterface/ConfigParser/InterfaceJSONParser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/ConfigParser/InterfaceJSONParser.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/ConfigParser/InterfaceJSONParser.cpp > CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/ConfigParser/InterfaceJSONParser.cpp -o CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.s

LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.o: ../LiMeInterface/Controllers/PanTilt/PanTiltController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Controllers/PanTilt/PanTiltController.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Controllers/PanTilt/PanTiltController.cpp > CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Controllers/PanTilt/PanTiltController.cpp -o CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.s

LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.o: ../LiMeInterface/Controllers/Camera/CameraInterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Controllers/Camera/CameraInterface.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Controllers/Camera/CameraInterface.cpp > CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Controllers/Camera/CameraInterface.cpp -o CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.s

LiMeInterface/CMakeFiles/LiMeInterface.dir/Network/Network.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/Network/Network.cpp.o: ../LiMeInterface/Network/Network.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/Network/Network.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/Network/Network.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Network/Network.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/Network/Network.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/Network/Network.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Network/Network.cpp > CMakeFiles/LiMeInterface.dir/Network/Network.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/Network/Network.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/Network/Network.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Network/Network.cpp -o CMakeFiles/LiMeInterface.dir/Network/Network.cpp.s

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.o: ../LiMeInterface/Threads/DeviceInterface/DeviceInterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/DeviceInterface/DeviceInterface.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/DeviceInterface/DeviceInterface.cpp > CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/DeviceInterface/DeviceInterface.cpp -o CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.s

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.o: ../LiMeInterface/Threads/ImperX/ImperxThread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/ImperX/ImperxThread.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/ImperX/ImperxThread.cpp > CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/ImperX/ImperxThread.cpp -o CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.s

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.o: ../LiMeInterface/Threads/PanTilt/PanTiltThread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/PanTilt/PanTiltThread.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/PanTilt/PanTiltThread.cpp > CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/PanTilt/PanTiltThread.cpp -o CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.s

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.o: ../LiMeInterface/Threads/Ximea/XimeaThread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/Ximea/XimeaThread.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/Ximea/XimeaThread.cpp > CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/Threads/Ximea/XimeaThread.cpp -o CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.s

LiMeInterface/CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.o: LiMeInterface/CMakeFiles/LiMeInterface.dir/flags.make
LiMeInterface/CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.o: ../LiMeInterface/PhotoAlbum/PhotoAlbum.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object LiMeInterface/CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/PhotoAlbum/PhotoAlbum.cpp

LiMeInterface/CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/PhotoAlbum/PhotoAlbum.cpp > CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.i

LiMeInterface/CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface/PhotoAlbum/PhotoAlbum.cpp -o CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.s

# Object files for target LiMeInterface
LiMeInterface_OBJECTS = \
"CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.o" \
"CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.o" \
"CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.o" \
"CMakeFiles/LiMeInterface.dir/Network/Network.cpp.o" \
"CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.o" \
"CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.o" \
"CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.o" \
"CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.o" \
"CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.o"

# External object files for target LiMeInterface
LiMeInterface_EXTERNAL_OBJECTS =

lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/ConfigParser/InterfaceJSONParser.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/PanTilt/PanTiltController.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/Controllers/Camera/CameraInterface.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/Network/Network.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/DeviceInterface/DeviceInterface.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/ImperX/ImperxThread.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/PanTilt/PanTiltThread.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/Threads/Ximea/XimeaThread.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/PhotoAlbum/PhotoAlbum.cpp.o
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/build.make
lib/libLiMeInterface.a: LiMeInterface/CMakeFiles/LiMeInterface.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX static library ../lib/libLiMeInterface.a"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && $(CMAKE_COMMAND) -P CMakeFiles/LiMeInterface.dir/cmake_clean_target.cmake
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LiMeInterface.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
LiMeInterface/CMakeFiles/LiMeInterface.dir/build: lib/libLiMeInterface.a

.PHONY : LiMeInterface/CMakeFiles/LiMeInterface.dir/build

LiMeInterface/CMakeFiles/LiMeInterface.dir/clean:
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface && $(CMAKE_COMMAND) -P CMakeFiles/LiMeInterface.dir/cmake_clean.cmake
.PHONY : LiMeInterface/CMakeFiles/LiMeInterface.dir/clean

LiMeInterface/CMakeFiles/LiMeInterface.dir/depend:
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/LiMeInterface /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/LiMeInterface/CMakeFiles/LiMeInterface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : LiMeInterface/CMakeFiles/LiMeInterface.dir/depend

