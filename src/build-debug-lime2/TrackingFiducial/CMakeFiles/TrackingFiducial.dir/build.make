# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home.shared/mruren/linux_libs/cmake-v3.15.2/bin/cmake

# The command to remove a file.
RM = /home.shared/mruren/linux_libs/cmake-v3.15.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2

# Include any dependencies generated for this target.
include TrackingFiducial/CMakeFiles/TrackingFiducial.dir/depend.make

# Include the progress variables for this target.
include TrackingFiducial/CMakeFiles/TrackingFiducial.dir/progress.make

# Include the compile flags for this target's objects.
include TrackingFiducial/CMakeFiles/TrackingFiducial.dir/flags.make

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.o: TrackingFiducial/CMakeFiles/TrackingFiducial.dir/flags.make
TrackingFiducial/CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.o: ../TrackingFiducial/ConfigParser/TrackingJSONParser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object TrackingFiducial/CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/ConfigParser/TrackingJSONParser.cpp

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/ConfigParser/TrackingJSONParser.cpp > CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.i

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/ConfigParser/TrackingJSONParser.cpp -o CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.s

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.o: TrackingFiducial/CMakeFiles/TrackingFiducial.dir/flags.make
TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.o: ../TrackingFiducial/FiducialTracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/FiducialTracker.cpp

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/FiducialTracker.cpp > CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.i

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/FiducialTracker.cpp -o CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.s

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.o: TrackingFiducial/CMakeFiles/TrackingFiducial.dir/flags.make
TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.o: ../TrackingFiducial/FTVideoObject.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/FTVideoObject.cpp

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/FTVideoObject.cpp > CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.i

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial/FTVideoObject.cpp -o CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.s

# Object files for target TrackingFiducial
TrackingFiducial_OBJECTS = \
"CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.o" \
"CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.o" \
"CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.o"

# External object files for target TrackingFiducial
TrackingFiducial_EXTERNAL_OBJECTS =

lib/libTrackingFiducial.a: TrackingFiducial/CMakeFiles/TrackingFiducial.dir/ConfigParser/TrackingJSONParser.cpp.o
lib/libTrackingFiducial.a: TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FiducialTracker.cpp.o
lib/libTrackingFiducial.a: TrackingFiducial/CMakeFiles/TrackingFiducial.dir/FTVideoObject.cpp.o
lib/libTrackingFiducial.a: TrackingFiducial/CMakeFiles/TrackingFiducial.dir/build.make
lib/libTrackingFiducial.a: TrackingFiducial/CMakeFiles/TrackingFiducial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library ../lib/libTrackingFiducial.a"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && $(CMAKE_COMMAND) -P CMakeFiles/TrackingFiducial.dir/cmake_clean_target.cmake
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TrackingFiducial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
TrackingFiducial/CMakeFiles/TrackingFiducial.dir/build: lib/libTrackingFiducial.a

.PHONY : TrackingFiducial/CMakeFiles/TrackingFiducial.dir/build

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/clean:
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial && $(CMAKE_COMMAND) -P CMakeFiles/TrackingFiducial.dir/cmake_clean.cmake
.PHONY : TrackingFiducial/CMakeFiles/TrackingFiducial.dir/clean

TrackingFiducial/CMakeFiles/TrackingFiducial.dir/depend:
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/TrackingFiducial /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2 /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-debug-lime2/TrackingFiducial/CMakeFiles/TrackingFiducial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : TrackingFiducial/CMakeFiles/TrackingFiducial.dir/depend
