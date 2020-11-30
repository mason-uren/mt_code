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
include PoseEstimation/CMakeFiles/PoseEstimation.dir/depend.make

# Include the progress variables for this target.
include PoseEstimation/CMakeFiles/PoseEstimation.dir/progress.make

# Include the compile flags for this target's objects.
include PoseEstimation/CMakeFiles/PoseEstimation.dir/flags.make

PoseEstimation/CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.o: PoseEstimation/CMakeFiles/PoseEstimation.dir/flags.make
PoseEstimation/CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.o: ../PoseEstimation/PoseEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object PoseEstimation/CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/PoseEstimator.cpp

PoseEstimation/CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/PoseEstimator.cpp > CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.i

PoseEstimation/CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/PoseEstimator.cpp -o CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.s

PoseEstimation/CMakeFiles/PoseEstimation.dir/SixDOF.cpp.o: PoseEstimation/CMakeFiles/PoseEstimation.dir/flags.make
PoseEstimation/CMakeFiles/PoseEstimation.dir/SixDOF.cpp.o: ../PoseEstimation/SixDOF.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object PoseEstimation/CMakeFiles/PoseEstimation.dir/SixDOF.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PoseEstimation.dir/SixDOF.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/SixDOF.cpp

PoseEstimation/CMakeFiles/PoseEstimation.dir/SixDOF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PoseEstimation.dir/SixDOF.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/SixDOF.cpp > CMakeFiles/PoseEstimation.dir/SixDOF.cpp.i

PoseEstimation/CMakeFiles/PoseEstimation.dir/SixDOF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PoseEstimation.dir/SixDOF.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/SixDOF.cpp -o CMakeFiles/PoseEstimation.dir/SixDOF.cpp.s

PoseEstimation/CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.o: PoseEstimation/CMakeFiles/PoseEstimation.dir/flags.make
PoseEstimation/CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.o: ../PoseEstimation/FusedPoses/FusedPoses.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object PoseEstimation/CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/FusedPoses/FusedPoses.cpp

PoseEstimation/CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/FusedPoses/FusedPoses.cpp > CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.i

PoseEstimation/CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/FusedPoses/FusedPoses.cpp -o CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.s

PoseEstimation/CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.o: PoseEstimation/CMakeFiles/PoseEstimation.dir/flags.make
PoseEstimation/CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.o: ../PoseEstimation/HandOffSolver/HandOffSolver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object PoseEstimation/CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.o"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.o -c /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/HandOffSolver/HandOffSolver.cpp

PoseEstimation/CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.i"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/HandOffSolver/HandOffSolver.cpp > CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.i

PoseEstimation/CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.s"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation/HandOffSolver/HandOffSolver.cpp -o CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.s

# Object files for target PoseEstimation
PoseEstimation_OBJECTS = \
"CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.o" \
"CMakeFiles/PoseEstimation.dir/SixDOF.cpp.o" \
"CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.o" \
"CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.o"

# External object files for target PoseEstimation
PoseEstimation_EXTERNAL_OBJECTS =

lib/libPoseEstimation.a: PoseEstimation/CMakeFiles/PoseEstimation.dir/PoseEstimator.cpp.o
lib/libPoseEstimation.a: PoseEstimation/CMakeFiles/PoseEstimation.dir/SixDOF.cpp.o
lib/libPoseEstimation.a: PoseEstimation/CMakeFiles/PoseEstimation.dir/FusedPoses/FusedPoses.cpp.o
lib/libPoseEstimation.a: PoseEstimation/CMakeFiles/PoseEstimation.dir/HandOffSolver/HandOffSolver.cpp.o
lib/libPoseEstimation.a: PoseEstimation/CMakeFiles/PoseEstimation.dir/build.make
lib/libPoseEstimation.a: PoseEstimation/CMakeFiles/PoseEstimation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library ../lib/libPoseEstimation.a"
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && $(CMAKE_COMMAND) -P CMakeFiles/PoseEstimation.dir/cmake_clean_target.cmake
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PoseEstimation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
PoseEstimation/CMakeFiles/PoseEstimation.dir/build: lib/libPoseEstimation.a

.PHONY : PoseEstimation/CMakeFiles/PoseEstimation.dir/build

PoseEstimation/CMakeFiles/PoseEstimation.dir/clean:
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation && $(CMAKE_COMMAND) -P CMakeFiles/PoseEstimation.dir/cmake_clean.cmake
.PHONY : PoseEstimation/CMakeFiles/PoseEstimation.dir/clean

PoseEstimation/CMakeFiles/PoseEstimation.dir/depend:
	cd /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/PoseEstimation /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-stand-alone/PoseEstimation/CMakeFiles/PoseEstimation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : PoseEstimation/CMakeFiles/PoseEstimation.dir/depend

