# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_test.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_test.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_test.dir/flags.make

utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o: utils/CMakeFiles/aruco_test.dir/flags.make
utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o: ../utils/aruco_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o"
	cd /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_test.dir/aruco_test.cpp.o -c /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/utils/aruco_test.cpp

utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_test.dir/aruco_test.cpp.i"
	cd /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/utils/aruco_test.cpp > CMakeFiles/aruco_test.dir/aruco_test.cpp.i

utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_test.dir/aruco_test.cpp.s"
	cd /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/utils/aruco_test.cpp -o CMakeFiles/aruco_test.dir/aruco_test.cpp.s

utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o.requires:

.PHONY : utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o.requires

utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o.provides: utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o.requires
	$(MAKE) -f utils/CMakeFiles/aruco_test.dir/build.make utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o.provides.build
.PHONY : utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o.provides

utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o.provides.build: utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o


# Object files for target aruco_test
aruco_test_OBJECTS = \
"CMakeFiles/aruco_test.dir/aruco_test.cpp.o"

# External object files for target aruco_test
aruco_test_EXTERNAL_OBJECTS =

utils/aruco_test: utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o
utils/aruco_test: utils/CMakeFiles/aruco_test.dir/build.make
utils/aruco_test: src/libaruco.so.3.1.11
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
utils/aruco_test: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
utils/aruco_test: utils/CMakeFiles/aruco_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable aruco_test"
	cd /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_test.dir/build: utils/aruco_test

.PHONY : utils/CMakeFiles/aruco_test.dir/build

utils/CMakeFiles/aruco_test.dir/requires: utils/CMakeFiles/aruco_test.dir/aruco_test.cpp.o.requires

.PHONY : utils/CMakeFiles/aruco_test.dir/requires

utils/CMakeFiles/aruco_test.dir/clean:
	cd /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_test.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_test.dir/clean

utils/CMakeFiles/aruco_test.dir/depend:
	cd /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11 /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/utils /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/utils /home/carlos/git/3DVisionMobileManipulation/ArUco/aruco-3.1.11/build/utils/CMakeFiles/aruco_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_test.dir/depend

