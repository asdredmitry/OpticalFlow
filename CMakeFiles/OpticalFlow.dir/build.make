# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/dmitry/OpticalFlow

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dmitry/OpticalFlow

# Include any dependencies generated for this target.
include CMakeFiles/OpticalFlow.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/OpticalFlow.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OpticalFlow.dir/flags.make

CMakeFiles/OpticalFlow.dir/main.cpp.o: CMakeFiles/OpticalFlow.dir/flags.make
CMakeFiles/OpticalFlow.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmitry/OpticalFlow/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OpticalFlow.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpticalFlow.dir/main.cpp.o -c /home/dmitry/OpticalFlow/main.cpp

CMakeFiles/OpticalFlow.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpticalFlow.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmitry/OpticalFlow/main.cpp > CMakeFiles/OpticalFlow.dir/main.cpp.i

CMakeFiles/OpticalFlow.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpticalFlow.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmitry/OpticalFlow/main.cpp -o CMakeFiles/OpticalFlow.dir/main.cpp.s

CMakeFiles/OpticalFlow.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/OpticalFlow.dir/main.cpp.o.requires

CMakeFiles/OpticalFlow.dir/main.cpp.o.provides: CMakeFiles/OpticalFlow.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/OpticalFlow.dir/build.make CMakeFiles/OpticalFlow.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/OpticalFlow.dir/main.cpp.o.provides

CMakeFiles/OpticalFlow.dir/main.cpp.o.provides.build: CMakeFiles/OpticalFlow.dir/main.cpp.o


# Object files for target OpticalFlow
OpticalFlow_OBJECTS = \
"CMakeFiles/OpticalFlow.dir/main.cpp.o"

# External object files for target OpticalFlow
OpticalFlow_EXTERNAL_OBJECTS =

OpticalFlow: CMakeFiles/OpticalFlow.dir/main.cpp.o
OpticalFlow: CMakeFiles/OpticalFlow.dir/build.make
OpticalFlow: /usr/local/lib/libopencv_videostab.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_objdetect.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_superres.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_dnn.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_shape.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_photo.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_ml.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_stitching.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_video.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_calib3d.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_features2d.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_flann.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_highgui.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_videoio.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_imgproc.so.3.4.0
OpticalFlow: /usr/local/lib/libopencv_core.so.3.4.0
OpticalFlow: CMakeFiles/OpticalFlow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dmitry/OpticalFlow/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable OpticalFlow"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OpticalFlow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OpticalFlow.dir/build: OpticalFlow

.PHONY : CMakeFiles/OpticalFlow.dir/build

CMakeFiles/OpticalFlow.dir/requires: CMakeFiles/OpticalFlow.dir/main.cpp.o.requires

.PHONY : CMakeFiles/OpticalFlow.dir/requires

CMakeFiles/OpticalFlow.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OpticalFlow.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OpticalFlow.dir/clean

CMakeFiles/OpticalFlow.dir/depend:
	cd /home/dmitry/OpticalFlow && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dmitry/OpticalFlow /home/dmitry/OpticalFlow /home/dmitry/OpticalFlow /home/dmitry/OpticalFlow /home/dmitry/OpticalFlow/CMakeFiles/OpticalFlow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OpticalFlow.dir/depend

