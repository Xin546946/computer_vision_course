# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /home/kit/Downloads/clion-2020.1.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/kit/Downloads/clion-2020.1.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kit/CLionProjects/Snake_active_contour

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kit/CLionProjects/Snake_active_contour/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/self_implement.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/self_implement.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/self_implement.dir/flags.make

CMakeFiles/self_implement.dir/self_implement.cpp.o: CMakeFiles/self_implement.dir/flags.make
CMakeFiles/self_implement.dir/self_implement.cpp.o: ../self_implement.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kit/CLionProjects/Snake_active_contour/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/self_implement.dir/self_implement.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/self_implement.dir/self_implement.cpp.o -c /home/kit/CLionProjects/Snake_active_contour/self_implement.cpp

CMakeFiles/self_implement.dir/self_implement.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/self_implement.dir/self_implement.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kit/CLionProjects/Snake_active_contour/self_implement.cpp > CMakeFiles/self_implement.dir/self_implement.cpp.i

CMakeFiles/self_implement.dir/self_implement.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/self_implement.dir/self_implement.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kit/CLionProjects/Snake_active_contour/self_implement.cpp -o CMakeFiles/self_implement.dir/self_implement.cpp.s

# Object files for target self_implement
self_implement_OBJECTS = \
"CMakeFiles/self_implement.dir/self_implement.cpp.o"

# External object files for target self_implement
self_implement_EXTERNAL_OBJECTS =

self_implement: CMakeFiles/self_implement.dir/self_implement.cpp.o
self_implement: CMakeFiles/self_implement.dir/build.make
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
self_implement: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
self_implement: CMakeFiles/self_implement.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kit/CLionProjects/Snake_active_contour/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable self_implement"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/self_implement.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/self_implement.dir/build: self_implement

.PHONY : CMakeFiles/self_implement.dir/build

CMakeFiles/self_implement.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/self_implement.dir/cmake_clean.cmake
.PHONY : CMakeFiles/self_implement.dir/clean

CMakeFiles/self_implement.dir/depend:
	cd /home/kit/CLionProjects/Snake_active_contour/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kit/CLionProjects/Snake_active_contour /home/kit/CLionProjects/Snake_active_contour /home/kit/CLionProjects/Snake_active_contour/cmake-build-debug /home/kit/CLionProjects/Snake_active_contour/cmake-build-debug /home/kit/CLionProjects/Snake_active_contour/cmake-build-debug/CMakeFiles/self_implement.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/self_implement.dir/depend

