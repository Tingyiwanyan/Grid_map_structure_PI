# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tingyi/Incremental-MFPT

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tingyi/Incremental-MFPT/build

# Include any dependencies generated for this target.
include CMakeFiles/ocean_planning.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ocean_planning.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ocean_planning.dir/flags.make

CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.o: CMakeFiles/ocean_planning.dir/flags.make
CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.o: ../src/main/ocean_planning.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tingyi/Incremental-MFPT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.o -c /home/tingyi/Incremental-MFPT/src/main/ocean_planning.cc

CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tingyi/Incremental-MFPT/src/main/ocean_planning.cc > CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.i

CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tingyi/Incremental-MFPT/src/main/ocean_planning.cc -o CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.s

# Object files for target ocean_planning
ocean_planning_OBJECTS = \
"CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.o"

# External object files for target ocean_planning
ocean_planning_EXTERNAL_OBJECTS =

ocean_planning: CMakeFiles/ocean_planning.dir/src/main/ocean_planning.cc.o
ocean_planning: CMakeFiles/ocean_planning.dir/build.make
ocean_planning: src/method_manager/libmethod_manager.a
ocean_planning: src/animation/libanimation.a
ocean_planning: src/mdp_methods/libmdp_methods.a
ocean_planning: src/info_methods/libinfo_methods.a
ocean_planning: src/viz_tool/libviz_tool.a
ocean_planning: src/spline/libspline.a
ocean_planning: src/utils/libutils.a
ocean_planning: src/vehicle/libvehicle.a
ocean_planning: src/disturbance/libdisturbance.a
ocean_planning: src/hungarian/libhungarian.a
ocean_planning: src/method_manager/libmethod_manager.a
ocean_planning: /usr/lib/x86_64-linux-gnu/libGL.so
ocean_planning: /usr/lib/x86_64-linux-gnu/libGLU.so
ocean_planning: /usr/lib/x86_64-linux-gnu/libglut.so
ocean_planning: /usr/lib/x86_64-linux-gnu/libXmu.so
ocean_planning: /usr/lib/x86_64-linux-gnu/libXi.so
ocean_planning: src/mdp_methods/libmdp_methods.a
ocean_planning: src/spline/libspline.a
ocean_planning: src/info_methods/libinfo_methods.a
ocean_planning: src/utils/libutils.a
ocean_planning: /usr/lib/libopencv_dnn.so.3.3.1
ocean_planning: /usr/lib/libopencv_ml.so.3.3.1
ocean_planning: /usr/lib/libopencv_objdetect.so.3.3.1
ocean_planning: /usr/lib/libopencv_shape.so.3.3.1
ocean_planning: /usr/lib/libopencv_stitching.so.3.3.1
ocean_planning: /usr/lib/libopencv_superres.so.3.3.1
ocean_planning: /usr/lib/libopencv_videostab.so.3.3.1
ocean_planning: /usr/lib/libopencv_calib3d.so.3.3.1
ocean_planning: /usr/lib/libopencv_features2d.so.3.3.1
ocean_planning: /usr/lib/libopencv_flann.so.3.3.1
ocean_planning: /usr/lib/libopencv_highgui.so.3.3.1
ocean_planning: /usr/lib/libopencv_photo.so.3.3.1
ocean_planning: /usr/lib/libopencv_video.so.3.3.1
ocean_planning: /usr/lib/libopencv_videoio.so.3.3.1
ocean_planning: /usr/lib/libopencv_imgcodecs.so.3.3.1
ocean_planning: /usr/lib/libopencv_imgproc.so.3.3.1
ocean_planning: /usr/lib/libopencv_core.so.3.3.1
ocean_planning: /usr/lib/x86_64-linux-gnu/libarmadillo.so
ocean_planning: CMakeFiles/ocean_planning.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tingyi/Incremental-MFPT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ocean_planning"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ocean_planning.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ocean_planning.dir/build: ocean_planning

.PHONY : CMakeFiles/ocean_planning.dir/build

CMakeFiles/ocean_planning.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ocean_planning.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ocean_planning.dir/clean

CMakeFiles/ocean_planning.dir/depend:
	cd /home/tingyi/Incremental-MFPT/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tingyi/Incremental-MFPT /home/tingyi/Incremental-MFPT /home/tingyi/Incremental-MFPT/build /home/tingyi/Incremental-MFPT/build /home/tingyi/Incremental-MFPT/build/CMakeFiles/ocean_planning.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ocean_planning.dir/depend

