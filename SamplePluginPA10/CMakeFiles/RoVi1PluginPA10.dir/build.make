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
CMAKE_SOURCE_DIR = /home/alexdupond/Documents/ROVI/SamplePluginPA10

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alexdupond/Documents/ROVI/SamplePluginPA10

# Include any dependencies generated for this target.
include CMakeFiles/RoVi1PluginPA10.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RoVi1PluginPA10.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RoVi1PluginPA10.dir/flags.make

ui_SamplePlugin.h: src/SamplePlugin.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alexdupond/Documents/ROVI/SamplePluginPA10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ui_SamplePlugin.h"
	/usr/lib/qt5/bin/uic -o /home/alexdupond/Documents/ROVI/SamplePluginPA10/ui_SamplePlugin.h /home/alexdupond/Documents/ROVI/SamplePluginPA10/src/SamplePlugin.ui

src/moc_SamplePlugin.cpp: src/SamplePlugin.hpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alexdupond/Documents/ROVI/SamplePluginPA10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating src/moc_SamplePlugin.cpp"
	cd /home/alexdupond/Documents/ROVI/SamplePluginPA10/src && /usr/lib/qt5/bin/moc @/home/alexdupond/Documents/ROVI/SamplePluginPA10/src/moc_SamplePlugin.cpp_parameters

qrc_resources.cpp: src/pa_icon.png
qrc_resources.cpp: src/resources.qrc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alexdupond/Documents/ROVI/SamplePluginPA10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating qrc_resources.cpp"
	/usr/lib/qt5/bin/rcc --name resources --output /home/alexdupond/Documents/ROVI/SamplePluginPA10/qrc_resources.cpp /home/alexdupond/Documents/ROVI/SamplePluginPA10/src/resources.qrc

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o: CMakeFiles/RoVi1PluginPA10.dir/flags.make
CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o: src/SamplePlugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alexdupond/Documents/ROVI/SamplePluginPA10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o -c /home/alexdupond/Documents/ROVI/SamplePluginPA10/src/SamplePlugin.cpp

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alexdupond/Documents/ROVI/SamplePluginPA10/src/SamplePlugin.cpp > CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.i

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alexdupond/Documents/ROVI/SamplePluginPA10/src/SamplePlugin.cpp -o CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.s

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.requires:

.PHONY : CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.requires

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.provides: CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/RoVi1PluginPA10.dir/build.make CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.provides.build
.PHONY : CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.provides

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.provides.build: CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o


CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o: CMakeFiles/RoVi1PluginPA10.dir/flags.make
CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o: src/moc_SamplePlugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alexdupond/Documents/ROVI/SamplePluginPA10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o -c /home/alexdupond/Documents/ROVI/SamplePluginPA10/src/moc_SamplePlugin.cpp

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alexdupond/Documents/ROVI/SamplePluginPA10/src/moc_SamplePlugin.cpp > CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.i

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alexdupond/Documents/ROVI/SamplePluginPA10/src/moc_SamplePlugin.cpp -o CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.s

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.requires:

.PHONY : CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.requires

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.provides: CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/RoVi1PluginPA10.dir/build.make CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.provides.build
.PHONY : CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.provides

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.provides.build: CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o


CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o: CMakeFiles/RoVi1PluginPA10.dir/flags.make
CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o: qrc_resources.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alexdupond/Documents/ROVI/SamplePluginPA10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o -c /home/alexdupond/Documents/ROVI/SamplePluginPA10/qrc_resources.cpp

CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alexdupond/Documents/ROVI/SamplePluginPA10/qrc_resources.cpp > CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.i

CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alexdupond/Documents/ROVI/SamplePluginPA10/qrc_resources.cpp -o CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.s

CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o.requires:

.PHONY : CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o.requires

CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o.provides: CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o.requires
	$(MAKE) -f CMakeFiles/RoVi1PluginPA10.dir/build.make CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o.provides.build
.PHONY : CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o.provides

CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o.provides.build: CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o


# Object files for target RoVi1PluginPA10
RoVi1PluginPA10_OBJECTS = \
"CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o" \
"CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o" \
"CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o"

# External object files for target RoVi1PluginPA10
RoVi1PluginPA10_EXTERNAL_OBJECTS =

libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o
libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o
libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o
libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/build.make
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_lua_s.a
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/liblua51.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_algorithms.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_pathplanners.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_pathoptimization.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_simulation.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_opengl.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_assembly.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_task.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_calibration.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_csg.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_control.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_proximitystrategies.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/libyaobi.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/libpqp.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/libfcl.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGL.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGLU.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libassimp.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_qhull.a
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_csgjs.a
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libdl.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.9.5
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGL.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGLU.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_lua_s.a
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/liblua51.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_algorithms.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_pathplanners.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_pathoptimization.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_simulation.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_opengl.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_assembly.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_task.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_calibration.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_csg.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_control.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_proximitystrategies.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/libyaobi.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/libpqp.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/libfcl.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGL.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGLU.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libassimp.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_qhull.a
libs/Release/libRoVi1PluginPA10.so: /home/alexdupond/.robwork/RobWork/libs/release/librw_csgjs.a
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libdl.so
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alexdupond/Documents/ROVI/SamplePluginPA10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared module libs/Release/libRoVi1PluginPA10.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RoVi1PluginPA10.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RoVi1PluginPA10.dir/build: libs/Release/libRoVi1PluginPA10.so

.PHONY : CMakeFiles/RoVi1PluginPA10.dir/build

CMakeFiles/RoVi1PluginPA10.dir/requires: CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.requires
CMakeFiles/RoVi1PluginPA10.dir/requires: CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.requires
CMakeFiles/RoVi1PluginPA10.dir/requires: CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o.requires

.PHONY : CMakeFiles/RoVi1PluginPA10.dir/requires

CMakeFiles/RoVi1PluginPA10.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RoVi1PluginPA10.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RoVi1PluginPA10.dir/clean

CMakeFiles/RoVi1PluginPA10.dir/depend: ui_SamplePlugin.h
CMakeFiles/RoVi1PluginPA10.dir/depend: src/moc_SamplePlugin.cpp
CMakeFiles/RoVi1PluginPA10.dir/depend: qrc_resources.cpp
	cd /home/alexdupond/Documents/ROVI/SamplePluginPA10 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alexdupond/Documents/ROVI/SamplePluginPA10 /home/alexdupond/Documents/ROVI/SamplePluginPA10 /home/alexdupond/Documents/ROVI/SamplePluginPA10 /home/alexdupond/Documents/ROVI/SamplePluginPA10 /home/alexdupond/Documents/ROVI/SamplePluginPA10/CMakeFiles/RoVi1PluginPA10.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RoVi1PluginPA10.dir/depend

