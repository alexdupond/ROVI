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
CMAKE_SOURCE_DIR = /home/miols15/ROVI/ROVI/SamplePluginPA10

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default

# Include any dependencies generated for this target.
include CMakeFiles/RoVi1PluginPA10.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RoVi1PluginPA10.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RoVi1PluginPA10.dir/flags.make

ui_SamplePlugin.h: /home/miols15/ROVI/ROVI/SamplePluginPA10/src/SamplePlugin.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ui_SamplePlugin.h"
	/usr/lib/qt5/bin/uic -o /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/ui_SamplePlugin.h /home/miols15/ROVI/ROVI/SamplePluginPA10/src/SamplePlugin.ui

src/moc_SamplePlugin.cpp: /home/miols15/ROVI/ROVI/SamplePluginPA10/src/SamplePlugin.hpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating src/moc_SamplePlugin.cpp"
	cd /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/src && /usr/lib/qt5/bin/moc @/home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/src/moc_SamplePlugin.cpp_parameters

qrc_resources.cpp: /home/miols15/ROVI/ROVI/SamplePluginPA10/src/pa_icon.png
qrc_resources.cpp: /home/miols15/ROVI/ROVI/SamplePluginPA10/src/resources.qrc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating qrc_resources.cpp"
	/usr/lib/qt5/bin/rcc --name resources --output /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/qrc_resources.cpp /home/miols15/ROVI/ROVI/SamplePluginPA10/src/resources.qrc

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o: CMakeFiles/RoVi1PluginPA10.dir/flags.make
CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o: /home/miols15/ROVI/ROVI/SamplePluginPA10/src/SamplePlugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o -c /home/miols15/ROVI/ROVI/SamplePluginPA10/src/SamplePlugin.cpp

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/miols15/ROVI/ROVI/SamplePluginPA10/src/SamplePlugin.cpp > CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.i

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/miols15/ROVI/ROVI/SamplePluginPA10/src/SamplePlugin.cpp -o CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.s

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.requires:

.PHONY : CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.requires

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.provides: CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/RoVi1PluginPA10.dir/build.make CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.provides.build
.PHONY : CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.provides

CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o.provides.build: CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o


CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o: CMakeFiles/RoVi1PluginPA10.dir/flags.make
CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o: src/moc_SamplePlugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o -c /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/src/moc_SamplePlugin.cpp

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/src/moc_SamplePlugin.cpp > CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.i

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/src/moc_SamplePlugin.cpp -o CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.s

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.requires:

.PHONY : CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.requires

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.provides: CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/RoVi1PluginPA10.dir/build.make CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.provides.build
.PHONY : CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.provides

CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o.provides.build: CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o


CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o: CMakeFiles/RoVi1PluginPA10.dir/flags.make
CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o: qrc_resources.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o -c /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/qrc_resources.cpp

CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/qrc_resources.cpp > CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.i

CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/qrc_resources.cpp -o CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.s

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

/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/src/SamplePlugin.cpp.o
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/src/moc_SamplePlugin.cpp.o
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/qrc_resources.cpp.o
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/build.make
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_algorithms.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_pathplanners.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_pathoptimization.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_simulation.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_opengl.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_assembly.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_task.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_calibration.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_csg.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_control.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_proximitystrategies.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/libyaobi.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/libpqp.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/libfcl.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGL.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_assimp.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_qhull.a
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_csgjs.a
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_unzip.a
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libz.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGL.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_algorithms.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_pathplanners.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_pathoptimization.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_simulation.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_opengl.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_assembly.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_task.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_calibration.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_csg.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_control.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_proximitystrategies.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/libyaobi.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/libpqp.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/libfcl.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGL.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_assimp.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_qhull.a
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_csgjs.a
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /home/miols15/RobWork/RobWork/libs/release/librw_unzip.a
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libz.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_dnn.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_ml.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_objdetect.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_shape.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_stitching.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_superres.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_videostab.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_calib3d.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_features2d.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_flann.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_highgui.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.9.5
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_photo.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_video.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_videoio.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_imgproc.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: /usr/local/lib/libopencv_core.so.3.4.3
/home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so: CMakeFiles/RoVi1PluginPA10.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared module /home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RoVi1PluginPA10.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RoVi1PluginPA10.dir/build: /home/miols15/ROVI/ROVI/SamplePluginPA10/libs/Release/libRoVi1PluginPA10.so

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
	cd /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/miols15/ROVI/ROVI/SamplePluginPA10 /home/miols15/ROVI/ROVI/SamplePluginPA10 /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default /home/miols15/ROVI/ROVI/build-SamplePluginPA10-Imported_Kit-Default/CMakeFiles/RoVi1PluginPA10.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RoVi1PluginPA10.dir/depend
