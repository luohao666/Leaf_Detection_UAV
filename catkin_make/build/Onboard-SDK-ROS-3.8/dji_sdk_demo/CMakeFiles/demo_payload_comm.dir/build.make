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
CMAKE_SOURCE_DIR = /home/dji/catkin_make/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dji/catkin_make/build

# Include any dependencies generated for this target.
include Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/depend.make

# Include the progress variables for this target.
include Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/progress.make

# Include the compile flags for this target's objects.
include Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/flags.make

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o: Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/flags.make
Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o: /home/dji/catkin_make/src/Onboard-SDK-ROS-3.8/dji_sdk_demo/src/demo_payload_comm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dji/catkin_make/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o"
	cd /home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk_demo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o -c /home/dji/catkin_make/src/Onboard-SDK-ROS-3.8/dji_sdk_demo/src/demo_payload_comm.cpp

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.i"
	cd /home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dji/catkin_make/src/Onboard-SDK-ROS-3.8/dji_sdk_demo/src/demo_payload_comm.cpp > CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.i

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.s"
	cd /home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dji/catkin_make/src/Onboard-SDK-ROS-3.8/dji_sdk_demo/src/demo_payload_comm.cpp -o CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.s

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o.requires:

.PHONY : Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o.requires

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o.provides: Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o.requires
	$(MAKE) -f Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/build.make Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o.provides.build
.PHONY : Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o.provides

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o.provides.build: Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o


# Object files for target demo_payload_comm
demo_payload_comm_OBJECTS = \
"CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o"

# External object files for target demo_payload_comm
demo_payload_comm_EXTERNAL_OBJECTS =

/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/build.make
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/libimage_transport.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/libmessage_filters.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/libclass_loader.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/libPocoFoundation.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libdl.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/libroscpp.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_signals.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/librosconsole.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/libroslib.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/librospack.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libtinyxml.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/librostime.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /opt/ros/kinetic/lib/libcpp_common.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/local/lib/libadvanced-sensing.a
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libavcodec.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libavformat.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libavutil.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libswscale.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libtheora.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libz.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/local/lib/libdjiosdk-core.a
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/local/lib/libadvanced-sensing.a
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libavcodec.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libavformat.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libavutil.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libswscale.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libtheora.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: /usr/lib/aarch64-linux-gnu/libz.so
/home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm: Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dji/catkin_make/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm"
	cd /home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk_demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo_payload_comm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/build: /home/dji/catkin_make/devel/lib/dji_sdk_demo/demo_payload_comm

.PHONY : Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/build

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/requires: Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/src/demo_payload_comm.cpp.o.requires

.PHONY : Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/requires

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/clean:
	cd /home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk_demo && $(CMAKE_COMMAND) -P CMakeFiles/demo_payload_comm.dir/cmake_clean.cmake
.PHONY : Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/clean

Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/depend:
	cd /home/dji/catkin_make/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dji/catkin_make/src /home/dji/catkin_make/src/Onboard-SDK-ROS-3.8/dji_sdk_demo /home/dji/catkin_make/build /home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk_demo /home/dji/catkin_make/build/Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Onboard-SDK-ROS-3.8/dji_sdk_demo/CMakeFiles/demo_payload_comm.dir/depend

