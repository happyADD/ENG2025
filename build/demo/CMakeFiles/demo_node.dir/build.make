# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dcy/ENG2025/rosdemo/src/demo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dcy/ENG2025/rosdemo/build/demo

# Include any dependencies generated for this target.
include CMakeFiles/demo_node.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/demo_node.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/demo_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo_node.dir/flags.make

CMakeFiles/demo_node.dir/src/node_run.cpp.o: CMakeFiles/demo_node.dir/flags.make
CMakeFiles/demo_node.dir/src/node_run.cpp.o: /home/dcy/ENG2025/rosdemo/src/demo/src/node_run.cpp
CMakeFiles/demo_node.dir/src/node_run.cpp.o: CMakeFiles/demo_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dcy/ENG2025/rosdemo/build/demo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/demo_node.dir/src/node_run.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/demo_node.dir/src/node_run.cpp.o -MF CMakeFiles/demo_node.dir/src/node_run.cpp.o.d -o CMakeFiles/demo_node.dir/src/node_run.cpp.o -c /home/dcy/ENG2025/rosdemo/src/demo/src/node_run.cpp

CMakeFiles/demo_node.dir/src/node_run.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_node.dir/src/node_run.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dcy/ENG2025/rosdemo/src/demo/src/node_run.cpp > CMakeFiles/demo_node.dir/src/node_run.cpp.i

CMakeFiles/demo_node.dir/src/node_run.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_node.dir/src/node_run.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dcy/ENG2025/rosdemo/src/demo/src/node_run.cpp -o CMakeFiles/demo_node.dir/src/node_run.cpp.s

CMakeFiles/demo_node.dir/src/demo_node.cpp.o: CMakeFiles/demo_node.dir/flags.make
CMakeFiles/demo_node.dir/src/demo_node.cpp.o: /home/dcy/ENG2025/rosdemo/src/demo/src/demo_node.cpp
CMakeFiles/demo_node.dir/src/demo_node.cpp.o: CMakeFiles/demo_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dcy/ENG2025/rosdemo/build/demo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/demo_node.dir/src/demo_node.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/demo_node.dir/src/demo_node.cpp.o -MF CMakeFiles/demo_node.dir/src/demo_node.cpp.o.d -o CMakeFiles/demo_node.dir/src/demo_node.cpp.o -c /home/dcy/ENG2025/rosdemo/src/demo/src/demo_node.cpp

CMakeFiles/demo_node.dir/src/demo_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_node.dir/src/demo_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dcy/ENG2025/rosdemo/src/demo/src/demo_node.cpp > CMakeFiles/demo_node.dir/src/demo_node.cpp.i

CMakeFiles/demo_node.dir/src/demo_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_node.dir/src/demo_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dcy/ENG2025/rosdemo/src/demo/src/demo_node.cpp -o CMakeFiles/demo_node.dir/src/demo_node.cpp.s

# Object files for target demo_node
demo_node_OBJECTS = \
"CMakeFiles/demo_node.dir/src/node_run.cpp.o" \
"CMakeFiles/demo_node.dir/src/demo_node.cpp.o"

# External object files for target demo_node
demo_node_EXTERNAL_OBJECTS =

demo_node: CMakeFiles/demo_node.dir/src/node_run.cpp.o
demo_node: CMakeFiles/demo_node.dir/src/demo_node.cpp.o
demo_node: CMakeFiles/demo_node.dir/build.make
demo_node: /opt/ros/humble/lib/libcomponent_manager.so
demo_node: /opt/ros/humble/lib/libcv_bridge.so
demo_node: /opt/ros/humble/lib/librclcpp.so
demo_node: /opt/ros/humble/lib/liblibstatistics_collector.so
demo_node: /opt/ros/humble/lib/librcl.so
demo_node: /opt/ros/humble/lib/librmw_implementation.so
demo_node: /opt/ros/humble/lib/librcl_logging_spdlog.so
demo_node: /opt/ros/humble/lib/librcl_logging_interface.so
demo_node: /opt/ros/humble/lib/librcl_yaml_param_parser.so
demo_node: /opt/ros/humble/lib/libyaml.so
demo_node: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
demo_node: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
demo_node: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
demo_node: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
demo_node: /opt/ros/humble/lib/libtracetools.so
demo_node: /opt/ros/humble/lib/libament_index_cpp.so
demo_node: /opt/ros/humble/lib/libclass_loader.so
demo_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
demo_node: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_generator_py.so
demo_node: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
demo_node: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/libcomposition_interfaces__rosidl_generator_c.so
demo_node: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
demo_node: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
demo_node: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
demo_node: /opt/ros/humble/lib/libfastcdr.so.1.0.24
demo_node: /opt/ros/humble/lib/librmw.so
demo_node: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
demo_node: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
demo_node: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_py.so
demo_node: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_c.so
demo_node: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
demo_node: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
demo_node: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
demo_node: /usr/lib/x86_64-linux-gnu/libpython3.10.so
demo_node: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
demo_node: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
demo_node: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
demo_node: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
demo_node: /opt/ros/humble/lib/librosidl_typesupport_c.so
demo_node: /opt/ros/humble/lib/librosidl_runtime_c.so
demo_node: /opt/ros/humble/lib/librcpputils.so
demo_node: /opt/ros/humble/lib/librcutils.so
demo_node: /usr/local/lib/libopencv_imgcodecs.so.4.10.0
demo_node: /usr/local/lib/libopencv_imgproc.so.4.10.0
demo_node: /usr/local/lib/libopencv_core.so.4.10.0
demo_node: CMakeFiles/demo_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dcy/ENG2025/rosdemo/build/demo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable demo_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo_node.dir/build: demo_node
.PHONY : CMakeFiles/demo_node.dir/build

CMakeFiles/demo_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo_node.dir/clean

CMakeFiles/demo_node.dir/depend:
	cd /home/dcy/ENG2025/rosdemo/build/demo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dcy/ENG2025/rosdemo/src/demo /home/dcy/ENG2025/rosdemo/src/demo /home/dcy/ENG2025/rosdemo/build/demo /home/dcy/ENG2025/rosdemo/build/demo /home/dcy/ENG2025/rosdemo/build/demo/CMakeFiles/demo_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demo_node.dir/depend

