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
CMAKE_SOURCE_DIR = /home/dcy/ENG/ENG2025/src/ImgProcess

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dcy/ENG/ENG2025/build/imgprocess

# Utility rule file for imgprocess_uninstall.

# Include any custom commands dependencies for this target.
include CMakeFiles/imgprocess_uninstall.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/imgprocess_uninstall.dir/progress.make

CMakeFiles/imgprocess_uninstall:
	/usr/local/bin/cmake -P /home/dcy/ENG/ENG2025/build/imgprocess/ament_cmake_uninstall_target/ament_cmake_uninstall_target.cmake

imgprocess_uninstall: CMakeFiles/imgprocess_uninstall
imgprocess_uninstall: CMakeFiles/imgprocess_uninstall.dir/build.make
.PHONY : imgprocess_uninstall

# Rule to build all files generated by this target.
CMakeFiles/imgprocess_uninstall.dir/build: imgprocess_uninstall
.PHONY : CMakeFiles/imgprocess_uninstall.dir/build

CMakeFiles/imgprocess_uninstall.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imgprocess_uninstall.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imgprocess_uninstall.dir/clean

CMakeFiles/imgprocess_uninstall.dir/depend:
	cd /home/dcy/ENG/ENG2025/build/imgprocess && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dcy/ENG/ENG2025/src/ImgProcess /home/dcy/ENG/ENG2025/src/ImgProcess /home/dcy/ENG/ENG2025/build/imgprocess /home/dcy/ENG/ENG2025/build/imgprocess /home/dcy/ENG/ENG2025/build/imgprocess/CMakeFiles/imgprocess_uninstall.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imgprocess_uninstall.dir/depend

