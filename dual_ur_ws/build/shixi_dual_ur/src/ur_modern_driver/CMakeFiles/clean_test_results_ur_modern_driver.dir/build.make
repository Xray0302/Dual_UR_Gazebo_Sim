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
CMAKE_SOURCE_DIR = /home/xurui/dual_ur_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xurui/dual_ur_ws/build

# Utility rule file for clean_test_results_ur_modern_driver.

# Include the progress variables for this target.
include shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/progress.make

shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver:
	cd /home/xurui/dual_ur_ws/build/shixi_dual_ur/src/ur_modern_driver && /usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/remove_test_results.py /home/xurui/dual_ur_ws/build/test_results/ur_modern_driver

clean_test_results_ur_modern_driver: shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver
clean_test_results_ur_modern_driver: shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/build.make

.PHONY : clean_test_results_ur_modern_driver

# Rule to build all files generated by this target.
shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/build: clean_test_results_ur_modern_driver

.PHONY : shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/build

shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/clean:
	cd /home/xurui/dual_ur_ws/build/shixi_dual_ur/src/ur_modern_driver && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_ur_modern_driver.dir/cmake_clean.cmake
.PHONY : shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/clean

shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/depend:
	cd /home/xurui/dual_ur_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xurui/dual_ur_ws/src /home/xurui/dual_ur_ws/src/shixi_dual_ur/src/ur_modern_driver /home/xurui/dual_ur_ws/build /home/xurui/dual_ur_ws/build/shixi_dual_ur/src/ur_modern_driver /home/xurui/dual_ur_ws/build/shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : shixi_dual_ur/src/ur_modern_driver/CMakeFiles/clean_test_results_ur_modern_driver.dir/depend

