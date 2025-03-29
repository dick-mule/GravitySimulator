# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/richardmule/CLionProjects/GravitySimulator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug

# Utility rule file for Shaders.

# Include any custom commands dependencies for this target.
include CMakeFiles/Shaders.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Shaders.dir/progress.make

CMakeFiles/Shaders: flat_grid.comp.spv
CMakeFiles/Shaders: hyperbolic_grid.comp.spv
CMakeFiles/Shaders: spherical_grid.comp.spv
CMakeFiles/Shaders: grid.vert.spv
CMakeFiles/Shaders: grid.frag.spv

flat_grid.comp.spv: /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/flat_grid.comp
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Compiling shader flat_grid.comp"
	/usr/local/bin/glslc /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/flat_grid.comp -o /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/flat_grid.comp.spv

grid.frag.spv: /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/grid.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Compiling shader grid.frag"
	/usr/local/bin/glslc /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/grid.frag -o /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/grid.frag.spv

grid.vert.spv: /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/grid.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Compiling shader grid.vert"
	/usr/local/bin/glslc /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/grid.vert -o /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/grid.vert.spv

hyperbolic_grid.comp.spv: /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/hyperbolic_grid.comp
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Compiling shader hyperbolic_grid.comp"
	/usr/local/bin/glslc /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/hyperbolic_grid.comp -o /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/hyperbolic_grid.comp.spv

spherical_grid.comp.spv: /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/spherical_grid.comp
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Compiling shader spherical_grid.comp"
	/usr/local/bin/glslc /Users/richardmule/CLionProjects/GravitySimulator/src/shaders/spherical_grid.comp -o /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/spherical_grid.comp.spv

Shaders: CMakeFiles/Shaders
Shaders: flat_grid.comp.spv
Shaders: grid.frag.spv
Shaders: grid.vert.spv
Shaders: hyperbolic_grid.comp.spv
Shaders: spherical_grid.comp.spv
Shaders: CMakeFiles/Shaders.dir/build.make
.PHONY : Shaders

# Rule to build all files generated by this target.
CMakeFiles/Shaders.dir/build: Shaders
.PHONY : CMakeFiles/Shaders.dir/build

CMakeFiles/Shaders.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Shaders.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Shaders.dir/clean

CMakeFiles/Shaders.dir/depend:
	cd /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/richardmule/CLionProjects/GravitySimulator /Users/richardmule/CLionProjects/GravitySimulator /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug /Users/richardmule/CLionProjects/GravitySimulator/cmake-build-debug/CMakeFiles/Shaders.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/Shaders.dir/depend

