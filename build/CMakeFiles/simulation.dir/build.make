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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.5.0/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.5.0/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build

# Include any dependencies generated for this target.
include CMakeFiles/simulation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simulation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simulation.dir/flags.make

CMakeFiles/simulation.dir/src/main.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/simulation.dir/src/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/main.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/main.cpp

CMakeFiles/simulation.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/main.cpp > CMakeFiles/simulation.dir/src/main.cpp.i

CMakeFiles/simulation.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/main.cpp -o CMakeFiles/simulation.dir/src/main.cpp.s

CMakeFiles/simulation.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/main.cpp.o.requires

CMakeFiles/simulation.dir/src/main.cpp.o.provides: CMakeFiles/simulation.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/main.cpp.o.provides

CMakeFiles/simulation.dir/src/main.cpp.o.provides.build: CMakeFiles/simulation.dir/src/main.cpp.o


CMakeFiles/simulation.dir/src/starter3_util.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/starter3_util.cpp.o: ../src/starter3_util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/simulation.dir/src/starter3_util.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/starter3_util.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/starter3_util.cpp

CMakeFiles/simulation.dir/src/starter3_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/starter3_util.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/starter3_util.cpp > CMakeFiles/simulation.dir/src/starter3_util.cpp.i

CMakeFiles/simulation.dir/src/starter3_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/starter3_util.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/starter3_util.cpp -o CMakeFiles/simulation.dir/src/starter3_util.cpp.s

CMakeFiles/simulation.dir/src/starter3_util.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/starter3_util.cpp.o.requires

CMakeFiles/simulation.dir/src/starter3_util.cpp.o.provides: CMakeFiles/simulation.dir/src/starter3_util.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/starter3_util.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/starter3_util.cpp.o.provides

CMakeFiles/simulation.dir/src/starter3_util.cpp.o.provides.build: CMakeFiles/simulation.dir/src/starter3_util.cpp.o


CMakeFiles/simulation.dir/src/camera.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/camera.cpp.o: ../src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/simulation.dir/src/camera.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/camera.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/camera.cpp

CMakeFiles/simulation.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/camera.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/camera.cpp > CMakeFiles/simulation.dir/src/camera.cpp.i

CMakeFiles/simulation.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/camera.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/camera.cpp -o CMakeFiles/simulation.dir/src/camera.cpp.s

CMakeFiles/simulation.dir/src/camera.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/camera.cpp.o.requires

CMakeFiles/simulation.dir/src/camera.cpp.o.provides: CMakeFiles/simulation.dir/src/camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/camera.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/camera.cpp.o.provides

CMakeFiles/simulation.dir/src/camera.cpp.o.provides.build: CMakeFiles/simulation.dir/src/camera.cpp.o


CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o: ../src/vertexrecorder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/vertexrecorder.cpp

CMakeFiles/simulation.dir/src/vertexrecorder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/vertexrecorder.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/vertexrecorder.cpp > CMakeFiles/simulation.dir/src/vertexrecorder.cpp.i

CMakeFiles/simulation.dir/src/vertexrecorder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/vertexrecorder.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/vertexrecorder.cpp -o CMakeFiles/simulation.dir/src/vertexrecorder.cpp.s

CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o.requires

CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o.provides: CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o.provides

CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o.provides.build: CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o


CMakeFiles/simulation.dir/src/clothsystem.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/clothsystem.cpp.o: ../src/clothsystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/simulation.dir/src/clothsystem.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/clothsystem.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/clothsystem.cpp

CMakeFiles/simulation.dir/src/clothsystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/clothsystem.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/clothsystem.cpp > CMakeFiles/simulation.dir/src/clothsystem.cpp.i

CMakeFiles/simulation.dir/src/clothsystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/clothsystem.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/clothsystem.cpp -o CMakeFiles/simulation.dir/src/clothsystem.cpp.s

CMakeFiles/simulation.dir/src/clothsystem.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/clothsystem.cpp.o.requires

CMakeFiles/simulation.dir/src/clothsystem.cpp.o.provides: CMakeFiles/simulation.dir/src/clothsystem.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/clothsystem.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/clothsystem.cpp.o.provides

CMakeFiles/simulation.dir/src/clothsystem.cpp.o.provides.build: CMakeFiles/simulation.dir/src/clothsystem.cpp.o


CMakeFiles/simulation.dir/src/timestepper.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/timestepper.cpp.o: ../src/timestepper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/simulation.dir/src/timestepper.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/timestepper.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/timestepper.cpp

CMakeFiles/simulation.dir/src/timestepper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/timestepper.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/timestepper.cpp > CMakeFiles/simulation.dir/src/timestepper.cpp.i

CMakeFiles/simulation.dir/src/timestepper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/timestepper.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/timestepper.cpp -o CMakeFiles/simulation.dir/src/timestepper.cpp.s

CMakeFiles/simulation.dir/src/timestepper.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/timestepper.cpp.o.requires

CMakeFiles/simulation.dir/src/timestepper.cpp.o.provides: CMakeFiles/simulation.dir/src/timestepper.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/timestepper.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/timestepper.cpp.o.provides

CMakeFiles/simulation.dir/src/timestepper.cpp.o.provides.build: CMakeFiles/simulation.dir/src/timestepper.cpp.o


CMakeFiles/simulation.dir/src/particlesystem.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/particlesystem.cpp.o: ../src/particlesystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/simulation.dir/src/particlesystem.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/particlesystem.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/particlesystem.cpp

CMakeFiles/simulation.dir/src/particlesystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/particlesystem.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/particlesystem.cpp > CMakeFiles/simulation.dir/src/particlesystem.cpp.i

CMakeFiles/simulation.dir/src/particlesystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/particlesystem.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/particlesystem.cpp -o CMakeFiles/simulation.dir/src/particlesystem.cpp.s

CMakeFiles/simulation.dir/src/particlesystem.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/particlesystem.cpp.o.requires

CMakeFiles/simulation.dir/src/particlesystem.cpp.o.provides: CMakeFiles/simulation.dir/src/particlesystem.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/particlesystem.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/particlesystem.cpp.o.provides

CMakeFiles/simulation.dir/src/particlesystem.cpp.o.provides.build: CMakeFiles/simulation.dir/src/particlesystem.cpp.o


CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o: ../src/pendulumsystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/pendulumsystem.cpp

CMakeFiles/simulation.dir/src/pendulumsystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/pendulumsystem.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/pendulumsystem.cpp > CMakeFiles/simulation.dir/src/pendulumsystem.cpp.i

CMakeFiles/simulation.dir/src/pendulumsystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/pendulumsystem.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/pendulumsystem.cpp -o CMakeFiles/simulation.dir/src/pendulumsystem.cpp.s

CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o.requires

CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o.provides: CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o.provides

CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o.provides.build: CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o


CMakeFiles/simulation.dir/src/simplesystem.cpp.o: CMakeFiles/simulation.dir/flags.make
CMakeFiles/simulation.dir/src/simplesystem.cpp.o: ../src/simplesystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/simulation.dir/src/simplesystem.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/src/simplesystem.cpp.o -c /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/simplesystem.cpp

CMakeFiles/simulation.dir/src/simplesystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/src/simplesystem.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/simplesystem.cpp > CMakeFiles/simulation.dir/src/simplesystem.cpp.i

CMakeFiles/simulation.dir/src/simplesystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/src/simplesystem.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/src/simplesystem.cpp -o CMakeFiles/simulation.dir/src/simplesystem.cpp.s

CMakeFiles/simulation.dir/src/simplesystem.cpp.o.requires:

.PHONY : CMakeFiles/simulation.dir/src/simplesystem.cpp.o.requires

CMakeFiles/simulation.dir/src/simplesystem.cpp.o.provides: CMakeFiles/simulation.dir/src/simplesystem.cpp.o.requires
	$(MAKE) -f CMakeFiles/simulation.dir/build.make CMakeFiles/simulation.dir/src/simplesystem.cpp.o.provides.build
.PHONY : CMakeFiles/simulation.dir/src/simplesystem.cpp.o.provides

CMakeFiles/simulation.dir/src/simplesystem.cpp.o.provides.build: CMakeFiles/simulation.dir/src/simplesystem.cpp.o


# Object files for target simulation
simulation_OBJECTS = \
"CMakeFiles/simulation.dir/src/main.cpp.o" \
"CMakeFiles/simulation.dir/src/starter3_util.cpp.o" \
"CMakeFiles/simulation.dir/src/camera.cpp.o" \
"CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o" \
"CMakeFiles/simulation.dir/src/clothsystem.cpp.o" \
"CMakeFiles/simulation.dir/src/timestepper.cpp.o" \
"CMakeFiles/simulation.dir/src/particlesystem.cpp.o" \
"CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o" \
"CMakeFiles/simulation.dir/src/simplesystem.cpp.o"

# External object files for target simulation
simulation_EXTERNAL_OBJECTS =

simulation: CMakeFiles/simulation.dir/src/main.cpp.o
simulation: CMakeFiles/simulation.dir/src/starter3_util.cpp.o
simulation: CMakeFiles/simulation.dir/src/camera.cpp.o
simulation: CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o
simulation: CMakeFiles/simulation.dir/src/clothsystem.cpp.o
simulation: CMakeFiles/simulation.dir/src/timestepper.cpp.o
simulation: CMakeFiles/simulation.dir/src/particlesystem.cpp.o
simulation: CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o
simulation: CMakeFiles/simulation.dir/src/simplesystem.cpp.o
simulation: CMakeFiles/simulation.dir/build.make
simulation: 3rd_party/glfw/src/libglfw3.a
simulation: vecmath/libvecmath.a
simulation: CMakeFiles/simulation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable simulation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simulation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simulation.dir/build: simulation

.PHONY : CMakeFiles/simulation.dir/build

CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/main.cpp.o.requires
CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/starter3_util.cpp.o.requires
CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/camera.cpp.o.requires
CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/vertexrecorder.cpp.o.requires
CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/clothsystem.cpp.o.requires
CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/timestepper.cpp.o.requires
CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/particlesystem.cpp.o.requires
CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/pendulumsystem.cpp.o.requires
CMakeFiles/simulation.dir/requires: CMakeFiles/simulation.dir/src/simplesystem.cpp.o.requires

.PHONY : CMakeFiles/simulation.dir/requires

CMakeFiles/simulation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simulation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simulation.dir/clean

CMakeFiles/simulation.dir/depend:
	cd /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build /Users/johnla/Documents/Senior_Fall/FinalProjects/6.837Final/smoke_and_mirrors/build/CMakeFiles/simulation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simulation.dir/depend

