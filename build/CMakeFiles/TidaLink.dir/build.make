# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.7.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.7.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Users/eric/Google Drive/tidalink"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Users/eric/Google Drive/tidalink/build"

# Include any dependencies generated for this target.
include CMakeFiles/TidaLink.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TidaLink.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TidaLink.dir/flags.make

CMakeFiles/TidaLink.dir/main.cpp.o: CMakeFiles/TidaLink.dir/flags.make
CMakeFiles/TidaLink.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/eric/Google Drive/tidalink/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TidaLink.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TidaLink.dir/main.cpp.o -c "/Users/eric/Google Drive/tidalink/main.cpp"

CMakeFiles/TidaLink.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TidaLink.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Users/eric/Google Drive/tidalink/main.cpp" > CMakeFiles/TidaLink.dir/main.cpp.i

CMakeFiles/TidaLink.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TidaLink.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Users/eric/Google Drive/tidalink/main.cpp" -o CMakeFiles/TidaLink.dir/main.cpp.s

CMakeFiles/TidaLink.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/TidaLink.dir/main.cpp.o.requires

CMakeFiles/TidaLink.dir/main.cpp.o.provides: CMakeFiles/TidaLink.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/TidaLink.dir/build.make CMakeFiles/TidaLink.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/TidaLink.dir/main.cpp.o.provides

CMakeFiles/TidaLink.dir/main.cpp.o.provides.build: CMakeFiles/TidaLink.dir/main.cpp.o


# Object files for target TidaLink
TidaLink_OBJECTS = \
"CMakeFiles/TidaLink.dir/main.cpp.o"

# External object files for target TidaLink
TidaLink_EXTERNAL_OBJECTS =

TidaLink: CMakeFiles/TidaLink.dir/main.cpp.o
TidaLink: CMakeFiles/TidaLink.dir/build.make
TidaLink: CMakeFiles/TidaLink.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/Users/eric/Google Drive/tidalink/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable TidaLink"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TidaLink.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TidaLink.dir/build: TidaLink

.PHONY : CMakeFiles/TidaLink.dir/build

CMakeFiles/TidaLink.dir/requires: CMakeFiles/TidaLink.dir/main.cpp.o.requires

.PHONY : CMakeFiles/TidaLink.dir/requires

CMakeFiles/TidaLink.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TidaLink.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TidaLink.dir/clean

CMakeFiles/TidaLink.dir/depend:
	cd "/Users/eric/Google Drive/tidalink/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Users/eric/Google Drive/tidalink" "/Users/eric/Google Drive/tidalink" "/Users/eric/Google Drive/tidalink/build" "/Users/eric/Google Drive/tidalink/build" "/Users/eric/Google Drive/tidalink/build/CMakeFiles/TidaLink.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/TidaLink.dir/depend

