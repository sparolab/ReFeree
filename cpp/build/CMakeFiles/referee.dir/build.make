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
CMAKE_COMMAND = /home/hogyun2/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/hogyun2/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hogyun2/git/test/referee/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hogyun2/git/test/referee/cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/referee.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/referee.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/referee.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/referee.dir/flags.make

CMakeFiles/referee.dir/src/referee.cpp.o: CMakeFiles/referee.dir/flags.make
CMakeFiles/referee.dir/src/referee.cpp.o: /home/hogyun2/git/test/referee/cpp/src/referee.cpp
CMakeFiles/referee.dir/src/referee.cpp.o: CMakeFiles/referee.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hogyun2/git/test/referee/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/referee.dir/src/referee.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/referee.dir/src/referee.cpp.o -MF CMakeFiles/referee.dir/src/referee.cpp.o.d -o CMakeFiles/referee.dir/src/referee.cpp.o -c /home/hogyun2/git/test/referee/cpp/src/referee.cpp

CMakeFiles/referee.dir/src/referee.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/referee.dir/src/referee.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hogyun2/git/test/referee/cpp/src/referee.cpp > CMakeFiles/referee.dir/src/referee.cpp.i

CMakeFiles/referee.dir/src/referee.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/referee.dir/src/referee.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hogyun2/git/test/referee/cpp/src/referee.cpp -o CMakeFiles/referee.dir/src/referee.cpp.s

# Object files for target referee
referee_OBJECTS = \
"CMakeFiles/referee.dir/src/referee.cpp.o"

# External object files for target referee
referee_EXTERNAL_OBJECTS =

libreferee.a: CMakeFiles/referee.dir/src/referee.cpp.o
libreferee.a: CMakeFiles/referee.dir/build.make
libreferee.a: CMakeFiles/referee.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hogyun2/git/test/referee/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libreferee.a"
	$(CMAKE_COMMAND) -P CMakeFiles/referee.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/referee.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/referee.dir/build: libreferee.a
.PHONY : CMakeFiles/referee.dir/build

CMakeFiles/referee.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/referee.dir/cmake_clean.cmake
.PHONY : CMakeFiles/referee.dir/clean

CMakeFiles/referee.dir/depend:
	cd /home/hogyun2/git/test/referee/cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hogyun2/git/test/referee/cpp /home/hogyun2/git/test/referee/cpp /home/hogyun2/git/test/referee/cpp/build /home/hogyun2/git/test/referee/cpp/build /home/hogyun2/git/test/referee/cpp/build/CMakeFiles/referee.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/referee.dir/depend
