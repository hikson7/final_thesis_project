# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.20.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.20.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/build

# Include any dependencies generated for this target.
include CMakeFiles/serial_comm_tester.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/serial_comm_tester.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/serial_comm_tester.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/serial_comm_tester.dir/flags.make

CMakeFiles/serial_comm_tester.dir/main.o: CMakeFiles/serial_comm_tester.dir/flags.make
CMakeFiles/serial_comm_tester.dir/main.o: ../main.cpp
CMakeFiles/serial_comm_tester.dir/main.o: CMakeFiles/serial_comm_tester.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/serial_comm_tester.dir/main.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/serial_comm_tester.dir/main.o -MF CMakeFiles/serial_comm_tester.dir/main.o.d -o CMakeFiles/serial_comm_tester.dir/main.o -c /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/main.cpp

CMakeFiles/serial_comm_tester.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_comm_tester.dir/main.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/main.cpp > CMakeFiles/serial_comm_tester.dir/main.i

CMakeFiles/serial_comm_tester.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_comm_tester.dir/main.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/main.cpp -o CMakeFiles/serial_comm_tester.dir/main.s

CMakeFiles/serial_comm_tester.dir/sercom.o: CMakeFiles/serial_comm_tester.dir/flags.make
CMakeFiles/serial_comm_tester.dir/sercom.o: ../sercom.cpp
CMakeFiles/serial_comm_tester.dir/sercom.o: CMakeFiles/serial_comm_tester.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/serial_comm_tester.dir/sercom.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/serial_comm_tester.dir/sercom.o -MF CMakeFiles/serial_comm_tester.dir/sercom.o.d -o CMakeFiles/serial_comm_tester.dir/sercom.o -c /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/sercom.cpp

CMakeFiles/serial_comm_tester.dir/sercom.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_comm_tester.dir/sercom.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/sercom.cpp > CMakeFiles/serial_comm_tester.dir/sercom.i

CMakeFiles/serial_comm_tester.dir/sercom.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_comm_tester.dir/sercom.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/sercom.cpp -o CMakeFiles/serial_comm_tester.dir/sercom.s

# Object files for target serial_comm_tester
serial_comm_tester_OBJECTS = \
"CMakeFiles/serial_comm_tester.dir/main.o" \
"CMakeFiles/serial_comm_tester.dir/sercom.o"

# External object files for target serial_comm_tester
serial_comm_tester_EXTERNAL_OBJECTS =

serial_comm_tester: CMakeFiles/serial_comm_tester.dir/main.o
serial_comm_tester: CMakeFiles/serial_comm_tester.dir/sercom.o
serial_comm_tester: CMakeFiles/serial_comm_tester.dir/build.make
serial_comm_tester: CMakeFiles/serial_comm_tester.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable serial_comm_tester"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial_comm_tester.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/serial_comm_tester.dir/build: serial_comm_tester
.PHONY : CMakeFiles/serial_comm_tester.dir/build

CMakeFiles/serial_comm_tester.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/serial_comm_tester.dir/cmake_clean.cmake
.PHONY : CMakeFiles/serial_comm_tester.dir/clean

CMakeFiles/serial_comm_tester.dir/depend:
	cd /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/build /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/build /Users/Hikari/Documents/GitHub/unsw_final_thesis/POSIX/serial_remote_controller/build/CMakeFiles/serial_comm_tester.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/serial_comm_tester.dir/depend

