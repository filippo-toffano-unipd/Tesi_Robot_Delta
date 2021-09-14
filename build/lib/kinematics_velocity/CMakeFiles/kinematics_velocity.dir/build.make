# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/emanuelezanoni/programma_c

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/emanuelezanoni/programma_c/build

# Include any dependencies generated for this target.
include lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/depend.make

# Include the progress variables for this target.
include lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/progress.make

# Include the compile flags for this target's objects.
include lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/flags.make

lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.o: lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/flags.make
lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.o: ../lib/kinematics_velocity/src/kinematics_velocity.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/emanuelezanoni/programma_c/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.o"
	cd /home/emanuelezanoni/programma_c/build/lib/kinematics_velocity && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.o   -c /home/emanuelezanoni/programma_c/lib/kinematics_velocity/src/kinematics_velocity.c

lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.i"
	cd /home/emanuelezanoni/programma_c/build/lib/kinematics_velocity && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/emanuelezanoni/programma_c/lib/kinematics_velocity/src/kinematics_velocity.c > CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.i

lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.s"
	cd /home/emanuelezanoni/programma_c/build/lib/kinematics_velocity && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/emanuelezanoni/programma_c/lib/kinematics_velocity/src/kinematics_velocity.c -o CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.s

# Object files for target kinematics_velocity
kinematics_velocity_OBJECTS = \
"CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.o"

# External object files for target kinematics_velocity
kinematics_velocity_EXTERNAL_OBJECTS =

lib/kinematics_velocity/libkinematics_velocity.a: lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/src/kinematics_velocity.c.o
lib/kinematics_velocity/libkinematics_velocity.a: lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/build.make
lib/kinematics_velocity/libkinematics_velocity.a: lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/emanuelezanoni/programma_c/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libkinematics_velocity.a"
	cd /home/emanuelezanoni/programma_c/build/lib/kinematics_velocity && $(CMAKE_COMMAND) -P CMakeFiles/kinematics_velocity.dir/cmake_clean_target.cmake
	cd /home/emanuelezanoni/programma_c/build/lib/kinematics_velocity && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinematics_velocity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/build: lib/kinematics_velocity/libkinematics_velocity.a

.PHONY : lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/build

lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/clean:
	cd /home/emanuelezanoni/programma_c/build/lib/kinematics_velocity && $(CMAKE_COMMAND) -P CMakeFiles/kinematics_velocity.dir/cmake_clean.cmake
.PHONY : lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/clean

lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/depend:
	cd /home/emanuelezanoni/programma_c/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/emanuelezanoni/programma_c /home/emanuelezanoni/programma_c/lib/kinematics_velocity /home/emanuelezanoni/programma_c/build /home/emanuelezanoni/programma_c/build/lib/kinematics_velocity /home/emanuelezanoni/programma_c/build/lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/kinematics_velocity/CMakeFiles/kinematics_velocity.dir/depend

