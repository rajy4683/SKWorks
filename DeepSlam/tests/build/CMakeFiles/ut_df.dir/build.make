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
CMAKE_SOURCE_DIR = /root/cuda_drivers/DeepFactors/tests

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/cuda_drivers/DeepFactors/tests/build

# Include any dependencies generated for this target.
include CMakeFiles/ut_df.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ut_df.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ut_df.dir/flags.make

CMakeFiles/ut_df.dir/main.o: CMakeFiles/ut_df.dir/flags.make
CMakeFiles/ut_df.dir/main.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/cuda_drivers/DeepFactors/tests/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ut_df.dir/main.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ut_df.dir/main.o -c /root/cuda_drivers/DeepFactors/tests/main.cpp

CMakeFiles/ut_df.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ut_df.dir/main.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/cuda_drivers/DeepFactors/tests/main.cpp > CMakeFiles/ut_df.dir/main.i

CMakeFiles/ut_df.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ut_df.dir/main.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/cuda_drivers/DeepFactors/tests/main.cpp -o CMakeFiles/ut_df.dir/main.s

CMakeFiles/ut_df.dir/main.o.requires:

.PHONY : CMakeFiles/ut_df.dir/main.o.requires

CMakeFiles/ut_df.dir/main.o.provides: CMakeFiles/ut_df.dir/main.o.requires
	$(MAKE) -f CMakeFiles/ut_df.dir/build.make CMakeFiles/ut_df.dir/main.o.provides.build
.PHONY : CMakeFiles/ut_df.dir/main.o.provides

CMakeFiles/ut_df.dir/main.o.provides.build: CMakeFiles/ut_df.dir/main.o


CMakeFiles/ut_df.dir/ut_decoder.o: CMakeFiles/ut_df.dir/flags.make
CMakeFiles/ut_df.dir/ut_decoder.o: ../ut_decoder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/cuda_drivers/DeepFactors/tests/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ut_df.dir/ut_decoder.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ut_df.dir/ut_decoder.o -c /root/cuda_drivers/DeepFactors/tests/ut_decoder.cpp

CMakeFiles/ut_df.dir/ut_decoder.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ut_df.dir/ut_decoder.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/cuda_drivers/DeepFactors/tests/ut_decoder.cpp > CMakeFiles/ut_df.dir/ut_decoder.i

CMakeFiles/ut_df.dir/ut_decoder.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ut_df.dir/ut_decoder.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/cuda_drivers/DeepFactors/tests/ut_decoder.cpp -o CMakeFiles/ut_df.dir/ut_decoder.s

CMakeFiles/ut_df.dir/ut_decoder.o.requires:

.PHONY : CMakeFiles/ut_df.dir/ut_decoder.o.requires

CMakeFiles/ut_df.dir/ut_decoder.o.provides: CMakeFiles/ut_df.dir/ut_decoder.o.requires
	$(MAKE) -f CMakeFiles/ut_df.dir/build.make CMakeFiles/ut_df.dir/ut_decoder.o.provides.build
.PHONY : CMakeFiles/ut_df.dir/ut_decoder.o.provides

CMakeFiles/ut_df.dir/ut_decoder.o.provides.build: CMakeFiles/ut_df.dir/ut_decoder.o


CMakeFiles/ut_df.dir/ut_cuda_utils.o: CMakeFiles/ut_df.dir/flags.make
CMakeFiles/ut_df.dir/ut_cuda_utils.o: ../ut_cuda_utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/cuda_drivers/DeepFactors/tests/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ut_df.dir/ut_cuda_utils.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ut_df.dir/ut_cuda_utils.o -c /root/cuda_drivers/DeepFactors/tests/ut_cuda_utils.cpp

CMakeFiles/ut_df.dir/ut_cuda_utils.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ut_df.dir/ut_cuda_utils.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/cuda_drivers/DeepFactors/tests/ut_cuda_utils.cpp > CMakeFiles/ut_df.dir/ut_cuda_utils.i

CMakeFiles/ut_df.dir/ut_cuda_utils.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ut_df.dir/ut_cuda_utils.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/cuda_drivers/DeepFactors/tests/ut_cuda_utils.cpp -o CMakeFiles/ut_df.dir/ut_cuda_utils.s

CMakeFiles/ut_df.dir/ut_cuda_utils.o.requires:

.PHONY : CMakeFiles/ut_df.dir/ut_cuda_utils.o.requires

CMakeFiles/ut_df.dir/ut_cuda_utils.o.provides: CMakeFiles/ut_df.dir/ut_cuda_utils.o.requires
	$(MAKE) -f CMakeFiles/ut_df.dir/build.make CMakeFiles/ut_df.dir/ut_cuda_utils.o.provides.build
.PHONY : CMakeFiles/ut_df.dir/ut_cuda_utils.o.provides

CMakeFiles/ut_df.dir/ut_cuda_utils.o.provides.build: CMakeFiles/ut_df.dir/ut_cuda_utils.o


CMakeFiles/ut_df.dir/ut_se3aligner.o: CMakeFiles/ut_df.dir/flags.make
CMakeFiles/ut_df.dir/ut_se3aligner.o: ../ut_se3aligner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/cuda_drivers/DeepFactors/tests/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ut_df.dir/ut_se3aligner.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ut_df.dir/ut_se3aligner.o -c /root/cuda_drivers/DeepFactors/tests/ut_se3aligner.cpp

CMakeFiles/ut_df.dir/ut_se3aligner.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ut_df.dir/ut_se3aligner.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/cuda_drivers/DeepFactors/tests/ut_se3aligner.cpp > CMakeFiles/ut_df.dir/ut_se3aligner.i

CMakeFiles/ut_df.dir/ut_se3aligner.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ut_df.dir/ut_se3aligner.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/cuda_drivers/DeepFactors/tests/ut_se3aligner.cpp -o CMakeFiles/ut_df.dir/ut_se3aligner.s

CMakeFiles/ut_df.dir/ut_se3aligner.o.requires:

.PHONY : CMakeFiles/ut_df.dir/ut_se3aligner.o.requires

CMakeFiles/ut_df.dir/ut_se3aligner.o.provides: CMakeFiles/ut_df.dir/ut_se3aligner.o.requires
	$(MAKE) -f CMakeFiles/ut_df.dir/build.make CMakeFiles/ut_df.dir/ut_se3aligner.o.provides.build
.PHONY : CMakeFiles/ut_df.dir/ut_se3aligner.o.provides

CMakeFiles/ut_df.dir/ut_se3aligner.o.provides.build: CMakeFiles/ut_df.dir/ut_se3aligner.o


CMakeFiles/ut_df.dir/ut_warping.o: CMakeFiles/ut_df.dir/flags.make
CMakeFiles/ut_df.dir/ut_warping.o: ../ut_warping.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/cuda_drivers/DeepFactors/tests/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/ut_df.dir/ut_warping.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ut_df.dir/ut_warping.o -c /root/cuda_drivers/DeepFactors/tests/ut_warping.cpp

CMakeFiles/ut_df.dir/ut_warping.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ut_df.dir/ut_warping.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/cuda_drivers/DeepFactors/tests/ut_warping.cpp > CMakeFiles/ut_df.dir/ut_warping.i

CMakeFiles/ut_df.dir/ut_warping.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ut_df.dir/ut_warping.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/cuda_drivers/DeepFactors/tests/ut_warping.cpp -o CMakeFiles/ut_df.dir/ut_warping.s

CMakeFiles/ut_df.dir/ut_warping.o.requires:

.PHONY : CMakeFiles/ut_df.dir/ut_warping.o.requires

CMakeFiles/ut_df.dir/ut_warping.o.provides: CMakeFiles/ut_df.dir/ut_warping.o.requires
	$(MAKE) -f CMakeFiles/ut_df.dir/build.make CMakeFiles/ut_df.dir/ut_warping.o.provides.build
.PHONY : CMakeFiles/ut_df.dir/ut_warping.o.provides

CMakeFiles/ut_df.dir/ut_warping.o.provides.build: CMakeFiles/ut_df.dir/ut_warping.o


# Object files for target ut_df
ut_df_OBJECTS = \
"CMakeFiles/ut_df.dir/main.o" \
"CMakeFiles/ut_df.dir/ut_decoder.o" \
"CMakeFiles/ut_df.dir/ut_cuda_utils.o" \
"CMakeFiles/ut_df.dir/ut_se3aligner.o" \
"CMakeFiles/ut_df.dir/ut_warping.o"

# External object files for target ut_df
ut_df_EXTERNAL_OBJECTS =

ut_df: CMakeFiles/ut_df.dir/main.o
ut_df: CMakeFiles/ut_df.dir/ut_decoder.o
ut_df: CMakeFiles/ut_df.dir/ut_cuda_utils.o
ut_df: CMakeFiles/ut_df.dir/ut_se3aligner.o
ut_df: CMakeFiles/ut_df.dir/ut_warping.o
ut_df: CMakeFiles/ut_df.dir/build.make
ut_df: lib/libgtest.a
ut_df: CMakeFiles/ut_df.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/cuda_drivers/DeepFactors/tests/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable ut_df"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ut_df.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ut_df.dir/build: ut_df

.PHONY : CMakeFiles/ut_df.dir/build

CMakeFiles/ut_df.dir/requires: CMakeFiles/ut_df.dir/main.o.requires
CMakeFiles/ut_df.dir/requires: CMakeFiles/ut_df.dir/ut_decoder.o.requires
CMakeFiles/ut_df.dir/requires: CMakeFiles/ut_df.dir/ut_cuda_utils.o.requires
CMakeFiles/ut_df.dir/requires: CMakeFiles/ut_df.dir/ut_se3aligner.o.requires
CMakeFiles/ut_df.dir/requires: CMakeFiles/ut_df.dir/ut_warping.o.requires

.PHONY : CMakeFiles/ut_df.dir/requires

CMakeFiles/ut_df.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ut_df.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ut_df.dir/clean

CMakeFiles/ut_df.dir/depend:
	cd /root/cuda_drivers/DeepFactors/tests/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/cuda_drivers/DeepFactors/tests /root/cuda_drivers/DeepFactors/tests /root/cuda_drivers/DeepFactors/tests/build /root/cuda_drivers/DeepFactors/tests/build /root/cuda_drivers/DeepFactors/tests/build/CMakeFiles/ut_df.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ut_df.dir/depend
