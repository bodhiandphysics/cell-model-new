# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.23.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.23.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/markholum/Programming/cellmodel-new

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/markholum/Programming/cellmodel-new/build

# Include any dependencies generated for this target.
include CMakeFiles/cellmodel.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/cellmodel.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/cellmodel.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cellmodel.dir/flags.make

CMakeFiles/cellmodel.dir/libs/givio.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/givio.cpp.o: ../libs/givio.cpp
CMakeFiles/cellmodel.dir/libs/givio.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cellmodel.dir/libs/givio.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/givio.cpp.o -MF CMakeFiles/cellmodel.dir/libs/givio.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/givio.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/givio.cpp

CMakeFiles/cellmodel.dir/libs/givio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/givio.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/givio.cpp > CMakeFiles/cellmodel.dir/libs/givio.cpp.i

CMakeFiles/cellmodel.dir/libs/givio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/givio.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/givio.cpp -o CMakeFiles/cellmodel.dir/libs/givio.cpp.s

CMakeFiles/cellmodel.dir/libs/givr.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/givr.cpp.o: ../libs/givr.cpp
CMakeFiles/cellmodel.dir/libs/givr.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/cellmodel.dir/libs/givr.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/givr.cpp.o -MF CMakeFiles/cellmodel.dir/libs/givr.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/givr.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/givr.cpp

CMakeFiles/cellmodel.dir/libs/givr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/givr.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/givr.cpp > CMakeFiles/cellmodel.dir/libs/givr.cpp.i

CMakeFiles/cellmodel.dir/libs/givr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/givr.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/givr.cpp -o CMakeFiles/cellmodel.dir/libs/givr.cpp.s

CMakeFiles/cellmodel.dir/libs/glad.c.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/glad.c.o: ../libs/glad.c
CMakeFiles/cellmodel.dir/libs/glad.c.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/cellmodel.dir/libs/glad.c.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/glad.c.o -MF CMakeFiles/cellmodel.dir/libs/glad.c.o.d -o CMakeFiles/cellmodel.dir/libs/glad.c.o -c /Users/markholum/Programming/cellmodel-new/libs/glad.c

CMakeFiles/cellmodel.dir/libs/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cellmodel.dir/libs/glad.c.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/glad.c > CMakeFiles/cellmodel.dir/libs/glad.c.i

CMakeFiles/cellmodel.dir/libs/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cellmodel.dir/libs/glad.c.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/glad.c -o CMakeFiles/cellmodel.dir/libs/glad.c.s

CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o: ../libs/imgui/imgui.cpp
CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o -MF CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui.cpp

CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui.cpp > CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.i

CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui.cpp -o CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.s

CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o: ../libs/imgui/imgui_demo.cpp
CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o -MF CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_demo.cpp

CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_demo.cpp > CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.i

CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_demo.cpp -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.s

CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o: ../libs/imgui/imgui_draw.cpp
CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o -MF CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_draw.cpp

CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_draw.cpp > CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.i

CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_draw.cpp -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.s

CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o: ../libs/imgui/imgui_impl_glfw.cpp
CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o -MF CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_impl_glfw.cpp

CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_impl_glfw.cpp > CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.i

CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_impl_glfw.cpp -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.s

CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o: ../libs/imgui/imgui_impl_opengl3.cpp
CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o -MF CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_impl_opengl3.cpp

CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_impl_opengl3.cpp > CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.i

CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_impl_opengl3.cpp -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.s

CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o: ../libs/imgui/imgui_tables.cpp
CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o -MF CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_tables.cpp

CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_tables.cpp > CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.i

CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_tables.cpp -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.s

CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o: ../libs/imgui/imgui_widgets.cpp
CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o -MF CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_widgets.cpp

CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_widgets.cpp > CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.i

CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/imgui/imgui_widgets.cpp -o CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.s

CMakeFiles/cellmodel.dir/libs/panel.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/panel.cpp.o: ../libs/panel.cpp
CMakeFiles/cellmodel.dir/libs/panel.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/cellmodel.dir/libs/panel.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/panel.cpp.o -MF CMakeFiles/cellmodel.dir/libs/panel.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/panel.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/panel.cpp

CMakeFiles/cellmodel.dir/libs/panel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/panel.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/panel.cpp > CMakeFiles/cellmodel.dir/libs/panel.cpp.i

CMakeFiles/cellmodel.dir/libs/panel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/panel.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/panel.cpp -o CMakeFiles/cellmodel.dir/libs/panel.cpp.s

CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o: ../libs/picking_controls.cpp
CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o -MF CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o.d -o CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o -c /Users/markholum/Programming/cellmodel-new/libs/picking_controls.cpp

CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/libs/picking_controls.cpp > CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.i

CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/libs/picking_controls.cpp -o CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.s

CMakeFiles/cellmodel.dir/src/cell.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/src/cell.cpp.o: ../src/cell.cpp
CMakeFiles/cellmodel.dir/src/cell.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/cellmodel.dir/src/cell.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/src/cell.cpp.o -MF CMakeFiles/cellmodel.dir/src/cell.cpp.o.d -o CMakeFiles/cellmodel.dir/src/cell.cpp.o -c /Users/markholum/Programming/cellmodel-new/src/cell.cpp

CMakeFiles/cellmodel.dir/src/cell.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/src/cell.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/src/cell.cpp > CMakeFiles/cellmodel.dir/src/cell.cpp.i

CMakeFiles/cellmodel.dir/src/cell.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/src/cell.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/src/cell.cpp -o CMakeFiles/cellmodel.dir/src/cell.cpp.s

CMakeFiles/cellmodel.dir/src/main.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/cellmodel.dir/src/main.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/cellmodel.dir/src/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/src/main.cpp.o -MF CMakeFiles/cellmodel.dir/src/main.cpp.o.d -o CMakeFiles/cellmodel.dir/src/main.cpp.o -c /Users/markholum/Programming/cellmodel-new/src/main.cpp

CMakeFiles/cellmodel.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/src/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/src/main.cpp > CMakeFiles/cellmodel.dir/src/main.cpp.i

CMakeFiles/cellmodel.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/src/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/src/main.cpp -o CMakeFiles/cellmodel.dir/src/main.cpp.s

CMakeFiles/cellmodel.dir/src/sim.cpp.o: CMakeFiles/cellmodel.dir/flags.make
CMakeFiles/cellmodel.dir/src/sim.cpp.o: ../src/sim.cpp
CMakeFiles/cellmodel.dir/src/sim.cpp.o: CMakeFiles/cellmodel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/cellmodel.dir/src/sim.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cellmodel.dir/src/sim.cpp.o -MF CMakeFiles/cellmodel.dir/src/sim.cpp.o.d -o CMakeFiles/cellmodel.dir/src/sim.cpp.o -c /Users/markholum/Programming/cellmodel-new/src/sim.cpp

CMakeFiles/cellmodel.dir/src/sim.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cellmodel.dir/src/sim.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/markholum/Programming/cellmodel-new/src/sim.cpp > CMakeFiles/cellmodel.dir/src/sim.cpp.i

CMakeFiles/cellmodel.dir/src/sim.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cellmodel.dir/src/sim.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/markholum/Programming/cellmodel-new/src/sim.cpp -o CMakeFiles/cellmodel.dir/src/sim.cpp.s

# Object files for target cellmodel
cellmodel_OBJECTS = \
"CMakeFiles/cellmodel.dir/libs/givio.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/givr.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/glad.c.o" \
"CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/panel.cpp.o" \
"CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o" \
"CMakeFiles/cellmodel.dir/src/cell.cpp.o" \
"CMakeFiles/cellmodel.dir/src/main.cpp.o" \
"CMakeFiles/cellmodel.dir/src/sim.cpp.o"

# External object files for target cellmodel
cellmodel_EXTERNAL_OBJECTS =

cellmodel: CMakeFiles/cellmodel.dir/libs/givio.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/givr.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/glad.c.o
cellmodel: CMakeFiles/cellmodel.dir/libs/imgui/imgui.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/imgui/imgui_demo.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/imgui/imgui_draw.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_glfw.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/imgui/imgui_impl_opengl3.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/imgui/imgui_tables.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/imgui/imgui_widgets.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/panel.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/libs/picking_controls.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/src/cell.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/src/main.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/src/sim.cpp.o
cellmodel: CMakeFiles/cellmodel.dir/build.make
cellmodel: libs/glfw/src/libglfw3.a
cellmodel: CMakeFiles/cellmodel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/markholum/Programming/cellmodel-new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Linking CXX executable cellmodel"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cellmodel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cellmodel.dir/build: cellmodel
.PHONY : CMakeFiles/cellmodel.dir/build

CMakeFiles/cellmodel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cellmodel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cellmodel.dir/clean

CMakeFiles/cellmodel.dir/depend:
	cd /Users/markholum/Programming/cellmodel-new/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/markholum/Programming/cellmodel-new /Users/markholum/Programming/cellmodel-new /Users/markholum/Programming/cellmodel-new/build /Users/markholum/Programming/cellmodel-new/build /Users/markholum/Programming/cellmodel-new/build/CMakeFiles/cellmodel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cellmodel.dir/depend

