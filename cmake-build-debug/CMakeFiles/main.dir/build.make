# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.29

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

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

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "E:\clion\CLion 2024.2.3\bin\cmake\win\x64\bin\cmake.exe"

# The command to remove a file.
RM = "E:\clion\CLion 2024.2.3\bin\cmake\win\x64\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = E:\jixiebi\NE30_Controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = E:\jixiebi\NE30_Controller\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles\main.dir\depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles\main.dir\compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles\main.dir\progress.make

# Include the compile flags for this target's objects.
include CMakeFiles\main.dir\flags.make

CMakeFiles\main.dir\main.cpp.obj: CMakeFiles\main.dir\flags.make
CMakeFiles\main.dir\main.cpp.obj: E:\jixiebi\NE30_Controller\main.cpp
CMakeFiles\main.dir\main.cpp.obj: CMakeFiles\main.dir\compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=E:\jixiebi\NE30_Controller\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/main.cpp.obj"
	$(CMAKE_COMMAND) -E cmake_cl_compile_depends --dep-file=CMakeFiles\main.dir\main.cpp.obj.d --working-dir=E:\jixiebi\NE30_Controller\cmake-build-debug --filter-prefix="ע��: �����ļ�:  " -- D:\msvc\VC\Tools\MSVC\14.41.34120\bin\Hostx64\x64\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /showIncludes /FoCMakeFiles\main.dir\main.cpp.obj /FdCMakeFiles\main.dir\ /FS -c E:\jixiebi\NE30_Controller\main.cpp
<<

CMakeFiles\main.dir\main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cpp.i"
	D:\msvc\VC\Tools\MSVC\14.41.34120\bin\Hostx64\x64\cl.exe > CMakeFiles\main.dir\main.cpp.i @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E E:\jixiebi\NE30_Controller\main.cpp
<<

CMakeFiles\main.dir\main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cpp.s"
	D:\msvc\VC\Tools\MSVC\14.41.34120\bin\Hostx64\x64\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoNUL /FAs /FaCMakeFiles\main.dir\main.cpp.s /c E:\jixiebi\NE30_Controller\main.cpp
<<

# Object files for target main
main_OBJECTS = \
"CMakeFiles\main.dir\main.cpp.obj"

# External object files for target main
main_EXTERNAL_OBJECTS =

main.exe: CMakeFiles\main.dir\main.cpp.obj
main.exe: CMakeFiles\main.dir\build.make
main.exe: E:\jixiebi\NE30_Controller\3rdparty\innfos\lib\windows_x64\debug\actuatorControllerd.lib
main.exe: CMakeFiles\main.dir\objects1.rsp
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=E:\jixiebi\NE30_Controller\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable main.exe"
	"E:\clion\CLion 2024.2.3\bin\cmake\win\x64\bin\cmake.exe" -E vs_link_exe --intdir=CMakeFiles\main.dir --rc=C:\PROGRA~2\WI3CF2~1\10\bin\100226~1.0\x64\rc.exe --mt=C:\PROGRA~2\WI3CF2~1\10\bin\100226~1.0\x64\mt.exe --manifests -- D:\msvc\VC\Tools\MSVC\14.41.34120\bin\Hostx64\x64\link.exe /nologo @CMakeFiles\main.dir\objects1.rsp @<<
 /out:main.exe /implib:main.lib /pdb:E:\jixiebi\NE30_Controller\cmake-build-debug\main.pdb /version:0.0 /machine:x64 /debug /INCREMENTAL /subsystem:console  Ws2_32.lib E:\jixiebi\NE30_Controller\3rdparty\innfos\lib\windows_x64\debug\actuatorControllerd.lib kernel32.lib user32.lib gdi32.lib winspool.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comdlg32.lib advapi32.lib 
<<
	echo >nul && "E:\clion\CLion 2024.2.3\bin\cmake\win\x64\bin\cmake.exe" -E echo "Executable path: E:/jixiebi/NE30_Controller/cmake-build-debug"
	echo >nul && "E:\clion\CLion 2024.2.3\bin\cmake\win\x64\bin\cmake.exe" -E echo "CMAKE_BUILD_TYPE: Debug"
	echo >nul && "E:\clion\CLion 2024.2.3\bin\cmake\win\x64\bin\cmake.exe" -E copy_if_different  E:/jixiebi/NE30_Controller/3rdparty/innfos/lib/windows_x64/debug/actuatorControllerd.dll E:/jixiebi/NE30_Controller/cmake-build-debug

# Rule to build all files generated by this target.
CMakeFiles\main.dir\build: main.exe
.PHONY : CMakeFiles\main.dir\build

CMakeFiles\main.dir\clean:
	$(CMAKE_COMMAND) -P CMakeFiles\main.dir\cmake_clean.cmake
.PHONY : CMakeFiles\main.dir\clean

CMakeFiles\main.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" E:\jixiebi\NE30_Controller E:\jixiebi\NE30_Controller E:\jixiebi\NE30_Controller\cmake-build-debug E:\jixiebi\NE30_Controller\cmake-build-debug E:\jixiebi\NE30_Controller\cmake-build-debug\CMakeFiles\main.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles\main.dir\depend

