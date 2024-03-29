cmake_minimum_required (VERSION 2.8.3)
project(InstantMeshes)

option(INSTANT_MESHES_DEV_MODE "Instant meshes developer mode" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" U_CMAKE_BUILD_TYPE)

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui/ext/glfw")
  message(FATAL_ERROR "The Instant Meshes dependency repositories (NanoGUI, GLFW, etc.) are missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by calling \"git submodule update --init --recursive\"")
endif()

# Enable folders for projects in Visual Studio
if (CMAKE_GENERATOR MATCHES "Visual Studio")
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

# Sanitize build environment for static build with C++11
if (MSVC)
  add_definitions (/D "_CRT_SECURE_NO_WARNINGS")
  add_definitions (/D "__TBB_NO_IMPLICIT_LINKAGE")

  # Parallel build on MSVC (all targets)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

  if (NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")

    # Disable Eigen vectorization for Windows 32 bit builds (issues with unaligned access segfaults)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DEIGEN_DONT_ALIGN")
  endif()

  # Static build
  set(CompilerFlags
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
  foreach(CompilerFlag ${CompilerFlags})
    string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
  endforeach()
elseif(APPLE)
  # Try to auto-detect a suitable SDK
  # Commented out for now -- causes a too new SDK to be selected on AppVeyor
  #execute_process(COMMAND bash -c "xcodebuild -version -sdk | grep MacOSX | grep Path | head -n 1 | cut -f 2 -d ' '" OUTPUT_VARIABLE CMAKE_OSX_SYSROOT)
  #string(REGEX REPLACE "(\r?\n)+$" "" CMAKE_OSX_SYSROOT "${CMAKE_OSX_SYSROOT}")
  #string(REGEX REPLACE "^.*X([0-9.]*).sdk$" "\\1" CMAKE_OSX_DEPLOYMENT_TARGET "${CMAKE_OSX_SYSROOT}")
endif()

# Enable C++11 mode on GCC / Clang
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

# Build NanoGUI
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_SHARED  OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui ext_build/nanogui)
set_property(TARGET nanogui glfw glfw_objects bin2c PROPERTY FOLDER "dependencies")

# Build TBB
set(TBB_BUILD_STATIC ON CACHE BOOL " " FORCE)
set(TBB_BUILD_SHARED OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC_PROXY OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TESTS OFF CACHE BOOL " " FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/tbb ext_build/tbb)
set_property(TARGET tbb_static tbb_def_files PROPERTY FOLDER "dependencies")

# Compile instant meshes with various compiler warnings turned on
if(MSVC)
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-gnu-anonymous-struct -Wno-c99-extensions -Wno-nested-anon-types")
  endif()
endif()

# Header file directories of dependency libraries
include_directories(
  # NanoGUI
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui/include
  # NanoGUI dependencies (Eigen, GLFW, NanoVG, GLAD?)
  ${NANOGUI_EXTRA_INCS}
  # Intel Thread Building Blocks
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/tbb/include
  # Disjoint set data structure
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/dset
  # Parallel stable sort
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/pss
  # Pseudorandom number generator
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/pcg32
  # RPLY mesh I/O library
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/rply
  # Half precision type
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/half
  # Resource file
  ${CMAKE_CURRENT_BINARY_DIR}
  # Gurobi
  "C:/gurobi752/win32/include"
)

link_directories(
	"C:/gurobi752/win32/lib"
)

# Run bin2c on resource files
set(PNG_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/comb.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/delete_stroke.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/loadmesh.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/scare.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/sing_dir.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/sing_pos.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/instantmeshes.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/arrow.png
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/uvTexture.png)
file(GLOB FRAG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/*.frag")
file(GLOB VERT_FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/*.vert")
file(GLOB GEO_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/resources/*.geo")
set(IM_RESOURCES ${PNG_FILES} ${FRAG_FILES} ${VERT_FILES} ${GEO_FILES})

add_custom_command(
  OUTPUT resources.cpp
  COMMAND bin2c ARGS resources.cpp resources.h ${IM_RESOURCES}
  DEPENDS bin2c ${IM_RESOURCES}
  COMMENT "Running bin2c"
  PRE_BUILD VERBATIM)

if (INSTANT_MESHES_DEV_MODE)
  add_definitions(-DDEV_MODE)
endif()

# Resource file (icons etc.)
set(EXTRA_SOURCE "")
if (APPLE)
  set(EXTRA_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/resources/im.icns")
elseif(WIN32)
  set(EXTRA_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/resources/im.rc")
endif()

add_executable(InstantMeshes MACOSX_BUNDLE
  resources.cpp
  src/glutil.h src/glutil.cpp
  src/widgets.h src/widgets.cpp
  src/aabb.h
  src/main.cpp src/common.h
  src/meshio.h src/meshio.cpp
  src/normal.h src/normal.cpp
  src/adjacency.h src/adjacency.cpp
  src/meshstats.h src/meshstats.cpp
  src/hierarchy.h src/hierarchy.cpp
  src/extract.h src/extract.cpp
  src/viewer.h src/viewer.cpp
  src/field.h src/field.cpp
  src/bvh.h src/bvh.cpp
  src/subdivide.h src/subdivide.cpp
  src/reorder.h src/reorder.cpp
  src/serializer.h src/serializer.cpp
  src/batch.h src/batch.cpp
  src/smoothcurve.h src/smoothcurve.cpp
  src/cleanup.h src/cleanup.cpp
  src/dedge.h src/dedge.cpp
  ext/rply/rply.c
  src/core/cyPoint.h
  src/core/dual.cpp src/core/dual.h
  src/core/HE_Face.cpp src/core/HE_Face.h
  src/core/HE_Halfedge.cpp src/core/HE_Halfedge.h
  src/core/HE_Polyhedron.cpp src/core/HE_Polyhedron.h
  src/core/HE_Vertex.cpp src/core/HE_Vertex.h
  src/core/mesh_io.cpp
  src/core/polyhedron_iterators.h
  src/core/stitchmeshing.cpp
  src/core/HE_Vertex.cpp src/core/HE_Vertex.h
  src/core/util.cpp src/core/util.h
  src/meshio.cpp src/meshio.h
  ${EXTRA_SOURCE})

add_definitions(${NANOGUI_EXTRA_DEFS})
target_link_libraries(InstantMeshes tbb_static nanogui gurobi75 ${NANOGUI_EXTRA_LIBS}
									optimized gurobi_c++mt2015
									debug gurobi_c++mtd2015)

set_target_properties(InstantMeshes PROPERTIES OUTPUT_NAME "Instant Meshes")

if (WIN32)
  # Quench some warnings on MSVC
  if (MSVC)
    set_source_files_properties(ext/rply/rply.c PROPERTIES COMPILE_FLAGS "/wd4127")
  endif()
elseif (APPLE)
  # Build an application bundle on OSX
  set_target_properties(InstantMeshes PROPERTIES MACOSX_BUNDLE_BUNDLE_NAME "Instant Meshes")
  set_target_properties(InstantMeshes PROPERTIES MACOSX_BUNDLE_BUNDLE_GUI_IDENTIFIER "com.im.InstantMeshes")
  set_target_properties(InstantMeshes PROPERTIES MACOSX_BUNDLE_ICON_FILE im.icns)
  set_target_properties(InstantMeshes PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/resources/MacOSXBundleInfo.plist.in)
  set_source_files_properties(resources/im.icns PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
else()
  # Insulate from a few types of ABI changes by statically linking against libgcc and libstdc++
  set_target_properties(InstantMeshes PROPERTIES LINK_FLAGS "-static-libgcc")
  #set_target_properties(InstantMeshes PROPERTIES LINK_FLAGS "-static-libgcc -static-libstdc++")
endif()

if (UNIX AND NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
  add_custom_command(TARGET InstantMeshes POST_BUILD COMMAND strip $<TARGET_FILE:InstantMeshes>)
endif()

if (WIN32)
	# Quench some warnings on MSVC
	if (MSVC)
		file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/arrow.png
			DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/uvTexture.png
			DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
		file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/arrow.png
			DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
		file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/uvTexture.png
			DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
		file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/arrow.png
			DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)
		file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/uvTexture.png
			DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)
	endif()
endif()

ADD_CUSTOM_TARGET( datasets ALL  COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/datasets ${CMAKE_CURRENT_BINARY_DIR}/datasets )

add_dependencies(InstantMeshes datasets)
