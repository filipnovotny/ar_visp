cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(ar_visp_common src/names.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()

FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

INCLUDE_DIRECTORIES(${OpenCV_DIR})

#ADD_EXECUTABLE( tracking main.cpp states.hpp events.h tracking.h tracking.cpp logfilewriter.hpp threading.h threading.cpp video.h video.cpp logitech.hpp)
#TARGET_LINK_LIBRARIES( tracking ${OpenCV_LIBS} qrcode_detector datamatrix_detector dmtx zbar boost_program_options cmd_line boost_thread)

rosbuild_add_executable(
ar_visp src/viz.cpp src/viz_main.cpp 
)
target_link_libraries(ar_visp ${OpenCV_LIBS} ar_visp_common visp_bridge)
rosbuild_link_boost(ar_visp thread signals program_options)