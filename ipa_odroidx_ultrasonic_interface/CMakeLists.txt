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
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(CommPortDriver ros/src/UARTDriver.cpp)

rosbuild_add_executable(us_driver ros/src/us_driver.cpp)
target_link_libraries(us_driver CommPortDriver)


#set(AVRLIB_URL "http://www.procyonengineering.com/embedded/avr/avrlib/avrlib.zip")
#set(AVRLIB_DOWNLOAD_PATH "/tmp/avrlib.zip")
#set(AVRLIB_EXTRACTED_DIR "..")

#if (NOT EXISTS "${AVRLIB_DOWNLOAD_PATH}")
#    file(DOWNLOAD "${AVRLIB_URL}" "${AVRLIB_DOWNLOAD_PATH}")
#    execute_process(COMMAND unzip -o "${AVRLIB_DOWNLOAD_PATH}" -d "${AVRLIB_EXTRACTED_DIR}")
#endif()

execute_process(COMMAND bash -c "cd ../mc && make")
