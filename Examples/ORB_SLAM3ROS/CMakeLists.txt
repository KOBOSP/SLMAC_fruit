cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(StereoImuRos)

rosbuild_init()

IF (NOT ROS_BUILD_TYPE)
    SET(ROS_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)

# 要保证整个工程的opencv版本一致，包括dbow，源码以及ros相关的
# 3 4 都可以正常运行
find_package(OpenCV 3.4.3)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.4 not found.")
endif ()
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        tf
        cv_bridge
        sensor_msgs
        image_transport
        )
#catkin_package()

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/../../
        ${PROJECT_SOURCE_DIR}/../../include
        ${PROJECT_SOURCE_DIR}/../../include/CameraModels
        ${PROJECT_SOURCE_DIR}/../../../ORB3Thirdparty/Pangolin/include
        ${PROJECT_SOURCE_DIR}/../../../ORB3Thirdparty/Pangolin/build/src/include
        ${PROJECT_SOURCE_DIR}/../../../ORB3Thirdparty/DBoW2
        ${PROJECT_SOURCE_DIR}/../../../ORB3Thirdparty/g2o
        ${PROJECT_SOURCE_DIR}/../../../ORB3Thirdparty/Sophus
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)
set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${catkin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/../../../ORB3Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/../../../ORB3Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/../../../ORB3Thirdparty/Pangolin/build/src/libpangolin.so
        ${PROJECT_SOURCE_DIR}/../../lib/libORB_SLAM3.so
        -lboost_system
        -lcrypto
        GL
        -lGLEW
        )

rosbuild_add_executable(StereoImuRos
        src/StereoImuRos.cc
        )

target_link_libraries(StereoImuRos
        ${LIBS}
        )
