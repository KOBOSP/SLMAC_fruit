cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
    #  SET(CMAKE_BUILD_TYPE Debug)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
#
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -W -fsanitize=address -fno-omit-frame-pointer")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -fsanitize=address -fno-omit-frame-pointer")
#-fsanitize=address
#-fno-omit-frame-pointer     #打印函数调用路径
#-fsanitize-recover=address  #AddressSanitizer遇到错误时能够继续
#-w的意思是关闭编译时的警告，也就是编译后不显示任何warning，因为有时在编译之后编译器会显示一些例如数据转换之类的警告，这些警告是我们平时可以忽略的。
#-W选项类似-Wall，会显示警告，但是只显示编译器认为会出现错误的警告。
#-Wall选项意思是编译后显示所有警告

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

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


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 要保证整个工程的opencv版本一致，包括dbow，源码以及ros相关的
# 3 4 都可以正常运行
find_package(OpenCV 3.4)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.4 not found.")
endif ()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
#find_package(Pangolin REQUIRED)
#find_package(realsense2)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${PROJECT_SOURCE_DIR}/../ORB3Thirdparty/Pangolin/include
        ${PROJECT_SOURCE_DIR}/../ORB3Thirdparty/Pangolin/build/src/include
        ${PROJECT_SOURCE_DIR}/../ORB3Thirdparty/DBoW2
        ${PROJECT_SOURCE_DIR}/../ORB3Thirdparty/g2o
        ${PROJECT_SOURCE_DIR}/../ORB3Thirdparty/Sophus
        ${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Atlas.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Viewer.cc
        src/ImuTypes.cc
        src/G2oTypes.cc
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/MLPnPsolver.cpp
        src/GeometricTools.cc
        src/Settings.cc
        include/System.h
        include/Tracking.h
        include/LocalMapping.h
        include/LoopClosing.h
        include/ORBextractor.h
        include/ORBmatcher.h
        include/FrameDrawer.h
        include/Converter.h
        include/MapPoint.h
        include/KeyFrame.h
        include/Atlas.h
        include/Map.h
        include/MapDrawer.h
        include/Optimizer.h
        include/Frame.h
        include/KeyFrameDatabase.h
        include/Sim3Solver.h
        include/Viewer.h
        include/ImuTypes.h
        include/G2oTypes.h
        include/CameraModels/GeometricCamera.h
        include/CameraModels/Pinhole.h
        include/CameraModels/KannalaBrandt8.h
        include/MLPnPsolver.h
        include/GeometricTools.h
        include/SerializationUtils.h
        include/Settings.h)


target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PROJECT_SOURCE_DIR}/../ORB3Thirdparty/Pangolin/build/src/libpangolin.so
        ${PROJECT_SOURCE_DIR}/../ORB3Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/../ORB3Thirdparty/g2o/lib/libg2o.so
        -lboost_serialization
        -lcrypto
        GL
        -lGLEW
        )

#Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/StereoImuOnEuroc)
add_executable(stereo_inertial_euroc
        Examples/StereoImuOnEuroc/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})


