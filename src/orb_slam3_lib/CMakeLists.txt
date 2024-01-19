cmake_minimum_required(VERSION 2.8)
project(orb_slam3_lib)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++11.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()



# add_subdirectory(Thirdparty/DBoW2)
# add_subdirectory(Thirdparty/g2o)

# set(OpenCV_DIR "/home/slam007/app/opencv3.4.5/install/share/OpenCV")
# 寻找 OpenCV 4.2
find_package(OpenCV 4.2 QUIET)
if(NOT OpenCV_FOUND)
    # 如果没有找到，尝试寻找 OpenCV 3.2
    find_package(OpenCV 3.2 QUIET)
    if(NOT OpenCV_FOUND)
        # 如果还是没有找到，报错
        message(FATAL_ERROR "OpenCV not found.")
    endif()
endif()

# eigen
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)
find_package(PCL REQUIRED)

# TBB
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(TBB REQUIRED)
message(STATUS "Found TBB ${TBB_VERSION_MAJOR}.${TBB_VERSION_MINOR} (interface version ${TBB_INTERFACE_VERSION}) headers in: ${TBB_INCLUDE_DIRS}")
if (TBB_INTERFACE_VERSION LESS 11004)
    # enable global_control header for earlier TBB versions (Ubuntu 16.04, 18.04)
    add_definitions(-DTBB_PREVIEW_GLOBAL_CONTROL)
endif ()

# cuda
find_package(CUDA REQUIRED)

#libboost_serialization is essential for map save / load
# find_library(BOOST_SERIALIZATION boost_serialization)
# if (NOT BOOST_SERIALIZATION)
#     message(FATAL_ERROR "Can't find libboost_serialization")
# endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../orb_slam2_lib/Thirdparty/DBoW2
${PROJECT_SOURCE_DIR}/../orb_slam2_lib/Thirdparty/g2o
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${TBB_INCLUDE_DIR}
${CUDA_INCLUDE_DIRS}
)
# tensorrt
include_directories(/home/hayden/app/TensorRT-8.0.0.3/include)
link_directories(/home/hayden/app/TensorRT-8.0.0.3/targets/x86_64-linux-gnu/lib)

include_directories(${CMAKE_BINARY_DIR}/devel/include) # 引用ros生成的msg header文件

MESSAGE(STATUS "OpenCV version and OpenCV_LIBS: " ${OpenCV_VERSION} ${OpenCV_LIBS})
MESSAGE(STATUS "PROJECT_SOURCE_DIR" ${PROJECT_SOURCE_DIR})
MESSAGE(STATUS "EIGEN_INCLUDE_DIR" ${EIGEN3_INCLUDE_DIR})

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
find_package(catkin REQUIRED)
catkin_package(
  LIBRARIES orb_slam3_lib
  )

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
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
src/YoloDetector.cc
src/PointCloudMapping.cc
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
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h
include/YoloDetector.h
include/PointCloudMapping.h)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../devel/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../orb_slam2_lib/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../orb_slam3_lib/Thirdparty/yolov5_tensorrtx/lib/libyolov5.so
${PCL_LIBRARIES}
${TBB_LIBRARIES}
${CUDA_LIBRARIES}
# ${BOOST_SERIALIZATION}
-lboost_system
-lboost_serialization
-lcrypto
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


add_definitions(-DEIGEN_DONT_PARALLELIZE)

# Build examples

# RGB-D examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

# add_executable(rgbd_tum
#         Examples/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbd_tum ${PROJECT_NAME})





# find_package(catkin REQUIRED)
# catkin_package(
#   LIBRARIES orb_slam2_lib
# )

# set(SRCS
#   src/System.cc
#   src/Tracking.cc
#   src/LocalMapping.cc
#   src/LoopClosing.cc
#   src/ORBextractor.cc
#   src/ORBmatcher.cc
#   src/FrameDrawer.cc
#   src/Converter.cc
#   src/MapPoint.cc
#   src/KeyFrame.cc
#   src/Map.cc
#   src/IMapPublisher.cc
#   src/DefaultImpls.cc
#   src/IPublisherThread.cc
#   src/Optimizer.cc
#   src/PnPsolver.cc
#   src/Frame.cc
#   src/KeyFrameDatabase.cc
#   src/Sim3Solver.cc
#   src/Initializer.cc
#   src/Object.cpp
# )

# set(LIBS
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
#   DBoW2
#   g2o
# )

# add_library(${PROJECT_NAME} SHARED ${SRCS})
# target_link_libraries(${PROJECT_NAME} ${LIBS})




# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
# add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
# target_link_libraries(bin_vocabulary DBoW2 ${OpenCV_LIBS})
