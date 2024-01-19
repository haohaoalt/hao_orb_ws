cmake_minimum_required(VERSION 3.10)
project(orb_slam3_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

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

#test
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -march=native")

#TBB
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../orb_slam3_lib/cmake_modules)

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

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
message(STATUS "Found TBB ${TBB_VERSION_MAJOR}.${TBB_VERSION_MINOR} (interface version ${TBB_INTERFACE_VERSION}) headers in: ${TBB_INCLUDE_DIRS}")
if (TBB_INTERFACE_VERSION LESS 11004)
    # enable global_control header for earlier TBB versions (Ubuntu 16.04, 18.04)
    add_definitions(-DTBB_PREVIEW_GLOBAL_CONTROL)
endif ()
find_package(CUDA REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../orb_slam3_lib/include
${PROJECT_SOURCE_DIR}/../orb_slam3_lib/include/CameraModels
${PROJECT_SOURCE_DIR}/../orb_slam3_lib/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${TBB_INCLUDE_DIR}
${CUDA_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )


find_package(catkin REQUIRED COMPONENTS
  orb_slam3_lib
  roscpp
  tf
  std_msgs
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  pcl_ros
  pcl_conversions
  message_generation
  octomap_ros
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# tensorrt
include_directories(/home/hayden/app/TensorRT-8.0.0.3/include)
include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/Thirdparty/DBoW2
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/Thirdparty/g2o
  ${PROJECT_SOURCE_DIR}/../orb_slam3_lib
  ${PROJECT_SOURCE_DIR}/../orb_slam3_lib/include

)
set(LIBS
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

# # Node for monocular camera
# add_executable(Mono
#   src/ros_mono.cc
#   src/ROSPublisher.cpp
# )

# add_dependencies(Mono ${${PROJECT_NAME}_EXPORTED_TARGETS})

# target_link_libraries(Mono
#   ${LIBS}
# )

# # Node for stereo camera
# add_executable(Stereo
#   src/ros_stereo.cc
#   src/ROSPublisher.cpp
# )

# add_dependencies(Stereo ${${PROJECT_NAME}_EXPORTED_TARGETS})

# target_link_libraries(Stereo
#   ${LIBS}
# )

# Node for RGB-D camera
add_executable(orb3RGBD
  src/ros_rgbd.cc
)
target_link_libraries(orb3RGBD
  ${LIBS}
)

# add_dependencies(Stereo ${${PROJECT_NAME}_EXPORTED_TARGETS})



# # Node for GPS path
#  add_executable(gps_to_rviz src/GPS/gps_to_rviz.cpp)
#  target_link_libraries(gps_to_rviz     ${catkin_LIBRARIES} )


