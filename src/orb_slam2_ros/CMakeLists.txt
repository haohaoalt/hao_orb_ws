cmake_minimum_required(VERSION 3.10)
project(orb_slam2_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

ADD_DEFINITIONS("-std=c++11")
ADD_DEFINITIONS(-w)

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

# find_package(Pangolin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  orb_slam2_lib
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

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  ORBState.msg
  mutimap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
# 查找 Boost 库
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/include

)
set(LIBS
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

# Node for monocular camera
add_executable(Mono
  src/ros_mono.cc
  src/ROSPublisher.cpp
)

add_dependencies(Mono ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(Mono
  ${LIBS}
)

# Node for stereo camera
add_executable(Stereo
  src/ros_stereo.cc
  src/ROSPublisher.cpp
)

add_dependencies(Stereo ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(Stereo
  ${LIBS}
)

# Node for stereo and gnss camera
add_executable(Stereo_gnss
  src/ros_stereoAndgnss.cc
  src/ROSPublisher.cpp
)

add_dependencies(Stereo_gnss ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(Stereo_gnss
  ${LIBS}
)

# Node for stereo kitti camera
add_executable(Stereo_kitti
  src/ros_stereo_kitti.cc
  src/ROSPublisher.cpp
)

add_dependencies(Stereo_kitti ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(Stereo_kitti
  ${LIBS}
)

# Node for RGB-D camera
add_executable(RGBD
  src/ros_rgbd.cc
  src/ROSPublisher.cpp
  src/colorutils.cpp
)

add_dependencies(Stereo ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(RGBD
  ${LIBS}
)

# Node for GPS path
 add_executable(gps_to_rviz src/GPS/gps_to_rviz.cpp)
add_dependencies(gps_to_rviz ${${PROJECT_NAME}_EXPORTED_TARGETS})
 target_link_libraries(gps_to_rviz  ${LIBS} ${Boost_LIBRARIES})


