cmake_minimum_required(VERSION 3.10)
project(orb_slam3_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

#test
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -march=native")

ADD_DEFINITIONS("-std=c++14")
# 在大多数编译器中，-w 选项用于关闭警告信息。
# 这意味着编译器将不会生成任何警告信息，即使代码中存在潜在的问题或不规范的写法。
add_definitions(-w)

# cuda
find_package(CUDA REQUIRED)
# OpenCV 4.2
find_package(OpenCV 4.2 QUIET)
if(NOT OpenCV_FOUND)
    # 如果没有找到，尝试寻找 OpenCV 3.2
    find_package(OpenCV 3.2 QUIET)
    if(NOT OpenCV_FOUND)
        # 如果还是没有找到，报错
        message(FATAL_ERROR "OpenCV not found.")
    endif()
endif()

find_package(Pangolin REQUIRED)

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

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  ORBState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../orb_slam3_lib
  ${PROJECT_SOURCE_DIR}/../orb_slam3_lib/include
  ${PROJECT_SOURCE_DIR}/../orb_slam3_lib/include/CameraModels
  ${PROJECT_SOURCE_DIR}/../orb_slam3_lib/Thirdparty/Sophus
  /home/ylsdb20/app/TensorRT-8.2.1.8.Linux.x86_64-gnu.cuda-11.4.cudnn8.2/TensorRT-8.2.1.8/include
)
set(LIBS
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${TORCH_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${TBB_LIBRARIES}
  ${CUDA_LIBRARIES}
  /home/ylsdb20/app/TensorRT-8.2.1.8.Linux.x86_64-gnu.cuda-11.4.cudnn8.2/TensorRT-8.2.1.8/targets/x86_64-linux-gnu/lib
  -lboost_system
  -lboost_serialization
  -lcrypto
)

# Node for RGB-D camera
add_executable(RGBD
  src/ros_rgbd.cc
  src/colorutils.cpp
)
target_link_libraries(RGBD
  ${LIBS}
)
# Node for GPS path
 add_executable(gps_to_rviz src/GPS/gps_to_rviz.cpp)
 target_link_libraries(gps_to_rviz     ${catkin_LIBRARIES} )