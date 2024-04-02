#cmake 版本
cmake_minimum_required(VERSION 3.10)
project(orb_slam3_lib)

# 设置颜色变量
set(CYAN_COLOR "\\033[1;36m")
set(RESET_COLOR "\\033[0m")

# 编译类型指定
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# 设置 C 语言编译器选项，
# 包括开启所有警告 -Wall、优化级别为 3 -O3，以及使用本地 CPU 架构 -march=native
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
# 设置 C++ 语言编译器选项
# 包括开启所有警告 -Wall、优化级别为 3 -O3，以及使用本地 CPU 架构 -march=native
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# 包含了一个 CMake 模块，用于检查编译器是否支持特定的编译选项
include(CheckCXXCompilerFlag)

# 分别检查编译器是否支持 C++14、C++11 和 C++0x（即 C++11 之前的标准）
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC14)
  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++0x.")
else(COMPILER_SUPPORTS_CXX0X)
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# 在大多数编译器中，-w 选项用于关闭警告信息。
# 这意味着编译器将不会生成任何警告信息，即使代码中存在潜在的问题或不规范的写法。
add_definitions(-w)

#opencv
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
    # 如果没有找到，尝试寻找 OpenCV 3.2
    find_package(OpenCV 3 QUIET)
    if(NOT OpenCV_FOUND)
        # 如果还是没有找到，报错
        message(FATAL_ERROR "OpenCV not found.")
    endif()
endif()
MESSAGE("${CYAN_COLOR}OPENCV VERSION:${RESET_COLOR}")
MESSAGE("${CYAN_COLOR}${OpenCV_VERSION}${RESET_COLOR}")

#eigen
find_package(Eigen3 REQUIRED)

#pangolin
find_package(Pangolin REQUIRED)

#PCL
find_package(PCL REQUIRED)

#TBB
find_package(TBB REQUIRED)

#cuda
find_package(CUDA REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#add_subdirectory() 函数用于向 CMake 构建系统添加一个子目录
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/yolov5_tensorrtx)

find_package(catkin REQUIRED  COMPONENTS roscpp std_msgs)
catkin_package(
  LIBRARIES orb_slam3_lib
)

include_directories(
  ${roscpp_INCLUDE_DIRS}   # ROS的C++客户端库roscpp的头文件所在的目录
  ${PROJECT_SOURCE_DIR}    # 根目录下的头文件目录
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${EIGEN3_INCLUDE_DIR}    # Eigen3 库的头文件目录
  ${Pangolin_INCLUDE_DIRS} # Pangolin 库的头文件目录
  ${PCL_INCLUDE_DIRS}      # PCL的头文件目录
  ${TBB_INCLUDE_DIR}       # TBB的头文件目录
  ${CUDA_INCLUDE_DIRS}     # CUDA是NVIDIA提供的并行计算框架
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
  /home/ylsdb20/app/TensorRT-8.2.1.8.Linux.x86_64-gnu.cuda-11.4.cudnn8.2/TensorRT-8.2.1.8/include
)

set(SRCS
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
  src/colorutils.cpp
)

set(LIBS
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${TBB_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/lib/libyolov5.so
  /home/ylsdb20/app/TensorRT-8.2.1.8.Linux.x86_64-gnu.cuda-11.4.cudnn8.2/TensorRT-8.2.1.8/targets/x86_64-linux-gnu/lib
  -lboost_system
  -lboost_serialization
  -lcrypto
)

add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME} ${LIBS})

