cmake_minimum_required(VERSION 3.10)
project(drone_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs interactive_markers quadrotor_msgs)
# 用老的 FindCUDA（3.16 自带）
find_package(CUDA REQUIRED)

# TensorRT（Jetson 常见路径）
set(TENSORRT_ROOT "/usr" CACHE PATH "TensorRT root")
find_path(TENSORRT_INCLUDE NvInfer.h
  HINTS $ENV{TENSORRT_DIR}/include ${TENSORRT_ROOT}/include /usr/include/aarch64-linux-gnu)
find_library(NVINFER nvinfer
  HINTS $ENV{TENSORRT_DIR}/lib ${TENSORRT_ROOT}/lib /usr/lib/aarch64-linux-gnu)
find_library(NVINFER_PLUGIN nvinfer_plugin
  HINTS $ENV{TENSORRT_DIR}/lib ${TENSORRT_ROOT}/lib /usr/lib/aarch64-linux-gnu)
find_library(NVONNXPARSER nvonnxparser
  HINTS $ENV{TENSORRT_DIR}/lib ${TENSORRT_ROOT}/lib /usr/lib/aarch64-linux-gnu)
if(NOT TENSORRT_INCLUDE OR NOT NVINFER OR NOT NVINFER_PLUGIN)
  message(FATAL_ERROR "TensorRT not found. Set TENSORRT_ROOT or TENSORRT_DIR properly.")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trtcore obsqueue
  CATKIN_DEPENDS roscpp visualization_msgs interactive_markers quadrotor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
  ${CUDA_INCLUDE_DIRS}
)

# 若 TensorRTRunner.cpp 里有 __global__ kernel，请打开下面开关
option(TRTCORE_COMPILE_AS_CUDA "Build TensorRTRunner.cpp as CUDA" OFF)

add_library(trtcore STATIC src/TensorRTRunner.cpp)
if(TRTCORE_COMPILE_AS_CUDA)
  enable_language(CUDA) # 3.16 支持
  set_source_files_properties(src/TensorRTRunner.cpp PROPERTIES LANGUAGE CUDA)
endif()
target_link_libraries(trtcore PUBLIC
  ${NVINFER} ${NVINFER_PLUGIN} ${NVONNXPARSER} ${CUDA_LIBRARIES})

add_library(obsqueue STATIC src/ObservationQueue.cpp)
target_include_directories(obsqueue PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_executable(drone_control_node src/drone_control_node.cpp)
target_link_libraries(drone_control_node PRIVATE trtcore obsqueue ${catkin_LIBRARIES})
set_target_properties(drone_control_node PROPERTIES
  BUILD_RPATH "/usr/lib/aarch64-linux-gnu;$ENV{TENSORRT_DIR}/lib")

add_executable(benchmark src/inferance_benchmark.cpp)
target_link_libraries(benchmark PRIVATE trtcore ${catkin_LIBRARIES})
set_target_properties(benchmark PROPERTIES
  BUILD_RPATH "/usr/lib/aarch64-linux-gnu;$ENV{TENSORRT_DIR}/lib")

install(TARGETS trtcore obsqueue drone_control_node benchmark
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
