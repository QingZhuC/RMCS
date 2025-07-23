cmake_minimum_required(VERSION 3.8)
project(auto_aim_test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--copy-dt-needed-entries")

# 查找ROS2基础依赖
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies (REQUIRED)

add_executable (
    ${PROJECT_NAME}
    src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  alliance_ros_auto_aim
  rclcpp
  std_msgs 
  hikcamera
)

### OpenCV ===
# set(OpenCV_DIR /path/to/opencv/build) # 如果需要这个可以方便你设置
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS} )
### OpenCV End ===

### Eigen ===
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
### Eigen End ===


ament_auto_package()
