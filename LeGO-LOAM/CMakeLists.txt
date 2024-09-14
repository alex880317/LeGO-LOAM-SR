cmake_minimum_required(VERSION 3.5)
project(lego_loam_sr)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

include_directories(${colcon_INCLUDE_DIRS})
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
include_directories(${PARENT_DIR}/plane_fitting/src)
include_directories(${PARENT_DIR}/customizeFactor/src)




# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cloud_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS thread serialization timer)

find_package(OpenCV REQUIRED)
message(STATUS "version: " ${OpenCV_VERSION})

# OpenMP
find_package(OpenMP)
IF(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message(STATUS "Found OpenMP")
ENDIF()

include_directories(include)
ament_export_include_directories(include)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  cloud_msgs
  rosbag2_cpp
)

add_executable(${PROJECT_NAME} src/imageProjection.cpp src/featureAssociation.cpp src/mapOptmization.cpp src/transformFusion.cpp src/main.cpp)
add_executable(AGPC test/AGPC_Detection.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${colcon_INCLUDE_DIRS} ${OpenCV_LIBS} gtsam)
target_link_libraries(AGPC ${PCL_LIBRARIES} ${colcon_INCLUDE_DIRS} ${OpenCV_LIBS} gtsam )
ament_target_dependencies(${PROJECT_NAME} ${dependencies} Eigen3)
ament_target_dependencies(AGPC ${dependencies} Eigen3)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
