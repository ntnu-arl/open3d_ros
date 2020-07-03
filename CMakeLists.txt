cmake_minimum_required(VERSION 3.1)
project(open3d_ros)
set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Open3D REQUIRED)
include_directories(${OPEN3D_INCLUDE_DIR})
list(APPEND Open3D_LIBRARIES dl)
message(STATUS "Found Open3D ${Open3D_VERSION}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")
include_directories(${Open3D_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES open3d_ros
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(o3d_test src/o3d_test.cpp)
target_link_libraries(o3d_test ${catkin_LIBRARIES})
target_link_libraries(o3d_test ${Open3D_LIBRARIES})
target_include_directories(o3d_test PUBLIC ${Open3D_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/open3d_ros.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)