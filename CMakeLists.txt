cmake_minimum_required(VERSION 3.1)
project(open3d_ros)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

# System Dependancies
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES open3d_ros
 CATKIN_DEPENDS roscpp sensor_msgs
 DEPENDS Open3D
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

# C++ library
add_library(${PROJECT_NAME} src/open3d_ros.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Examples
add_executable(subscriber_example examples/ex_pc2_subscribe.cpp)
target_link_libraries(subscriber_example ${catkin_LIBRARIES} ${Open3D_LIBRARIES} ${PROJECT_NAME})

add_executable(publisher_example examples/ex_pcd_publish.cpp)
target_link_libraries(publisher_example ${catkin_LIBRARIES} ${Open3D_LIBRARIES} ${PROJECT_NAME})

add_executable(conversion_time examples/conv_time.cpp)
target_link_libraries(conversion_time ${catkin_LIBRARIES} ${Open3D_LIBRARIES} ${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS subscriber_example publisher_example conversion_time
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY data/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(open3d_ros_tests test/open3d_ros_tests.cpp)
  target_link_libraries(open3d_ros_tests ${PROJECT_NAME} ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
endif()

