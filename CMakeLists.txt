cmake_minimum_required(VERSION 2.8)
project(octomap_ros)

find_package(catkin REQUIRED COMPONENTS sensor_msgs tf pcl_ros octomap_msgs)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS sensor_msgs tf pcl_ros octomap_msgs)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

include_directories(${octomap_INCLUDE_DIRS})
link_directories(${octomap_LIBRARY_DIRS})
link_libraries(${PROJECT_NAME} ${octomap_LIBRARIES})

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/conversions.cpp)

install(TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/octomap_ros/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/octomap_ros/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
