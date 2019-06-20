cmake_minimum_required(VERSION 2.8)
project(octomap_ros)

find_package(catkin REQUIRED COMPONENTS octomap_msgs sensor_msgs tf)
find_package(OCTOMAP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS octomap_msgs sensor_msgs tf
  DEPENDS OCTOMAP)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/conversions.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}
  PUBLIC include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
