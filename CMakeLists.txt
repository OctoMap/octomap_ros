cmake_minimum_required(VERSION 2.8)
project(octomap_ros)
find_package(catkin)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS sensor_msgs tf pcl_ros)
else()
  catkin_stack()
  catkin_project(octomap_ros)
endif()

find_package(catkin REQUIRED sensor_msgs tf pcl_ros)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

find_package(octomap REQUIRED)
include_directories(${octomap_INCLUDE_DIRS})
link_directories(${octomap_LIBRARY_DIRS})
link_libraries(${PROJECT_NAME} ${octomap_LIBRARIES})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/conversions.cpp)

if (ROS_FUERTE_FOUND)
# install the applications
install(TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION share/${PROJECT_NAME}/lib)

endif()
if (ROS_GROOVY_OR_ABOVE_FOUND)
install(TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

