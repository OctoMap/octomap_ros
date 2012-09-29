cmake_minimum_required(VERSION 2.8)
project(octomap_ros)
find_package(catkin)
catkin_stack()

find_package(catkin REQUIRED sensor_msgs tf pcl_ros octomap_msgs)
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

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  DEPENDS sensor_msgs tf pcl_ros octomap_msgs  
)

add_library(${PROJECT_NAME} src/conversions.cpp)

install(TARGETS ${PROJECT_NAME}  
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

