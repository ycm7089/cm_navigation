cmake_minimum_required(VERSION 3.0.2)
project(cm_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  message_generation
  image_transport
  cv_bridge
  tf
  tf_conversions
  aruco
  visualization_msgs
  cm_aruco_msgs
  nav_msgs

)

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/aruco_mapping.h)

generate_messages(DEPENDENCIES
                  std_msgs
                  nav_msgs
                  geometry_msgs)
set( sources 
)

set( headers
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp std_msgs cm_aruco_msgs nav_msgs
  DEPENDS system_lib
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(my_tf_listener src/tf_listener.cpp)
target_link_libraries(my_tf_listener ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(WayPoint_Saver src/file_maker.cpp)
target_link_libraries(WayPoint_Saver ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pose_checker src/pose_checker.cpp)
target_link_libraries(pose_checker ${catkin_LIBRARIES})

