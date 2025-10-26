cmake_minimum_required(VERSION 3.0.2)
project(henes_car_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  nav_msgs 
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES henes_car_control
   CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(henes_car_control_node src/henes_car_control_node2025.cpp)
add_dependencies(henes_car_control_node ${henes_car_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(henes_car_control_node ${catkin_LIBRARIES} )

add_executable(henes_car_control_joy_node src/henes_car_control_node_joy.cpp)
add_dependencies(henes_car_control_joy_node ${henes_car_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(henes_car_control_joy_node ${catkin_LIBRARIES} )

add_executable(henes_car_control_node_lidar src/henes_car_control_node_lidar.cpp)
add_dependencies(henes_car_control_node_lidar ${${PROJECT_NAMW}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(henes_car_control_node_lidar ${catkin_LIBRARIES} )
