cmake_minimum_required(VERSION 3.0.2)
project(my_drive)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 파이썬 노드 설치
catkin_install_python(PROGRAMS
  scripts/waypoint_drive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

