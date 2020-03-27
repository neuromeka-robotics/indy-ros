cmake_minimum_required(VERSION 2.8.3)
project(indy_driver_py)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  rviz
  tf
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

