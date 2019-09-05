cmake_minimum_required(VERSION 2.8.3)
project(cssd_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  gazebo_ros
)
catkin_package()
catkin_install_python(
  PROGRAMS
    scripts/gazebo_unpauser.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
include
 ${catkin_INCLUDE_DIRS}
)


