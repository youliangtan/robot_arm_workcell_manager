cmake_minimum_required(VERSION 2.8.3)
project(cssd_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  gazebo_ros
)
catkin_package()

find_package(gazebo REQUIRED)

add_executable(warm_start src/warm_start.cpp)
target_link_libraries(warm_start PRIVATE ${GAZEBO_LIBRARIES})
target_include_directories(warm_start PRIVATE ${GAZEBO_INCLUDE_DIRS})

install(
  TARGETS warm_start
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
