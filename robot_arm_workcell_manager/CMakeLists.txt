cmake_minimum_required(VERSION 2.8.3)
project(robot_arm_workcell_manager)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  gazebo_ros
  geometry_msgs
  ur_description
  ur_gazebo
  moveit_ros_move_group
  fiducial_msgs
  aruco_detect
  rmf_dispenser_msgs
  hanwha_controller
)

find_package(YAML-CPP REQUIRED yaml-cpp)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(gazebo 9 REQUIRED)
find_package(hanwha_controller REQUIRED)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    gazebo_ros
    geometry_msgs
    rmf_dispenser_msgs
  DEPENDS
    EIGEN3
    gazebo
)

#---------------------------------- Exec Stuffs--------------------------------------------------

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library( robot_arm_controller src/robot_arm_controller.cpp )
target_link_libraries(  robot_arm_controller  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} )
add_dependencies(robot_arm_controller ${catkin_EXPORTED_TARGETS})

add_executable( robot_arm_controller_node src/robot_arm_controller.cpp )
target_link_libraries(  robot_arm_controller_node  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} )

add_library( fiducial_markers_handler src/fiducial_markers_handler.cpp )
target_link_libraries( fiducial_markers_handler  ${catkin_LIBRARIES}  ${YAML_CPP_LIBRARIES}  )
add_dependencies(fiducial_markers_handler ${catkin_EXPORTED_TARGETS})

add_executable( fiducial_markers_handler_node src/fiducial_markers_handler.cpp )
target_link_libraries( fiducial_markers_handler_node  ${catkin_LIBRARIES}  ${YAML_CPP_LIBRARIES}  )

add_library( dispenser_workcell_adapter src/dispenser_workcell_adapter.cpp )
target_link_libraries(  dispenser_workcell_adapter  ${catkin_LIBRARIES} )
add_dependencies(dispenser_workcell_adapter ${catkin_EXPORTED_TARGETS})

add_executable( dispenser_workcell_test src/dispenser_workcell_adapter.cpp )
target_link_libraries(  dispenser_workcell_test  ${catkin_LIBRARIES} )

## -------------------------------- Main Exec for RAWM Package -----------------------------------

add_executable( robot_arm_workcell_manager src/robot_arm_workcell_manager.cpp )
target_link_libraries(  robot_arm_workcell_manager
  ${catkin_LIBRARIES}
  robot_arm_controller
  fiducial_markers_handler
  dispenser_workcell_adapter
)

## ------------------------------- Hanwha Arm -----------------------------

add_executable( hanwha_arm_workcell_manager src/hanwha_arm_workcell_manager.cpp )
target_link_libraries(  hanwha_arm_workcell_manager
  ${catkin_LIBRARIES}
  hanwha_arm_controller
  fiducial_markers_handler
  dispenser_workcell_adapter
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
