cmake_minimum_required(VERSION 2.8.3)
project(robot_arm_workcell_manager)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  
  fiducial_msgs
  aruco_detect
  rmf_msgs
)

find_package(YAML-CPP REQUIRED yaml-cpp)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(gazebo 9 REQUIRED)



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
  DEPENDS
    EIGEN3
    gazebo
)
# catkin_install_python(
#   PROGRAMS
#     scripts/gazebo_unpauser.py
#     scripts/RequestCoffeebot.py
#     scripts/gazebo_model_spawner.py
#     scripts/LaunchBeverage.py
#   DESTINATION
#     ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#------------------------------- Gazebo stuffs -----------------------------------------
# set(coffeebot_plugin_source
#   src/CoffeebotPlugin.cpp
#   src/GripperHandler.cpp
# )
# add_library(CoffeebotPlugin SHARED
#   ${coffeebot_plugin_source}
# )
# target_link_libraries(CoffeebotPlugin PRIVATE
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
# )
# target_include_directories(CoffeebotPlugin PRIVATE
#   ${GAZEBO_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )
# install(TARGETS CoffeebotPlugin
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#---------------------------------- Exec Stuffs--------------------------------------------------


# add_library(object_pose_estimation  src/object_pose_estimation.cpp )
# target_link_libraries(object_pose_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${YAML_CPP_LIBRARIES} ${EIGEN3_LIBRARIES})
# add_dependencies(object_pose_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable( robot_arm_controller src/robot_arm_controller.cpp )
target_link_libraries(  robot_arm_controller  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ) #${GAZEBO_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS} )

add_executable( fiducial_markers_handler src/fiducial_markers_handler.cpp )
target_link_libraries( fiducial_markers_handler  ${catkin_LIBRARIES} ) #${GAZEBO_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS} )


# TODO create lib for above dependencies
add_executable( robot_arm_workcell_manager src/robot_arm_workcell_manager.cpp )
target_link_libraries( robot_arm_workcell_manager  ${catkin_LIBRARIES} ) #${GAZEBO_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS} )



# set(Executables
#   CoffeebotController
#   hacky_coffeebot_workspace
# )
# foreach(exec ${Executables})
#   add_executable(${exec} 
#     src/${exec}.cpp
#   )
#   target_link_libraries(${exec} PRIVATE
#     ${GAZEBO_LIBRARIES}
#     ${catkin_LIBRARIES}
#   )
#   target_include_directories(${exec} PRIVATE
#     ${GAZEBO_INCLUDE_DIRS}
#     ${catkin_INCLUDE_DIRS}
#   )
#   install(TARGETS ${exec}
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )
# endforeach()


# #---------------------------------------------------------------------------------------------------
# install(
#   DIRECTORY
#     config 
#     controller
#     launch
#     meshes
#     scripts
#     urdf
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
