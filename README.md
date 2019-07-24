# robot_arm_workcell_manager (RAWM)
robot arm manipulation with moveit and fiducial markers.  This package is developed and tested with ros-melodic. 

**DEVELOPPING!!!**

## Getting Started

Installation
```
# ROS moveit stuffs
# Robot Arm Dependencies
# Gazebo Stuffs
```

Make and Build
```
catkin_make --pkg robot_arm_workcell_manager -j4
```

# Run the Code
```
# Absolute Path
rosrun robot_arm_workcell_manager robot_arm_controller _motion_target_yaml_path:="/home/youliang/catkin_ws/src/robot_arm_workcell_manager/config/motion_target.yaml"
# OR
roslaunch robot_arm_workcell_manager arm_controller.launch

rosrun robot_arm_workcell_manager fiducial_markers_handler 
```

# Notes
- 3 executables are used in this application, namely: `robot_arm_workcell_manager` (MAIN), `robot_arm_controller`, `fiducial_markers_handler`.
- "Named Motion Target" can be used to name then request each "joint/pose goal" of the robot arm. Edit `motion_config.yaml` accordingly.