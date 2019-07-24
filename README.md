# robot_arm_workcell_manager (RAWM)
robot arm manipulation with moveit and fiducial markers.  This package is tested with ros-melodic. 

**DEVELOPPING!!!**

## Getting Started

Installation
```
# ros moveit stuffs
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

roslaunch robot_arm_workcell_manager arm_controller.launch

rosrun robot_arm_workcell_manager fiducial_markers_handler 
```