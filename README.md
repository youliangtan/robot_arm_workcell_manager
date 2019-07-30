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

### Dependencies
- Fiducial Marker Detector: [here](https://github.com/UbiquityRobotics/fiducials)
```
sudo apt-get install ros-melodic-aruco-detect
sudo apt-get install ros-melodic-fiducial-msgs
```
- rmf_msgs: [here](null)


### Make and Build
```
catkin_make --pkg robot_arm_workcell_manager -j4
```

## Run the Code

### Robot Arm Controller Testing
```
# Absolute Path
rosrun robot_arm_workcell_manager robot_arm_controller _motion_target_yaml_path:="/home/youliang/catkin_ws/src/robot_arm_workcell_manager/config/motion_target.yaml" _group_name:="manipulator"
## OR
roslaunch robot_arm_workcell_manager arm_controller.launch

roslaunch robot_arm_workcell_manager demo.launch
```

### Fiducial Markers Testing
```
roslaunch robot_arm_workcell_manager markers_detector.launch
rosrun robot_arm_workcell_manager fiducial_markers_handler _camera_frame_id:="camera" _marker_tf_path:="/home/youliang/catkin_ws/src/robot_arm_workcell_manager/config/markers_tf.yaml"
```

### Overall Test with RAWM
```
rosrun robot_arm_workcell_manager robot_arm_workcell_manager
```

## Notes
- 3 executables are used in this application, namely: `robot_arm_workcell_manager` (MAIN), `robot_arm_controller`, `fiducial_markers_handler`.
- "Named Motion Target" can be used to name then request each "joint/pose goal" of the robot arm. Edit `motion_config.yaml` accordingly.
- Camera calib is tuned, and written here: `/config/usb_cam.yaml`
- Dispenser req will be received by RAWM, id: with convention of `marker_{$fiducial_id}`