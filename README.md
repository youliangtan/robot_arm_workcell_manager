# robot_arm_workcell_manager (RAWM)
robot arm manipulation with moveit and fiducial markers.  This package is developed and tested with ros-melodic. 

**IN DEVELOPMENT!!!**

![alt text](/documentations/rviz_bot.png?)


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

- robot_manipulator_manager: [here](https://github.com/tanyouliang95/robot_manipulator_control)
   "This will eventually be unnessasary"


### Make and Build
```
catkin_make --pkg robot_arm_workcell_manager -j4
```

## Run the Code

### Robot Arm Controller Testing

To individually test the robot arm control. To test this, rmb to uncomment line 99 `add_executable(XXX)` and comment line 100 `add_library(XXX)`, only a executable will be generated.

```
# Absolute Path
rosrun robot_arm_workcell_manager robot_arm_controller _motion_target_yaml_path:="/home/youliang/catkin_ws/src/robot_arm_workcell_manager/config/motion_target.yaml" _group_name:="manipulator"
## OR
roslaunch robot_arm_workcell_manager arm_controller.launch

roslaunch robot_arm_workcell_manager demo.launch
```

### Fiducial Markers Testing

To individually test fiducial marker detection. If testing this, rmb to uncomment line 1.4 `add_executable(XXX)` and comment line 105 `add_library(XXX)`, only a executable will be generated.

```
# Check Camera and configure path
vlc v4l2:///dev/video{$NUM}

# Run Test Code
roslaunch robot_arm_workcell_manager markers_detector.launch
rosrun robot_arm_workcell_manager fiducial_markers_handler _camera_frame_id:="camera" _marker_tf_path:="/home/youliang/catkin_ws/src/robot_arm_workcell_manager/config/markers_tf.yaml"
```

**Calibration**
Follow OpenCV Checkers Bot Camera Calibration, then copy the camera & distortion matrix to `usb_cam.yaml`


### Overall Test with RAWM
```
## Run Rviz and Moveit
roslaunch robot_arm_workcell_manager demo.launch

## Load Param
roscd robot_arm_workcell_manager
cd config
rosparam load rawm_param.yaml

## Run Exec
rosrun robot_arm_workcell_manager robot_arm_workcell_manager

# Pub Dispenser Request
rostopic pub /dispenser_request rmf_msgs/DispenserRequest '{request_id: test_reqeust, dispenser_name: ur10_001, items:[{item_type: marker_0, quantity: 1}] }' --once
```

## Notes
- 3 executables are used in this application, namely: `robot_arm_workcell_manager` (MAIN), `robot_arm_controller`, `fiducial_markers_handler`.
- "Named Motion Target" can be used to name then request each "joint/pose goal" of the robot arm. Edit `motion_config.yaml` accordingly.
- Camera calib is tuned, and written here: `/config/usb_cam.yaml`
- Dispenser req will be received by RAWM, id: with convention of `marker_{$fiducial_id}`

## TODO
- Namespace for rosparam (senario of runnin multiple robot arms)
- Fix camera frame on `wrist_3_link` and sync with `camera_optical_frame`  (new tf tree)