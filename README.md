# robot_arm_workcell_manager (RAWM)
Robot arm manipulation with moveit and fiducial markers. This Robot Arm manipulation will act as a individual standalone workcell, which user can interact with it by standard `rmf_msgs`. Robot arm will execute simple pick and place, which fiducial markers will be used to locate the target object's pose. Package is developed and tested with ros-melodic. 

**PACKAGE IS STILL IN DEVELOPMENT!!!**

![alt text](/documentations/rviz_bot.png?)

**Testing on Gazebo** (in development)
![alt text](/documentations/gazebo.png?)

---

## Getting Started

### Basic Installation

```
# ROS, Moveit stuffs
# Gazebo Stuffs
```

### Dependencies

- Universal Robot: [here](https://github.com/ros-industrial/universal_robot), **Remember to switch branch
- HanWha: TBC
- Fiducial Marker Detector: [here](https://github.com/UbiquityRobotics/fiducials)
```
sudo apt-get install ros-melodic-aruco-detect
sudo apt-get install ros-melodic-fiducial-msgs
```
- rmf_msgs: [here](https://github.com/RMFHOPE/rmf_msgs_ros1), **Phasing Out Soon
- CSSD_workcell_manager (ROS2): Work in Progress

### Make and Build
```
catkin_make --pkg cssdbot_moveit_config cssdbot_description cssd_gazebo
catkin_make --pkg robot_arm_workcell_manager -j4
```

---


## Run the Code

### 1. Run Robot Arm Controller Test Code
Run motion executor test code.
```
# Terminal A: Run Rviz and Robot Desciption... blablabla
roslaunch robot_arm_workcell_manager demo.launch

# Terminal B: Run arm_controller Node 
roslaunch robot_arm_workcell_manager arm_controller.launch
```

### 2. Run Fiducial Markers Handler Test Code
Test code to try out aruco marker detection. Camera and aruco markers are used for this application.


```
# Check Camera and configure path
vlc v4l2:///dev/video{$NUM}
```

**Calibration**: Refer to OpenCV Camera Calibration code, [here](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html#results). Once done, then copy the camera & distortion matrix from a .xml file to `/robot_arm_workcell_manager/config/usb_cam.yaml`.

Run Test Code
```
roslaunch robot_arm_workcell_manager fiducial_markers_handler.launch
```


### 3. Overall Test with Robot Arm Workcell Manager (RAWM)

This RAWM exec depends on above `robot_arm_controller` and `fiducial_markers_handler` libs. 

```
## Terminal A: Run Rviz and Moveit
roslaunch robot_arm_workcell_manager demo.launch

## Terminal B: Run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch
```

### Request a Task 

Open another terminal, then use rostopic to publish a `DispenserRequest.msg` to start the pick and place motion.
```
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: test_reqeust, dispenser_name: ur10_001, items:[{item_type: marker_0, quantity: 1, compartment_name: 'marker_100'}] }' --once
```

While the task is ongoing, user can check state of the arm_workcell by rostopic echo `/dispenser_state` and `dispenser_result` topic. 

---

## Gazebo Environment Testing


### Add models to Gazebo
Symlink models from package to `~/.gazebo/`.
```
ln -s ~/catkin_ws/src/robot_arm_workcell_manager/cssd_gazebo/models ~/.gazebo/models
```

### Running CSSD gazebo environment
```
# Terminal A: Run Gazebo, alongside with Rviz and Moveit
roslaunch cssd_gazebo ur10_gazebo.launch

# Terminal B: Run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch

# Terminal C: Send the same `DispenserRequest.msg` as above
```

***You will notice that the gazebo and RAWM terminal is unable to finish launch file. This is due to gazebo hacking method to initialise joint in the home position. Wait for a few seconds then press play at the bottom of gazebo. May not work***


### Editing the Position of Trolley (TBC)
Dont save the map above, to permanently save the world, edit it according to the step below.
```
roslaunch cssd_gazebo gazebo_editing_world.launch
# Edit then save and close the world, then spawn the launch file below 
```

***[this](http://sdformat.org/spec) link to SDF format***

---

## Issues on Gazebo

1. Models phasing through eef due to mesh of tray being too thin. Solved by adding another collision box in the tray result is seen in picture below. If items are to go into the box, bitmask is needed
![alt text](/documentations/picking_up_tray.png)

2. Models jitters on ground and arm. 
- Tuned [kp](http://sdformat.org/spec?ver=1.6&elem=collision#ode_kp) and [kd](http://sdformat.org/spec?ver=1.6&elem=collision#ode_kd) but more fine tunning needed
- Changed arm urdf. Not sure if it's right but follow [pr2 arm urdf](https://github.com/PR2/pr2_common/blob/melodic-devel/pr2_description/urdf/gripper_v0/gripper.gazebo.xacro) but different from what is written in [tutorial](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros).

3. Lack of friction. Not solved.

![](documentations/slip.gif)


---


## Notes
- A custom designed "fork-lift" end effector is used in this process
- 3 executables are used in this application, namely: `robot_arm_workcell_manager` (MAIN), `robot_arm_controller`, `fiducial_markers_handler`.
- "Named Motion Target" can be used to name then request each "joint/pose goal" of the robot arm. Edit `motion_config.yaml` accordingly.
- Camera calib is tuned, and written here: `/config/usb_cam.yaml`
- Dispenser req will be received by RAWM, id: with convention of `marker_{$fiducial_id}`
- Use Ros_bridge/SOSS to link ros1 msg to ros2
- Models must be moved to .gazebo folder in home directory to insert new model


## TODO
- Namespace for rosparam (senario of runnin multiple robot arms)
- Gazebo simulation
  - initial joint position in joint
  - friction of eef
- Stop remaining motion when one of the motion in the middle fails. (e.g stop unloading if one of the loading motion fails)
- Set start point before executing every motion.
Error statement: Invalid Trajectory: start point deviates from current robot state more than 0.01
joint 'shoulder_pan_joint': expected: 0.215141, current: 0.204255

