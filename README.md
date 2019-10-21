
# robot_arm_workcell_manager (RAWM)
Robot arm manipulation manager package is one of the module for the Central Sterile Services Department (CSSD) workcell application. This package will act as a standalone workcell (aka: Dispenser Robot), which handles the robotics aspect of cssd_workcell. When a `DispenserRequest` is being sent out by a user to RAWM, a "RAWM" workcell will begin execute the pick and place task. `ur10` and `ur10e` are used in this application. Current package is developed and tested on `ros-melodic` and `gazebo 9.1`. 

The task sequence starts with the action of picking up a custom design instrument tray from a medical rack, then eventually place the target tray on the transporter cart (follow-up delivery task by a AGV). Fiducial visual markers (aruco) will function as locating markers for pose estimation and id matching. Aruco markers are attached to the trays and AGV cart.

Now with namespace support! Enabling two (or more!) arms to perform a choreographed dance!! 🤖🤖

**Active in Development!!!**

![alt text](/documentations/two_arms_dance.gif?)

![alt text](/documentations/robot_in_room2.png?)

*Full Video Link* (with one arm),  [here](https://drive.google.com/open?id=1dGKh3FVMlUwX8GUMv3mgxQFBm0OnGa8B)

*Full Video Link* (with two arms), [here](https://drive.google.com/open?id=1dT9zQ5bbWr0oMqf9hO2wWMH2uiiHR1AT)

---

## Getting Started

### Basic Installation

```
# ROS, Moveit stuffs
# Gazebo Stuffs
```

### Dependencies

- Universal Robot: [here](https://github.com/ros-industrial/universal_robot), **Remember to switch branch
- Fiducial Marker Detector: [here](https://github.com/UbiquityRobotics/fiducials)
```bash
sudo apt-get install ros-melodic-aruco-detect
sudo apt-get install ros-melodic-fiducial-msgs
```
- rmf_msgs: [here](https://github.com/RMFHOPE/rmf_msgs_ros1), **Phasing Out Soon
- CSSD_workcell_manager (ROS2): [Here](https://github.com/sharp-rmf/rmf-workcell/tree/cssd_workcell/cssd_workcell_manager) Not Necessary

### Make and Build
```bash
catkin_make --pkg cssdbot_moveit_config cssdbot_description cssd_gazebo
catkin_make --pkg robot_arm_workcell_manager -j4
```

---

## Run RAWM with Gazebo
Motion are planned dynamically and markers are being detected by the eef cameras on gazebo.

### Run Single Arm

```bash
# Terminal A: Run Gazebo Env
roslaunch cssd_gazebo one_arm.launch

# Terminal B: Run MoveIt Env With Rviz
roslaunch robot_arm_workcell_manager demo.launch sim:=true enable_fake_joints_execution:=false

# Terminal C: Run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch
```

_p/s: Wait each launch terminal to be fully launched before launching the next `.launch`._

### Run 2 Arms 

In this case, there are 2 `RAWM` workcells running... ✌️
```bash
# Terminal A: Run Gazebo Env
roslaunch cssd_gazebo two_arms.launch

# Terminal B: Run MoveIt Env With Rviz
roslaunch robot_arm_workcell_manager two_arms_rviz.launch

# Terminal C: Run RAWM
roslaunch robot_arm_workcell_manager two_arms_rawm.launch
```

_p/s: Wait each launch terminal to be fully launched before launching the next `.launch`._

---

## Request a Task 

Open another terminal, then use rostopic to publish a `DispenserRequest.msg` to start the pick and place motion. Each request will execute one pick and place task. Intotal, 4 requests will be sent out to fill up the Transporter!!

*Request Task to UR10 arm!* 🤖

```bash
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx01, dispenser_name: ur10_001, items:[{item_type: marker_1, quantity: 1, compartment_name: 'marker_101'}] }' --once
## second request
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx02, dispenser_name: ur10_001, items:[{item_type: marker_2, quantity: 1, compartment_name: 'marker_100'}] }' --once
```

*Request Task to UR10e arm!* 🤖

```bash
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx03, dispenser_name: ur10e_001, items:[{item_type: marker_0, quantity: 1, compartment_name: 'marker_102'}] }' --once
## second request (IK Flip issue TOBEFIX)
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx04, dispenser_name: ur10e_001, items:[{item_type: marker_4, quantity: 1, compartment_name: 'marker_103'}] }' --once
```

By now, the robot dispenser will execute the task according to the `DispenserRequest`. GoodLuck!!

_p/s: You can play with the gazebo model by manually move the position of the transporter cart_

---

---

## Testing on submodules and lib 

### 1. Run Robot Arm Controller Test Code  (ToBeTested)
Run motion executor test code.
```bash
# Terminal A: Run Rviz and Robot Desciption... blablabla
roslaunch robot_arm_workcell_manager demo.launch

# Terminal B: Run arm_controller Node 
roslaunch robot_arm_workcell_manager arm_controller.launch
```

### 2. Run Fiducial Markers Handler Test Code (ToBeTested)
Test code to try out aruco marker detection. Camera and aruco markers are used for this application.

```bash
# Check Camera and configure path
vlc v4l2:///dev/video{$NUM}
```

**Calibration**: Refer to OpenCV Camera Calibration code, [here](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html#results). Once done, then copy the camera & distortion matrix from a .xml file to `/robot_arm_workcell_manager/config/usb_cam.yaml`.

```bash
roslaunch robot_arm_workcell_manager fiducial_markers_handler.launch
```

### 3. Overall Test with Robot Arm Workcell Manager (RAWM)
Single arm test just with Rviz and moveit
```bash
## Terminal A: Run Rviz and Moveit
roslaunch robot_arm_workcell_manager demo.launch sim:=false enable_fake_joints_execution:=true

## Terminal B: Run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch 

# Terminal C: Send the same `DispenserRequest.msg` as above
```

---

## Setting environment

The config file is in rawm package called environment_object. This is to set the environment in moveit. Please subscribe to planning scene in rviz to see the objects.

- The object must be called object_1, object_2 and so on. 
- type 1 for box, 2 sphere, 3 clinder, 4 cone. 

## Octomapping 
Information is pulled from the depth camera and added to the planning scene. The params can be found at cssdbot_ur10_moveitconfig package, sensor_manager.launch.xml and sensors_kinect_pointcloud.yaml

---

## Testing Arms on HARDWARE!

Please Refer to the readme in README in [cssd_hardware](/cssd_hardware/).

![alt text](/documentations/test_on_hardware.gif?)

---

## Notes
- Custom designed "fork-lift" end effector, trays and tray placements are used in this application.
- 3 executables are used in this application, namely: `robot_arm_workcell_manager` (MAIN), `robot_arm_controller`, `fiducial_markers_handler`.
- `robot_arm_workcell_manager` executable depends on above `robot_arm_controller` and `fiducial_markers_handler` libs. 
- "Named Motion Target" can be used to name then request each "joint/pose goal" of the robot arm. Edit `motion_config.yaml` accordingly.
- construction of extended TFs from the detected marker is configured in `extended_tf.yaml`
- Camera should be calibrated, and intrinsic params should be written here: `/config/usb_cam.yaml`
- Dispenser req will be received by RAWM, id: with convention of `marker_{$fiducial_id}`
- Use Ros_bridge/SOSS to link ros1 msg to ros2, eventually communicates with a ``cssd_workcell_manager`
- Tune all relevant param for RAWM in `robot_arm_workcell_manager.launch`, including `robot_id` and `transporter_placement`
- To check out `tf_tree` and `rqt_graph`, go to `documentations` folder
- To add more arms: expand `cssd_gazebo two_arms.launch`, `two_arms_rviz.launch`, `two_arms_rawm.launch`
- master branch for `ur_modern_driver` currently doesn't support UR-E series

## Debuging process
- Jittering Problem during picking up of tray with Eef:  use `velocity_controllers` instead of `position_controller`
- MoveIt Namespace issue: In `robot_arm_controller.cpp` there's a declaration of namespace in for the movegroup by `ros::NodeHandle moveit_nh(arm_namespace_)`
- `GOAL_TOLERANCE_VIOLATED` error code in action server: As mentioned in [here](https://github.com/ros-planning/moveit/issues/1475#issuecomment-504364419), move group setTolerance is not working here. Will need to manually change it in `ur_velocity/position_controller.yaml`. This is the input param for `ros_control/joint_trajectory_controller` during spawn
- Joint IK flip issue while placing to ``marker_103` : Tried switch the planner from `ompl` to `stomp` (in `planning_context.launch`), still not able to fully solve the issue.

## TODO
- Further Code clean up!!
- gazebo model clean upssss
- collision model creation on the scene in moveit
- Warm start Issue!!!! Also ability to launch gazebo with rviz (Controller issue!!!)
- tune the PID for the arms / and physics params for physical models, seems to jitter slightly.
- robot arm left right scanning feature when searching for the mir transporter cart
- intergration with greater RMF environment
- Eventually, *FULL* Hardware Test!!!!!!
- test usb cam on new hardware, and config all usb video path
- Potential prob: checkout `prob1.png`, encounter yaw prob on ur10e (instead of ur10). Quick fix on cart's aruco marker's yaw angle
- Dynamic payload setting on Ur10e
- External camera support, create another new `cssd_hardware_launch` pkg
