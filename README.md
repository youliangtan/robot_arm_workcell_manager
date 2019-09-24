
# robot_arm_workcell_manager (RAWM)
Robot arm manipulation manager package is one of the module for the Central sterile services department (cssd) workcell application. This package will act as a standalone workcell (aka: Dispenser Robot), which handles the robotics aspect of cssd_workcell. When a `DispenserRequest` is being sent out by a user to RAWM, a 'RAWM' workcell will begin execute the pick and place task. `ur10` and `ur10e` are used in this application. Current package is developed and tested on `ros-melodic` and `gazebo 9.1`. 

The task sequence starts with the action of picking up a custom design instrument tray from a medical rack, then eventually place the target tray on the transporter cart (follow-up delivery task by a AGV). Fiducial visual markers (aruco) will function as locating markers for pose estimation and id matching. Aruco markers are attached to the trays and AGV cart.

Now with namespace support!, enable two arms to perform choreographed dance!! 🤖🤖

**Active in Development!!!**

![alt text](/documentations/two_arms_dance.gif?)

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

## Run RAWM with Gazebo
Motion are planned dynamically and markers are being detected by the eef cameras on gazebo.

### Run Single Arm

```
# Terminal A: Run Gazebo Env
roslaunch cssd_gazebo one_arm.launch

# Terminal B: Run MoveIt Env With Rviz
roslaunch robot_arm_workcell_manager demo.launch sim:=true enable_fake_joints_execution:=false

# Terminal C: Run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch
```

### Run 2 Arms 

In this case, there are 2 `RAWM` workcells running... ✌️
```
# Terminal A: Run Gazebo Env
roslaunch cssd_gazebo main.launch

# Terminal B: Run MoveIt Env With Rviz
roslaunch robot_arm_workcell_manager two_arms_rviz.launch

# Terminal C: Run RAWM
roslaunch robot_arm_workcell_manager two_arms_rawm.launch
```

---

## Request a Task 

Open another terminal, then use rostopic to publish a `DispenserRequest.msg` to start the pick and place motion. Each request will execute one pick and place task. Intotal, 4 requests will be sent out to fill up the Transporter!!

*Request Task to UR10 arm!* 🤖
```
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx01, dispenser_name: ur10_001, items:[{item_type: marker_1, quantity: 1, compartment_name: 'marker_101'}] }' --once
## second request
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx02, dispenser_name: ur10_001, items:[{item_type: marker_2, quantity: 1, compartment_name: 'marker_100'}] }' --once
```

*Request Task to UR10e arm!* 🤖
```
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx03, dispenser_name: ur10e_001, items:[{item_type: marker_0, quantity: 1, compartment_name: 'marker_102'}] }' --once
## second request
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx04, dispenser_name: ur10e_001, items:[{item_type: marker_4, quantity: 1, compartment_name: 'marker_103'}] }' --once
```

By now, the robot dispenser will execute the task according to the `DispenserRequest`. GoodLuck!!

_p/s: You can play with the gazebo model by manually move the position of the transporter cart_

---

---

## Testing on submodules and lib 

### 1. Run Robot Arm Controller Test Code  (ToBeTested)
Run motion executor test code.
```
# Terminal A: Run Rviz and Robot Desciption... blablabla
roslaunch robot_arm_workcell_manager demo.launch

# Terminal B: Run arm_controller Node 
roslaunch robot_arm_workcell_manager arm_controller.launch
```

### 2. Run Fiducial Markers Handler Test Code (ToBeTested)
Test code to try out aruco marker detection. Camera and aruco markers are used for this application.

```
# Check Camera and configure path
vlc v4l2:///dev/video{$NUM}
```

**Calibration**: Refer to OpenCV Camera Calibration code, [here](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html#results). Once done, then copy the camera & distortion matrix from a .xml file to `/robot_arm_workcell_manager/config/usb_cam.yaml`.

```
roslaunch robot_arm_workcell_manager fiducial_markers_handler.launch
```

### 3. Overall Test with Robot Arm Workcell Manager (RAWM)
Single arm test just with Rviz and moveit
```
## Terminal A: Run Rviz and Moveit
roslaunch robot_arm_workcell_manager demo.launch sim:=false enable_fake_joints_execution:=true

## Terminal B: Run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch 

# Terminal C: Send the same `DispenserRequest.msg` as above
```

---

## Testing Arms on HARDWARE! (ToBeTested)

### Prerequisites
PLEASE KEEP YOUR HANDS ON THE BIG RED BUTTON!
Also, Download [ur_modern_driver with e series](https://github.com/AdmiralWall/ur_modern_driver/tree/kinetic_ur_5_4). Then install it.

### 1. Terminal A (robot bringup):
```
# For ur10
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=198.168.88.XX
# For ur10e,
roslaunch ur_modern_driver ur10e_bringup.launch robot_ip:=198.168.88.XX
```

### 2. Terminal B (moveit & Rviz):

*a) With Full RAWM package*
```
roslaunch robot_arm_workcell_manager demo.launch sim:=false enable_fake_joints_execution:=false
# then on terminal C, run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch 
```

*b) Or, just test it with moveit on Rviz (terminal B)*

![alt text](/documentations/rviz.gif?)

```
# For ur10,
roslaunch cssdbot_ur10_moveit_config hardware_minimal.launch
# or For ur10e,
roslaunch cssdbot_ur10e_moveit_config hardware_minimal.launch
```

With pure tryout using moveit on rviz, remember:
- !!!PLAN BEFORE EXECUTING. SCALE YORU VELOCITY!!!!
- Remember `CURRENT STATE` should always be :`<current_state>`, `GROUP` should be: `MANIPULATOR`
- Joint states can be altered in `cssdbot_urxx_moveit_config/config/urxx.srdf`


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
- master branch for `ur_modern_driver` currently doesnt support UR-E series

## TODO
- Further Code clean up!!
- gazebo model clean upssss
- collision model creation on the scene in moveit
- Warm start Issue!!!! Also ability to launch gazebo with rviz (Controller issue!!!)
- tune the PID for the arms / and physics params for physical models, seems to jitter slightly.
- use standard `robotDispenserBaseApadpter` as lib to make it more modular and compatable to rmf workcell framework
- robot arm left right scanning feature when searching for the mir transporter cart
- intergration with greater RMF environment
- Eventually, *FULL* Hardware Test!!!!!!
- test usb cam on new hardware, and config all usb video path
- Potential prob: checkout `prob1.png`, encounter yaw prob on ur10e (instead of ur10). Quick fix on cart's aruco marker's yaw angle