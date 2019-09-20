# robot_arm_workcell_manager (RAWM)
Robot arm manipulation manager package is one of the module for the Central sterile services department (cssd) workcell application. This package will act as a standalone workcell (aka: Dispenser Robot), which handles the robotics aspect of cssd_workcell. When a `DispenserRequest` is being sent out by a user to RAWM, this workcell will begin executing the task, starting with the picking motion of custom design instrument tray from a medical rack, then end it by placing the tray on MIR cart (follow-up delivery task by MIR). Fiducial Aruco markers will function as locating visual markers for pose estimation. Aruco markers are attached to the trays and MIR cart. Current package is developed and tested on `ros-melodic` and `gazebo 9.1`. 

## TO-DO DEVELOPMENT ON THIS BRANCH

1. Two arm launch in gazebo [done, for now]

   [main changes done]
   - added ur10e urdf.xacro model with end effector to cssdbot_description/urdf/ur10e (altered it minorly to support     velocityController) 
   - new saved gazebo world "cssd_two_arms.world" with one more arm and one more rack
   - namespaced two arms in gazebo -> "/arm1" and "/arm2"
   - main.launch gives you an option to choose ur10 or ur10e as the second arm, refer to arg="mode"
   - added a few urdf files from universal_robot/ur_description & ur_e_description to cssdbot_description to make it a more standalone pkg.
   - added ur10 and ur10e meshes into package
   
   [some more improvements to be done]
   - tune the PID for the arms / and physics params for physical models, seems to jitter slightly.

2. Two arm launch rviz/RAWM namespaced [ongoing]

3. clean up gazebo launch [partially done]

4. moveit_config folders (should be standalone) [partially done]
   - generated own standalone moveit config pkgs for ur10 and ur10e using setup assistant with cssdbot_description's urdf
   - included limited joints option since universal_robot' ori pkg supports.
   - included srdf, different default poses!
   - included another launch file for launches for hardware - hardware_minimal.launch
   
   [to-do]
   - check with coffeebot's to check for anymore insights
   - check own concern - that universal_robot's moveit_config was generated using an older version of setup assistant since 
     some params give out warnings on the new moveit - signalling depreciation of some params.

5. clean up cssdbot urdf [onging]

6. add ur10e with end effector [done]

**STILL IN DEVELOPMENT!!!**

![alt text](/documentations/gazebo_test.png?)

*Full Video Link*, [here](https://drive.google.com/open?id=1dGKh3FVMlUwX8GUMv3mgxQFBm0OnGa8B)

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

## Run RAWM with Gazebo

```
# Terminal A: Run Gazebo, alongside with Rviz and Moveit
roslaunch cssd_gazebo ur10_gazebo.launch

# Terminal B: Run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch
```

### Request a Task 

Open another terminal, then use rostopic to publish a `DispenserRequest.msg` to start the pick and place motion.
```
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx01, dispenser_name: ur10_001, items:[{item_type: marker_2, quantity: 1, compartment_name: 'marker_101'}] }' --once

# second request
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx02, dispenser_name: ur10_001, items:[{item_type: marker_1, quantity: 1, compartment_name: 'marker_100'}] }' --once

# third request: manually remove tray, and play with the pose of cart on gazebo
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx00, dispenser_name: ur10_001, items:[{item_type: marker_0, quantity: 1, compartment_name: 'marker_100'}] }' --once
```

By now, the robot dispenser will execute the task according to the `DispenserRequest`. GoodLuck!!

---


## Testing on submodules and lib 

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

# Terminal C: Send the same `DispenserRequest.msg` as above
```

---


## Notes
- A custom designed "fork-lift" end effector is used in this process
- 3 executables are used in this application, namely: `robot_arm_workcell_manager` (MAIN), `robot_arm_controller`, `fiducial_markers_handler`.
- "Named Motion Target" can be used to name then request each "joint/pose goal" of the robot arm. Edit `motion_config.yaml` accordingly.
- Camera calib is tuned, and written here: `/config/usb_cam.yaml`
- Dispenser req will be received by RAWM, id: with convention of `marker_{$fiducial_id}`
- Use Ros_bridge/SOSS to link ros1 msg to ros2, eventually communicates with a ``cssd_workcell_manager`


## TODO
- Code clean up!!
- gazebo model clean upssss
- robust multiple level scanning of rack
- collision model creation on the scene in moveit
- Namespace for rosparam (senario of runnin multiple robot arms)
- intergration with greater RMF environment
- Eventually, Hardware Test!!!!!!
