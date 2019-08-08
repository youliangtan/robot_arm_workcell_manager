# robot_arm_workcell_manager (RAWM)
Robot arm manipulation with moveit and fiducial markers. This Robot Arm manipulation will act as a individual standalone workcell, which user can interact with it by standard `rmf_msgs`. Robot arm will execute simple pick and place, which fiducial markers will be used to locate the target object's pose. Package is developed and tested with ros-melodic. 

**PACKAGE IS STILL IN DEVELOPMENT!!!**

![alt text](/documentations/rviz_bot.png?)

**Testing on Gazebo** (in development)
![alt text](/documentations/gazebo.png?)

---

## Getting Started

### Installation
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

- Work together with: CSSD_workcell_manager (ROS2)

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

**Calibration**: Refer to OpenCV Camera Calibration code, [here](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html#results). This tool has provided a chessboard.png for calibration. Once done, then copy the camera & distortion matrix from a .xml file to `/robot_arm_workcell_manager/config/usb_cam.yaml`.

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

Use Gazebo environment to test the pick and place of the workcell.

```
## Terminal A: Run Gazebo, alongside with Rviz and Moveit
roslaunch cssd_gazebo ur10_gazebo.launch

## Terminal B: Run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch
```

Then send the same `DispenserRequest.msg` as above. 


**IN DEVELOPMENT**

---

## Multiple Robot Arm Testing
To enable multiple `RAWM` (robot arm workcells) to work together on the same mahchine, namespace is used in the respective launchfile. To change edit the namespace, go to `demo.launch` and `robot_arm_workcell_manager.launch` to change the desired namespace.

```
  <!-- Namespace Toggling -->
  <arg name="arm_namespace"                 default="ur10"/>  
  <arg name="enable_namespace"              default="true"/> 
```

Then, run the code as usual....

3 topics: `/cssd_workcell/dispenser_request`, `/cssd_workcell/dispenser_state`, `/cssd_workcell/dispenser_result` will work as the same.

Note: This function is currently in development. TODO: create namespace for tf_lookup in `fiducial_markers_handler.cpp` code.

---


## Notes
- A custom designed "fork-lift" end effector is used in this process
- 3 executables are used in this application, namely: `robot_arm_workcell_manager` (MAIN), `robot_arm_controller`, `fiducial_markers_handler`.
- "Named Motion Target" can be used to name then request each "joint/pose goal" of the robot arm. Edit `motion_config.yaml` accordingly.
- Camera calib is tuned, and written here: `/config/usb_cam.yaml`
- Dispenser req will be received by RAWM, id: with convention of `marker_{$fiducial_id}`
- Use Ros_bridge to link ros1 msg to ros2
- Models must be moved to .gazebo folder in home directory to insert new model


## TODO
- Namespace for rosparam (senario of runnin multiple robot arms)
- Fix camera frame between `camera_optical_frame` and `camera`  (and update new tf tree), Upsidedown case!!
- Gazebo simulation
