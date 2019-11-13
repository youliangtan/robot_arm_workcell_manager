
# robot_arm_workcell_manager (RAWM)
Testing Arms on HARDWARE! (Testing on Going). Here, the test is mainly on UR10e.  PLEASE KEEP YOUR HANDS ON THE BIG RED BUTTON!

## Prerequisites

 - Intel Realsense, install from [here](https://github.com/IntelRealSense/realsense-ros)
 - UR_modern_driver: [ur_modern_driver with e series](https://github.com/AdmiralWall/ur_modern_driver/tree/kinetic_ur_5_4), `kinectic_ur_5_4` branch

```
catkin_make --pkg cssd_hardware -j4
```

## Launching of single arm
The Current UR10e robor IP is: `172.16.17.2`. Remember to switch the UR10e pendant to "remote" in order to control the arm via ROS. 

Then launch 'single_arm launch'. This bringup will spawn all required nodes: driver, RAWM, MoveIt, RVIZ, realsense cam:
```
roslaunch cssd_hardware single_arm.launch
```

After bringup, send a `dispenserRequest` to move the arm
```
rostopic pub /cssd_workcell/dispenser_request rmf_msgs/DispenserRequest '{request_id: 0xx03, dispenser_name: ur10e_001, items:[{item_type: marker_1, quantity: 1, compartment_name: 'marker_100'}] }' --once
```

Again, PLEASE KEEP YOUR HANDS ON THE BIG RED BUTTON!

## Launching of two arms

- Please change launch file below to set the ip of robot, and all other parameters

```
roslaunch cssd_hardware two_arm.launch
```


## Notes

To test it with moveit on Rviz (terminal B)*

![alt text](/documentations/rviz.gif?)

```bash
# For ur10e,
roslaunch ur_modern_driver ur10e_bringup.launch robot_ip:=XXXXXXX
# Another terminal, For ur10e,
roslaunch cssdbot_ur10e_moveit_config realistic_minimal.launch
```

To purely tryout using moveit on rviz, remember:
- !!!PLAN BEFORE EXECUTING. SCALE YORU VELOCITY!!!!
- Remember `CURRENT STATE` should always be :`<current_state>`, `GROUP` should be: `MANIPULATOR`
- Joint states can be altered in `cssdbot_urxx_moveit_config/config/urxx.srdf`
