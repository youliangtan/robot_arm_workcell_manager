
# cssd_hardware
This package is to bring up all dependent codes/packages to spawn up the hardware. (RAWM, robot arm, camera....).

## Testing Arms on HARDWARE! (ToBeTested)

### Prerequisites
PLEASE KEEP YOUR HANDS ON THE BIG RED BUTTON!
Also, Download [ur_modern_driver with e series](https://github.com/AdmiralWall/ur_modern_driver/tree/kinetic_ur_5_4). Then install it.

### Launching of single arm

```bash
roslaunch cssd_hardware single_arm.launch
```

### Launching of two arms

- Please change launch file below to set the ip of robot, and all other parameters

```bash
roslaunch cssd_hardware two_arm.launch
```

---

### Others
This is to run arms without the launch file

#### 1. Terminal A (robot bringup):
```bash
# For ur10
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=198.168.88.XX
# For ur10e,
roslaunch ur_modern_driver ur10e_bringup.launch robot_ip:=198.168.88.XX
```

#### 2. Terminal B (moveit & Rviz):

*a) With Full RAWM package*
```bash
roslaunch robot_arm_workcell_manager demo.launch sim:=false enable_fake_joints_execution:=false arm_type:=ur10e
# then on terminal C, run RAWM
roslaunch robot_arm_workcell_manager robot_arm_workcell_manager.launch dispenser_name:=ur10e_001
```

Then on Terminal D: Pub dispenser request!!!! Good luck 

*b) Or, just test it with moveit on Rviz (terminal B)*

![alt text](/documentations/rviz.gif?)

```bash
# For ur10,
roslaunch cssdbot_ur10_moveit_config realistic_minimal.launch
# or For ur10e,
roslaunch cssdbot_ur10e_moveit_config realistic_minimal.launch
```

With pure tryout using moveit on rviz, remember:
- !!!PLAN BEFORE EXECUTING. SCALE YORU VELOCITY!!!!
- Remember `CURRENT STATE` should always be :`<current_state>`, `GROUP` should be: `MANIPULATOR`
- Joint states can be altered in `cssdbot_urxx_moveit_config/config/urxx.srdf`
