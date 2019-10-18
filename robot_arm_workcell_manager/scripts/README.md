# Test individual arm with this python code


## Test Arm with Requested Motion

```bash
# No ns
rosrun robot_arm_workcell_manager test_arm.py

# with arm1 ns
rosrun robot_arm_workcell_manager test_arm.py  /joint_states:=/arm1/joint_states

# with arm2 ns
rosrun robot_arm_workcell_manager test_arm.py  /joint_states:=/arm2/joint_states

```

also if with ns, or without, pls change the code here at `line 442`
```py
  ur10 = ArmManipulation( arm_ns_ = "/arm1/" )
```

Also, to control position change the `joint_goal`


## Test Workcell by making a `DispenserRequest`

TODO

```
rosrun robot_arm_workcell_manager request_item
```
