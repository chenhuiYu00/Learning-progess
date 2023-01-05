# 如我所见：yaml

``` c++
rrbot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
  //  type: joint_state_controller/JointStateController
    publish_rate: 50  

  # Position Controllers ---------------------------------------
  joint1_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint2
    pid: {p: 100.0, i: 0.01, d: 10.0}

```



``` c++
dh_arm_controller:
//  type: position_controllers/JointTrajectoryController
  joints:
     - arm_joint_1
     - arm_joint_2
     - arm_joint_3
     - arm_joint_4

  constraints:
      goal_time: 0.5
      stopped_velocity_tolerance: 0.2
  stop_trajectory_duration: 0
  state_publish_rate:  20
  action_monitor_rate: 10


```

