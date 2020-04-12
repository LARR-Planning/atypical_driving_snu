# atypical_driving_snu

## Instruction
### 1. Getting started 
#### Dependencies until now 
* Octomap 

#### Test run 
```
$ cd ~/catkin_ws/
$ catkin build atypical_driving
$ rosrun atypical_driving atypical_driving_test 
$ rostopic pub -r 30 /atypical_planning_test/car_pose_cov geometry_msgs/PoseWithCovariance "pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
```

In the terminal, you will see ROS_WARNINGS the printed output which says that 

1. *Planning* and *modification* in the subset of callback cannot be performed at the same time.
2. Planning output *update to p_base* and ros *message update from p_base* cannot be done at the same time.


### 2. Now, do your jobs! 

| Workforce      | Header           | Source  |
| ------------- |:-------------:| -----:|
| Boseong      | Wrapper.h | Wrapper.cpp |
| Jungwon      | Wrapper.h / GlobalPlanner.h      |  Wrapper.cpp / GlobalPlanner.cpp |
| Yunwoo | LocalPlanner.h      |    LocalPlanner.cpp |


###  3. 
