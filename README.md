# atypical_driving_snu

<p align="center">
<img src = "https://github.com/LARR-Planning/atypical_driving_snu/blob/master/img/data_structure.png" width="480"> 
</p>

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

Simple thing. your works are completing all the functions in your header and source files referring the summary [diagram](https://www.lucidchart.com/documents/edit/2f00f5b3-6e62-4ff4-8b19-dc401daf80f8/GdaP.vOKzV1F). 
All the example implementation can be found in the source files. Please understand the overall flow of the codes. 

### Parameter description 

### Boseong 

* `use_keti_velocity` : Setting this to true will use the velocity of `detected_object` message as the constant velocity of a obstacle path. 
The velocity of the obstacle path is the average of the keti velocities. Using this, the target position is predicted. In case of the pose,
 use just fit using a linear regression of the raw data of keti observation. Setting this parameter to false will use linear regression for 
 all translation and orientation altogther while using coefficients of the fitting model for the representative speed (for dynamic object thresholding or visualization)
    
  
### Jungown 


### Yunwoo 
* `smoothing_type`: it has value: 0(exponential average), 1(moving average), 2(expoential+moving average), 3(ignore small handle angle)
* `smooth_weight`: when you use exponential average, this values should satisfy 0<value<1, and higher value is regarded as giving more weight to current value
* `moving_horizon`: when you use moving average, this means size of horizon.
* `ignore_angle`: Amount of Ignoring small angle (handle)
