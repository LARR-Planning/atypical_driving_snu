# atypical_driving_snu

<p align="center">
<img src = "https://github.com/LARR-Planning/atypical_driving_snu/blob/master/img/data_structure.png" width="480"> 
</p>

## Instruction
### 1. Getting started 
#### Dependencies
* driving_msgs: 
  ```
  git clone https://gitlab.com/InstitutMaupertuis/graph_rviz_plugin
  ```  

* graph_rviz_plugin:
  ```
  https://gitlab.com/InstitutMaupertuis/graph_rviz_plugin
  ```

* misc
    ```
    sudo apt-get install ros-${ROS_DISTRO}-costmap-2d
    sudo apt-get install ros-${ROS_DISTRO}-pointcloud-to-laserscan
    sudo apt-get install ros-${ROS_DISTRO}-tf2-bullet 
    sudo apt-get install ros-${ROS_DISTRO}-jsk-visualization
    ```

* occupancy grid utils for 18.04

    ```
    https://github.com/clearpathrobotics/occupancy_grid_utils.git
    ```
* occupancy grid utils for 20.04

    ```
    https://github.com/LARR-Planning/occupancy_grid_utils.git
    ```

#### Test launch
```
roslaunch atypical_driving atypical_driving_snu_yugokri.launch 
```


### 2. Now, do your jobs! 

| Workforce      | Header           | Source  |
| ------------- |:-------------:| -----:|
| Boseong      | Wrapper.h | Wrapper.cpp |
| Jungwon      | Wrapper.h / GlobalPlanner.h      |  Wrapper.cpp / GlobalPlanner.cpp |
| Yunwoo | LocalPlanner.h      |    LocalPlanner.cpp |

Simple thing. your works are completing all the functions in your header and source files referring the summary [diagram](https://www.lucidchart.com/documents/edit/2f00f5b3-6e62-4ff4-8b19-dc401daf80f8/GdaP.vOKzV1F). 
All the example implementation can be found in the source files. Please understand the overall flow of the codes. 

### Parameter description 

### B

#### 1) lane-dependent 
* `use_keti_velocity` : Setting this to true will use the velocity of `detected_object` message as the constant velocity of a obstacle path. 
The velocity of the obstacle path is the average of the keti velocities. Using this, the target position is predicted. In case of the pose,
 use just fit using a linear regression of the raw data of keti observation. Setting this parameter to false will use linear regression for 
 all translation and orientation altogther while using coefficients of the fitting model for the representative speed (for dynamic object thresholding or visualization)
* `map/pcl_lx[ly]` : pointcloud process bound (+lx/2 -lx/2). This region is cropped for `velodyne_points_snu` . The reference frame is `velodyne`
* `map/pcl_z_min` : we first set the `pnt.z` < `pcl_z_min` as the candidates for the ground pointcloud. We run ransac to this points only 
* `map/pcl_dbscan_minpnts` : the minimum neighbors in the search radius for not to be removed as speckle
* `map/pcl_dbscan_eps`: the radius search bound for speckle removal
* `use_ransac` : whether to use ransac over original simple cropping  
* `map/ransac_post_inclusion_offset` : distance offset for including additional ground points (distance from a points to the plane found from ransac) - large: more ground / small: less ground 
* `map/ransac_distance_threshold` : the parameter used for ransac algorithm. large = loose plane / small = more strict for judging inliners 

#### 2) lane-independent

* `example...` : example... 


### J
* `global_planner/car_width` : When construcing laneTree, car width is used to determine whether two node are connected. (m)
* `global_planner/period`: ????
* `global_planner/grid_resolution` : Lane grid resolution (m)
* `global_planner/max_steering_angle` : Maximum angle between two adjacent points in smoothLane (rad)
* `global_planner/smoothing_cliff_min_bias` : If distance between the waypoint and a point on unsmoothed lane is larger than this parameter, set that point as the start point of line smoothing. (m)
* `global_planner/smoothing_cliff_ratio` : If distance between the waypoint and a point on unsmoothed lane is larger than this parameter, set that point as the end point of line smoothing.
* `global_planner/smoothing_distance`: Range of line smoothring (m)
* `global_planner/start_smoothing_distance`: Not used in this version
* `global_planner/corridor_width_min`: If lane is blocked by obstacle, minimum width of corridor should be larger than this parameter. (m) 
* `global_planner/corridor_width_dynamic_min`: If lane is blocked by object, minimum width of corridor should be larger than this parameter. (m) 
* `global_planner/safe_distance`: Distance between smoothLane and blocked point if lane is blocked by obstacle or object. (m) 
* `global_planner/nominal_acceleration`: Nominal acceleration for determining time segment of smoothLane. (m/s^2)
* `global_planner/object_velocity_threshold`: If object is faster than this parameter, object is determined to dynamic object (m/s)
* `global_planner/max_obstacle_prediction_query_size`: obstacle prediction query size

### Y
* `local_planner/horizon`: MPC horizon (seconds)
* `local_planner/ts`: MPC time step (seconds)
* `local_planner/max_steer`: Max Steering Angle
* `local_planner/max_accel`: Max Acceleration
* `local_planner/min_accel`: Min Acceleration (minus)
* `local_planner/N_corr`: Number of Corridor
* `local_planner/isRearWheel`: RealCar:0, Airsim:1
* `local_planner/dyn_obst_range`: Dynamic Obstacle Consideration Range

* `smoothing_type`: it has value: 0(exponential average), 1(moving average), 2(expoential+moving average), 3(ignore small handle angle)
* `smooth_weight`: when you use exponential average, this values should satisfy 0<value<1, and higher value is regarded as giving more weight to current value
* `moving_horizon`: when you use moving average, this means size of horizon.
* `ignore_angle`: Amount of Ignoring small angle (handle)


### Known issues 

#### Boseong
1. Slope ground segmentation speed check and threading 
2. Costmap 2d reference frame 
3. Lane data datatype + lane customization  
4. Random start random goal 2d nav goal 

#### ChanageLog  
*  Async spinner used but cbOccuMap and cbOccuUpdate are treated in the same nodehandle. Modified 


#### Yunwoo 



#### Jungwon 


