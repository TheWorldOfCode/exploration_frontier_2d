# Frontier Exploration 
This package implements a Cost-Utility-Goal approach for frontier exploration. The package depends on the ROS package move_base to move the robot and it uses a occupancy grid to find frontier cells. The current sensor model is a LIDAR and the parameters for it is compiled into and can only be change by changing the code. This package is a focused exploration which means it allow the user to set goals for the exploration to reach as faste as possible. 


## Approach 
This package uses a simple state machine which only contains four states. 


State    | Description
------------ | -------------
Defualt | Find new target cell
Move     | Moving the robot
Recovery  | Basic recovery 
Waiting   | Waiting for then movement to be done




### Frontier
In this state, it will find frontiers cells and cluster them together, in order to reduce the amount for calculates. There are minimum number of frontier cells before the cluster is accepted. Then the frontier cells are clustered, will the center of the cluster are used for the rest. The center is found by finding the "center of mass" for the cluster. 

Then for each center a theoretical scan with the cell as center is made with the help of map, this scan count the maximum number of unknown cells in 360 degrees around the robot, with a given angle resolution, this gives a histogram. This histogram is used to determine the optimal orientation for the robot to face when scanning, the optimal orientation is found by clustering bins together that contains a value larger end the mean and are next to each other. When the clustering is done the largest cluster must contain the optimal orientation and the optimal orientation must be at the center of the cluster. 

### Benefit function
The benefit function used in the Cost-Utility-Goal is 
![equation](http://www.sciweavers.org/tex2img.php?eq=B%28a%29%20%3D%20u%20%5Ccdot%20U%28a%29%20-%20c%20%5Ccdot%20C%28a%29%20%2B%20g%20%5Ccdot%20G%28a%2Cq%29&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)

Where U is the utility function, which gives the reward of travelling to cell a, and C is the cost function, which describes the cost of travelling to cell a. The G is the goal function, which return the reward of how close a cell a is to the goal q.  u, c and g is modifiers. 

The target cell for the exploration is the cell that maximise the benefit function.

#### Utility function
The Utility function return the the normalize count of the number of unknown cell when the robot has the optimal orientation in a given cell a. It is normalized by largest utility reward
#### Cost function 
The Cost function return normalized manhattan distance from the robot position to the cell a, where a valid path is used to determine the distance. It is normalized by the largest cost. 

#### Goal function
The Goal function return the normalized reward, which is given by the exp(-d) where d is the manhattan distance from the cell a to the goal q. It is normalized by the largest goal reward


## ROS Information

### Subscribed Topic
- /map (nav_msgs/OccupancyGrid) \
  Receiving the map from this topic
- /odom (nav_msgs/Odometry) \
  Receives the robot position (looking for base_footprint)
  - /frontier/goals (geometry_msgs/Pose_array) \
  Send the targets for the focused exploration. When the message is recieved is it latch and is overwrited if a new message are recieved. 
### Published Topics
  - /frontier_cells (nav_msgs/GridCells) \
  Send out frontier grid cells
  - /frontier_cluster_centers (nav_msgs/GridCells) \
  Send out center of the frontier clusters
  - /frontier_target_center (nav_msgs/GridCells) \
  Send out the center the robot is moving to 
  - /frontier_sensor_range (nav_msgs/GridCells) \
  Send out the theoretical sensor range (only a indicated)
  
### Parameters 
- ~/map_frame_id  (string) \
   The frame id of the map. Defualt: map
- ~/minimum_cluster_size (int)\
    The minimum size of a cluster before been consider for the exploration. Defualt: 5
- ~/unknown_marker (int8)\
    What number represent unknown cells in the map. Defualt: -1
- ~/occupied_above (int8) \
    The upper limit of number consider as occupied cells. Defualt: 25
- ~/sensor_range (float) \
    The range of the LIDAR in meters.
- ~/sensor_span (float) \
    The complete angle of view for the center. 
- ~/angle_resolution (float) \
     Angle resolution in the sensor model. Defualt: 0.1
- ~/cost_function_modifier (float) \
     The cost modifier in the benefit function. Defualt: 1
- ~/utility_function_modifier (float) \
    The utility modifier in the benefit function. Defualt: 1
- ~/goal_function_modifier (float) \
     The goal modifier in the benefit function. Defualt: 1
- ~/move_base_global (string) \
     Used for telling which frame the robot pose for the move base should be in. Defualt: map
- ~/goal_wait (bool) \
     Should the robot when it has reached the last goal or when there is no goal. Defualt: false
- ~/sensor_model (string) \
    Specify which plugin to used as the sensor model.  Defualt: exploration_sensor_model::SensorModelBase
- ~/navigation (string) \
    Specify which plugin to used as the sensor model. Defualt: exploration_navigation::NavigationBase
    
### Plugin
This package used two plugins, the sensor model and the navigation. The plugin loading works, but chancing the plugin through ROS parameter is not tested and it is not sure it works
