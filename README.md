# Frontier Exploration 
This package implements a cost-utility approach for frontier exploration. The package depends on the ROS package move_base to move the robot and it uses a occupancy grid to find frontier cells. The current sensor model is a LIDAR and the parameters for it is compiled into and can only be change by changing the code. 


## Approach 
This package uses a simple state machine which only contains two states. 


State    | Description
------------ | -------------
frontier | Find new position to move to
Move     | Moving the robot



### Frontier
In this state, it will find frontiers cells and cluster them together, in order to reduce the amount for calculates. There are minimum number of frontier cells before the cluster is accepted. Then the frontier cells are clustered, will the center of the cluster are used for the rest. The center is found by finding the "center of mass" for the cluster. 

Then for each center a theoretical scan by center is made with the help of map, this scan count the maximum number of unknown cells in 360 degrees around the robot, with a given angle resolution, this gives a histogram. This histogram is used to determine the optimal direction for the robot to face when scanning, the optimal direction is found by clustering bins together that contains a value larger end the mean and are next to each other. When the clustering is done the largest cluster must container the optimal direction and the optimal direction must be at the center of the cluster. 


Now the cost - utility function can be used, the function is defines as 


![equation](http://www.sciweavers.org/tex2img.php?eq=E%28q%2Cp%2Cs%29%20%3D%20U%28s%29%20-%20C%28q%2Cp%29&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)

Where U is the utility function which take the histogram for the optimal direction s, and C is the cost function which take the robot position q and the target position p. 

The cluster center that will be moved to will be maximise this function.

#### Utility function
The Utility function is very simple are used the histogram for the sensor scans, it count the amount for unknown cell in sensor range from the optimal direction. This count want the Utility function returns. 


#### Cost function 
The Cost function return the squared euclidean distance between the robot position and the target cells. Rotation doesn't count as a costed. 



## ROS Information

# Subscribed Topic
- /map (nav_msgs/OccupancyGrid)
  Receiving the map from this topic
- /odom (nav_msgs/Odometry)
  Receives the robot position (looking for base_footprint)
# Published Topics
  - /frontier_cells (nav_msgs/GridCells)
  Send out frontier grid cells
  - /frontier_cluster_centers (nav_msgs/GridCells)
  Send out center of the frontier clusters
  - /frontier_target_center (nav_msgs/GridCells)
  Send out the center the robot is moving to 

  - /frontier_sensor_range (nav_msgs/GridCells)
  Send out the theoretical sensor range (only a indicated)
  - /frontier_cluster_X (nav_msgs/GridCells)
  Send out cluster X, X is a number between 0 and 24 
  
