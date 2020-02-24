#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h" 
#include "nav_msgs/MapMetaData.h" 
#include "nav_msgs/GridCells.h" 

#include "../includes/frontier.h" 
#include "../includes/map.h" 
#include "../includes/global_definition.h" 
#include "../includes/sensor.h" 

#include <string>
Map g_map;
bool g_map_available = false;


void getMap(const nav_msgs::OccupancyGrid& map)
{

  g_map = Map(map);

  ROS_INFO("%s %i%s%i", "Receive Map, Size", map.info.width, "x", map.info.height  );
  g_map_available = true;
  
}  

int main(int argc, char **argv) 
{

// g_debug = true;
/***********************************************
 * Setup ROS                                   *
 ***********************************************/

  ros::init(argc, argv, "exploration");

  // access point to communications 
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 1000, getMap);
#if DEBUG == 1
  ros::Publisher free_cells = n.advertise<nav_msgs::GridCells>("free_cells" , 1000); 
  ros::Publisher frontier_cells = n.advertise<nav_msgs::GridCells>("frontier_cells" , 1000); 
  ros::Publisher frontier_cluster_centers = n.advertise<nav_msgs::GridCells>("frontier_cluster_centers" , 1000); 
  ros::Publisher frontier_sensor_range = n.advertise<nav_msgs::GridCells>("frontier_sensor_range" , 1000); 


  std::vector<ros::Publisher> frontier_clusters;

  for(int i = 0; i < 25; i++)  
  { 
    std::string s = "frontier_cluster_" + std::to_string((int) i);
    frontier_clusters.push_back(n.advertise<nav_msgs::GridCells>(s.c_str() , 1000));
  }

#endif


  ros::Rate loop_rate(10); 

  /***********************************************
   * Setup node                                  *
   ***********************************************/
  Frontier frontier(-1 , 25);
  Sensor sensor(8,270); 



  while(ros::ok())
  {

    if(g_map_available)
    {
      size_t frontiers = frontier.search(g_map); 
      ROS_INFO("Number of frontier %i", (int) frontiers); 
      size_t clustered = frontier.clustering(5);  
      ROS_INFO("NUMBER OF Cluster %i", (int)  clustered );
      size_t center = frontier.calcCenter(g_map);
      ROS_INFO("NUMBER OF Cluster centers %i", (int)  center);

      sensor.calcNumberOfCellTotal(frontier.getClusterCenter()[0], g_map, &frontier ); 

#if DEBUG == 1

      for(size_t i = 0; i < clustered; i++ )
      {
        if(i > frontier_clusters.size()) 
          break;

        frontier_clusters[i].publish(frontier.getCluster(g_map, i) );

      } 


      frontier_cells.publish(frontier.getFrontierCells()); 
      frontier_cluster_centers.publish(frontier.getClusterCenterGridCells()); 
      frontier_sensor_range.publish(sensor.getExploredArea()); 

#endif

    }  


    ros::spinOnce(); 
    loop_rate.sleep(); 


  } 


  return 0;

} 
