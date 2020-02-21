#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h" 
#include "nav_msgs/MapMetaData.h" 
#include "nav_msgs/GridCells.h" 


#include "../includes/frontier.h" 
#include "../includes/map.h" 

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
#endif


  ros::Rate loop_rate(10); 

/***********************************************
 * Setup node                                  *
 ***********************************************/
  Frontier frontier(-1 , 25);



  while(ros::ok())
  {

    if(g_map_available)
    {
      frontier.search(g_map); 

#if DEBUG == 1
      free_cells.publish(frontier.getExploredArea()); 
      frontier_cells.publish(frontier.getFrontierCells()); 
#endif
  int x,y;
  geometry_msgs::Point p;
  p = g_map.translateCellInToPosition(6,6); 
  g_map.translatePositionInToCell(p,x,y); 


  ROS_INFO("%s %f %f %s %i %i", "Cell to position", p.x, p.y, "Position to Cell", x,y);
    }  


    ros::spinOnce(); 
    loop_rate.sleep(); 


  } 


  return 0;

} 
