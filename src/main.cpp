#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h" 
#include "nav_msgs/MapMetaData.h" 
#include "nav_msgs/GridCells.h" 
#include <nav_msgs/Odometry.h>

#include "../includes/frontier.h" 
#include "../includes/map.h" 
#include "../includes/global_definition.h" 
#include "../includes/sensor.h" 
#include "../includes/robot.h" 
#include "../includes/exploration.h" 
#include "../includes/exception.h" 

#include <string>
#include <memory>
Map g_map;
bool g_map_available = false;

// Determine the size of the sensor scan to be showed, if -1 then the full ranges wil be showed.
int g_sensor_display_size = 20;

void getMap(const nav_msgs::OccupancyGrid& map)
{

  g_map = Map(map);

  ROS_INFO("%s %i%s%i", "Receive Map, Size", map.info.width, "x", map.info.height  );
  g_map_available = true;

}  


int main(int argc, char **argv) 
{

  /***********************************************
   * Setup node                                  *
   ***********************************************/
  std::shared_ptr<Frontier> frontier    = std::make_shared<Frontier>(-1 , 25);
  std::shared_ptr<Sensor> sensor        = std::make_shared<Sensor>(8,270);
  std::shared_ptr<Robot> robot          = std::make_shared<Robot>() ;
  std::shared_ptr<Exploration> explorer = std::make_shared<Exploration>(sensor, frontier, robot) ;

  // g_debug = true;
  /***********************************************
   * Setup ROS                                   *
   ***********************************************/

  ros::init(argc, argv, "exploration");

  // access point to communications 
  ros::NodeHandle n;

  ros::Subscriber sub  = n.subscribe("map", 1000, getMap);
  ros::Subscriber sub2 = n.subscribe("odom", 1000, &Robot::getOdom, robot.get());
#if DEBUG == 1
  ros::Publisher free_cells               = n.advertise<nav_msgs::GridCells>("free_cells" , 1000);
  ros::Publisher frontier_cells           = n.advertise<nav_msgs::GridCells>("frontier_cells" , 1000);
  ros::Publisher frontier_cluster_centers = n.advertise<nav_msgs::GridCells>("frontier_cluster_centers" , 1000);
  ros::Publisher frontier_target_center   = n.advertise<nav_msgs::GridCells>("frontier_target_center" , 1000);
  ros::Publisher frontier_sensor_range    = n.advertise<nav_msgs::GridCells>("frontier_sensor_range" , 1000);


  std::vector<ros::Publisher> frontier_clusters;

  for(int i = 0; i < 25; i++)  
  { 
    std::string s = "frontier_cluster_" + std::to_string((int) i);
    frontier_clusters.push_back(n.advertise<nav_msgs::GridCells>(s.c_str() , 1000));
  }

#endif
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); 

  while(!ac.waitForServer(ros::Duration(5.0) ) )
  {
    ROS_INFO("Waiting for the move_base action server to come up" );
  } 

  ros::Rate loop_rate(10); 

  nav_msgs::GridCells target; 
  nav_msgs::GridCells explored_area;

  explored_area.header.frame_id = "map";
  explored_area.header.seq      = -1;

  target.header.frame_id        = "map";
  target.header.seq             = -1;


  bool save = false;

  geometry_msgs::Pose goal;
  std::vector<geometry_msgs::Point> tmp;


  int state = 0;
  while(ros::ok())
  {

    if(g_map_available)
    {
      switch(state) 
      { 
        case 1: 
          tmp.clear(); 
          tmp.push_back(goal.position); 

          target.cell_width  = g_map.map.info.resolution;
          target.cell_height = g_map.map.info.resolution;
          target.cells       = tmp;
          target.header.seq++;

          frontier_target_center.publish(target);

          ac.sendGoal(robot->move(goal)); 

          ac.waitForResult(); 

          if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Reached goal");
          else
            ROS_INFO("Didn't reach the goal  ");

          ROS_INFO("Canceling all goals"); 
          ac.cancelAllGoals(); 


          if(frontier->getNumberOfCluster() == 0 ) 
            state = 2;
          else
            state = 0;
          break;
        case 2:
          break;

        default:
          try
          {
            goal = explorer->explore(g_map); 
          } 
          catch(NoFrontier ex) 
          {
            ROS_WARN("%s stopping", ex.what()  );
            goto EXITINGS;
          } 
          catch(NoFrontierCluster ex) 
          {
            ROS_WARN("%s stopping", ex.what()  );
            goto EXITINGS;
          } 
          state = 1;


#if DEBUG == 1

          for(size_t i = 0; i < frontier->getNumberOfCluster() ; i++ )
          {
            if(i > frontier_clusters.size()) 
              break;

            frontier_clusters[i].publish(frontier->getCluster(g_map, i) );

          } 

          frontier_cells.publish(frontier->getFrontierCells()); 
          frontier_cluster_centers.publish(frontier->getClusterCenterGridCells()); 
          explored_area.cell_width = g_map.map.info.resolution;
          explored_area.cell_height = g_map.map.info.resolution;
          explored_area.cells = explorer->getSensorVising(); 

          explored_area.header.seq++;
          frontier_sensor_range.publish(explored_area); 
#endif
      }

    }  


    ros::spinOnce(); 


    loop_rate.sleep(); 


  } 

EXITINGS:

  return 0;

} 
