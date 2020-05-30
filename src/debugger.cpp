#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h" 
#include "nav_msgs/MapMetaData.h" 
#include "nav_msgs/GridCells.h" 
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>

#include "../includes/frontier.h" 
#include "../includes/map.h" 
#include "../includes/global_definition.h" 
#include "../includes/robot.h" 
#include "../includes/exploration.h" 
#include "../includes/exception.h" 
#include "../includes/sensor_base.h" 
#include "../includes/navigation_base.h" 

#include <string>
#include <memory>
namespace exploration 
{ 

  std::string g_map_frame_id; // The frame id for the map  (used for navigation and gridcells) 

  int g_minimum_cluster_size;
}


Map g_map;
bool g_map_available = false;

// Determine the size of the sensor scan to be showed, if -1 then the full ranges wil be showed.
int g_sensor_display_size = 20;

void getMap(const nav_msgs::OccupancyGrid& map)
{
  if(map.header.seq > g_map.map.header.seq || map.header.seq == 0) 
  { 
    ROS_INFO("New map" );
    g_map = Map(map);
    g_map_available = true;
  }
}  


int main(int argc, char **argv) 
{
  /***********************************************
   * Setup ROS                                   *
   ***********************************************/

  ros::init(argc, argv, "exploration_debugger");

  // access point to communications 
  ros::NodeHandle n;
  // Private namespace
  std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~"); 

  /***********************************************
   * Setup global variables                      *
   ***********************************************/ 

  std::string filepath_debug;
  if(!nh->getParam("debug_filepath", filepath_debug)) 
    filepath_debug = "";

  if(!nh->getParam("map_frame_id", exploration::g_map_frame_id)) 
    exploration::g_map_frame_id = "map" ;

  if(!nh->getParam("minimum_cluster_size", exploration::g_minimum_cluster_size) ) 
    exploration::g_minimum_cluster_size = 5;

  ROS_INFO("Param debug_filepath %s", filepath_debug.c_str()  ); 
  ROS_INFO("Param map_frame_id %s", exploration::g_map_frame_id.c_str());
  ROS_INFO("Param minimum_cluster_size %i", exploration::g_minimum_cluster_size);

  /***********************************************
   * Setup node                                  *
   ***********************************************/
  pluginlib::ClassLoader<exploration_sensor_model::SensorModelBase> sensor_plug_loader("exploration_frontier_2d", "exploration_sensor_model::SensorModelBase"); 
  pluginlib::ClassLoader<exploration_navigation::NavigationBase> navigation_plug_loader("exploration_frontier_2d", "exploration_navigation::NavigationBase"); 


  // Plug in 
  boost::shared_ptr<exploration_sensor_model::SensorModelBase> sensor; 
  boost::shared_ptr<exploration_navigation::NavigationBase> navigation;

  std::shared_ptr<Frontier> frontier;
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Exploration> explorer;

  try
  { 
    /*********************************************** * Creating instance of plugins                *
     ***********************************************/
    sensor = sensor_plug_loader.createInstance("exploration_sensor_model::LIDAR"); 
    navigation = navigation_plug_loader.createInstance("exploration_frontier_2d/exploration_navigation/NavigationMoveBase"); 

    /***********************************************
     * Initizering                                 *
     ***********************************************/
    frontier = std::make_shared<Frontier>(nh);
    robot    = std::make_shared<Robot>(navigation, nh) ;
    explorer = std::make_shared<Exploration>(sensor, frontier, robot, nh) ;

    sensor->initialize(nh); 
  }
  catch(ParameterNotInServer ex) 
  {
    ROS_FATAL("Error while setting up the node: %s", ex.what().c_str() ); 
    return 1;
  } 
  catch(pluginlib::PluginlibException & ex) 
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    return 1;
  } 
  /***********************************************
   * Creating topics and subscriber              *
   ***********************************************/

  ros::Subscriber sub  = n.subscribe("map", 1000, getMap);
  ros::Subscriber sub2 = n.subscribe("odom", 1000, &Robot::getOdom, robot.get());

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

  ros::Rate loop_rate(10); 

  nav_msgs::GridCells target; 
  nav_msgs::GridCells explored_area;

  explored_area.header.frame_id = exploration::g_map_frame_id;
  explored_area.header.seq      = -1;

  target.header.frame_id        = exploration::g_map_frame_id;
  target.header.seq             = -1;

  geometry_msgs::Pose goal;
  std::vector<geometry_msgs::Point> tmp;

  while(ros::ok())
  {

    if(g_map_available)
    {
      try
      {
        uint32_t start = ros::Time::now().toNSec();
        explorer->explore_debug(g_map); 
        ROS_INFO("%ld", ros::Time::now().toNSec() - start);
      } 
      catch(NoFrontier ex) 
      {
        ROS_WARN("%s stopping", ex.what()  );
      } 
      catch(NoFrontierCluster ex) 
      {
        ROS_WARN("%s stopping", ex.what()  );
      } 


      for(size_t i = 0; i < frontier->getNumberOfCluster() ; i++ )
      {
        if(i > frontier_clusters.size() - 1) 
          break;

        frontier_clusters[i].publish(frontier->getCluster(g_map, i) );

      } 

      frontier_cluster_centers.publish(frontier->getClusterCenterGridCells()); 
      frontier_cells.publish(frontier->getFrontierCells()); 
      explored_area.cell_width = g_map.map.info.resolution;
      explored_area.cell_height = g_map.map.info.resolution;
      explored_area.cells = explorer->getSensorVising(); 

      explored_area.header.seq++;
      frontier_sensor_range.publish(explored_area); 

      
      std::vector<exploration_sensor_model::SensorReading> r = explorer->getSensorReading(); 

      size_t i = 0;
      for( exploration_sensor_model::SensorReading & e  : r) 
      {
        e.count_of_angle_resolution.save(filepath_debug + "/cluster" + std::to_string(i) + ".txt"); 
        i++;
      }

      g_map_available = false;
    }  

    ros::spinOnce(); 
    loop_rate.sleep(); 

  } 

  return 0;
} 
