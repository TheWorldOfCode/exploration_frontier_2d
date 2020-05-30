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

enum State {MOVE, EXPLORE, WAIT, RECOVER};

Map g_map;
bool g_map_available = false;

// Determine the size of the sensor scan to be showed, if -1 then the full ranges wil be showed.
int g_sensor_display_size = 20;

void getMap(const nav_msgs::OccupancyGrid& map)
{

  g_map = Map(map);
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
  // Private namespace
  std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~"); 

 /***********************************************
  * Setup global variables                      *
  ***********************************************/ 

  if(!nh->getParam("map_frame_id", exploration::g_map_frame_id)) 
       exploration::g_map_frame_id = "map" ;

  if(!nh->getParam("minimum_cluster_size", exploration::g_minimum_cluster_size) ) 
    exploration::g_minimum_cluster_size = 5;

  std::string nav_plug, sensor_plug;
  if(!nh->getParam("navigation", nav_plug))
    nav_plug = "exploration_navigation::NavigationBase";

  if(!nh->getParam("sensor_model", sensor_plug))
    sensor_plug = "exploration_sensor_model::SensorModelBase";

  ROS_INFO("Param map_frame_id %s", exploration::g_map_frame_id.c_str());
  ROS_INFO("Param minimum_cluster_size %i", exploration::g_minimum_cluster_size);
  ROS_INFO("Param navigation %s", nav_plug.c_str());
  ROS_INFO("Param sensor_model %s", sensor_plug.c_str());

  /***********************************************
   * Setup node                                  *
   ***********************************************/
  pluginlib::ClassLoader<exploration_sensor_model::SensorModelBase> sensor_plug_loader("exploration_frontier_2d", sensor_plug); 
  pluginlib::ClassLoader<exploration_navigation::NavigationBase> navigation_plug_loader("exploration_frontier_2d", nav_plug); 


  // Plug in 
  boost::shared_ptr<exploration_sensor_model::SensorModelBase> sensor; 
  boost::shared_ptr<exploration_navigation::NavigationBase> navigation;

  std::shared_ptr<Frontier> frontier;
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Exploration> explorer;

  try
  { 
    /***********************************************
     * Creating instance of plugins                *
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
    navigation->initialize(nh); 
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
  ros::Subscriber sub3 = n.subscribe("frontier/goals", 1, &Exploration::getPoseArray, explorer.get());
#if DEBUG == 1
  ros::Publisher frontier_cells           = n.advertise<nav_msgs::GridCells>("frontier_cells" , 1000);
  ros::Publisher frontier_cluster_centers = n.advertise<nav_msgs::GridCells>("frontier_cluster_centers" , 1000);
  ros::Publisher frontier_target_center   = n.advertise<nav_msgs::GridCells>("frontier_target_center" , 1000);
  ros::Publisher frontier_sensor_range    = n.advertise<nav_msgs::GridCells>("frontier_sensor_range" , 1000);

#endif

  ros::Rate loop_rate(10); 

  nav_msgs::GridCells target; 
  nav_msgs::GridCells explored_area;

  explored_area.header.frame_id = exploration::g_map_frame_id;
  explored_area.header.seq      = -1;

  target.header.frame_id        = exploration::g_map_frame_id;
  target.header.seq             = -1;


  bool save = false;

  // Contain the goal position
  geometry_msgs::Pose goal;
  // Contains the start position 
  geometry_msgs::Point start;
  vec2 start_vec;

  // Used to save the point so the can be writed to target
  std::vector<geometry_msgs::Point> tmp;

  // used to save the x and y map coordinates 
  int x,y;


  // used to calculate recovering movements
  geometry_msgs::Point p; 
  int8_t recover = 0;

  State current_state = EXPLORE;

  while(ros::ok())
  {
    bool flag = true;

    if(g_map_available)
    {
      switch(current_state) 
      { 
        case MOVE: 
          // Publish the start and end position of the robot
          start_vec = robot->getLocalization();
          start.x = start_vec.getX();
          start.y = start_vec.getY();
          start.z = 0;

          tmp.clear(); 
          tmp.push_back(goal.position); 
          tmp.push_back(explorer->exploration_goal());
          tmp.push_back(start);

          target.cell_width  = g_map.getResolution();
          target.cell_height = g_map.getResolution();
          target.cells       = tmp;
          target.header.seq++;

          ROS_INFO("MOVE POSITION (%f %f)", goal.position.x, goal.position.y);

          robot->move(goal, &g_map, frontier);

          current_state = WAIT;
          break;
        case WAIT:

          if(!robot->isMoving())
          {
            if(robot->reachedGoal())
            {
              explorer->goalReached();
              current_state = EXPLORE;
            }
            else 
            {
              if((robot->getLocalization() - start_vec).squaredLength() < 6)
              {
                ROS_WARN("Robot has not move far from start position, starting recovery");
                current_state = RECOVER;
              }
              else
                current_state = EXPLORE;

            }
          }
          else
          {
            g_map.translatePositionInToCell(goal.position, x,y);
            if(frontier->isUnknown(g_map.getCellData(x,y)) || frontier->isOccupied(g_map.getCellData(x,y)))
            {
              ROS_INFO("Goal is now placed in unknown or occupied space");
              robot->cancelGoal();

              if(recover == 0)
                current_state = EXPLORE;
              else
                current_state = RECOVER;
            }
          }
          break;

        case RECOVER:
          current_state = MOVE;
          if(recover == 0)
          {
            p.x = 1;
            p.y = 0;
          }
          else if(recover == 1)
          {
            p.x = 1;
            p.y = 0;
          }
          else if(recover == 2)
          {
            p.x = 0;
            p.y = 1;
          }
          else if(recover == 3)
          {
            p.x = 0;
            p.y = 1;
          }
          else
            current_state = EXPLORE;

          goal = robot->moveRelative(p,0);
          recover++;
          break;
        case EXPLORE:
          try
          {
            goal = explorer->explore(g_map); 
          } 
          catch(NoGoal ex)
          {
            ROS_WARN("%s", ex.what());
            flag = false;
          }
          catch(CostInf ex)
          {
            ROS_WARN("%s", ex.what());
            current_state = RECOVER;
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


          if(flag)
            current_state = MOVE;
          recover = 0;

          frontier_cluster_centers.publish(frontier->getClusterCenterGridCells()); 

          explored_area.cell_width = g_map.getResolution();
          explored_area.cell_height = g_map.getResolution();
          explored_area.cells = explorer->getSensorVising(); 
          explored_area.header.seq++;
      }

    }  

    frontier_cells.publish(frontier->getFrontierCells()); 
    frontier_sensor_range.publish(explored_area); 
    frontier_target_center.publish(target);

    ros::spinOnce(); 


    loop_rate.sleep(); 


  } 

EXITINGS:
  return 0;

} 
