#ifndef EXPLORATION
#define EXPLORATION

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "../includes/vec2.h"
#include "../includes/frontier.h" 
#include "../includes/map.h" 
#include "../includes/robot.h" 
#include "../includes/sensor_base.h" 

#include <memory>
#include <vector>
#include <list>

class Exploration 
{

  public:
    Exploration(const boost::shared_ptr<exploration_sensor_model::SensorModelBase> sensor, const std::shared_ptr<Frontier> frontier, const std::shared_ptr<Robot> robot, const std::shared_ptr<ros::NodeHandle> h);

    /****************************************************
     * Name: explore                                                  
     * Description: Selected a new target for the exploration
     * Parameters: 
*                   const Map & map - TODO
     * Return: The next goal pose
     * Throws: NoFrontier, NoFrontierCluster
     * Errors: 
     ****************************************************/
    geometry_msgs::Pose explore(const Map & map);
    void explore_debug(const Map & map);

    /****************************************************
     * Name: getPoseArray                                                  
     * Description: Get the array of goals 
     * Parameters: 
*                   geometry_msgs::PoseArray ar - TODO
     * Return:  
     * Throws: 
     * Errors:
     ****************************************************/
    void getPoseArray(geometry_msgs::PoseArray ar);

    ~Exploration();  

    /****************************************************
     * Name: getSensorVising
     * Description: Returns the cell "scanned" by the sensor 
     * Parameters: 
     * Return:  Scanned cells
     * Throws:  
     * Errors: 
     ****************************************************/
    std::vector<geometry_msgs::Point> getSensorVising() const;


    /****************************************************
     * Name: function name                                                  
     * Description: Get the sensor reading from the last exploration
     * Parameters: 
     * Return:  List of sensor reading 
     * Throws:  
     * Errors:  
     ****************************************************/
    std::vector<exploration_sensor_model::SensorReading> getSensorReading() const;

    /****************************************************
     * Name: goalReached                                                  
     * Description: Tell that the current goal is reached
     * Parameters: 
     * Return: 
     * Throws: 
     * Errors: 
     ****************************************************/
    void goalReached();


    geometry_msgs::Point exploration_goal()
    {
      return goal_list.front().position;
    }

  private:
    struct Tabel 
    {
      vec2 position;
      double heading;
      double reward;
      double cost;
      double goal;
      double total;
    };


    std::list<geometry_msgs::Pose> goal_list;

    Tabel center_goal;
    std::list<Tabel> center_goal_list;
    std::vector<Tabel> info; 

    boost::shared_ptr<exploration_sensor_model::SensorModelBase> sensor;
    std::shared_ptr<Frontier> frontier;
    std::shared_ptr<Robot> robot;

    std::vector<geometry_msgs::Point> sensor_vising; // Point on the map that the sensor will scan.
    std::vector<exploration_sensor_model::SensorReading> total_reading;


    double cost_modifier;

    double utility_modifier;

    double goal_modifier; 
    bool goal_wait;

    double cost(const vec2 center, const double heading) const; 

    /****************************************************
     * Name: reward                                                  
     * Description: calculate the reward for travaling to a given point
     * Parameters: 
     *                   const vec2 position - TODO
     *                   const exploration_sensor_model::SensorReading reading - Optimal reading for a given position
     * Return:  The reward of moving to a position
     * Throws:  
     * Errors: 
     ****************************************************/
    double reward(const vec2 position, const Map & map, double & heading);
    double reward(const vec2 position, const exploration_sensor_model::SensorReading reading) const;

    /****************************************************
     * Name: goalFunction                                          
     * Description: Calculate the reward of travelling to center according to goal
     * Parameters: 
*                   cost vec2 center - TODO
     * Return:  The reward
     * Throws:  
     * Errors:  
     ****************************************************/
    double goalFunction(const vec2 center);

    /****************************************************
     * Name: goalAvailable                                                  
     * Description: Check if the goal is available 
     * Parameters: 
*                   const Map & map - TODO
     * Return: True if it is 
     * Throws: 
     * Errors: 
     ****************************************************/
    bool goalAvailable(const Map & map);

      /****************************************************
     * Name: sortClusters                                                  
     * Description: Sort the cluster in order to remove cluster has have inf cost multiple times 
     * Parameters: 
     *                   std::vector<vec2> & cluster - TODO
     *                   std::vector<vec2> & sorted - TODO
     * Return:  Sorted list
     * Throws:  
     * Errors:  
     ****************************************************/
    void sortClusters(std::vector<vec2> & cluster, std::vector<std::tuple<vec2,int>> & sorted) const;

    /****************************************************
     * Name: cleanup_cluster_inf_count                                                  
     * Description: Cleaup the vector container the cluster with if cost
     * Parameters: 
     * Return: 
     * Throws: 
     * Errors: 
     ****************************************************/
    void cleanup_cluster_inf_count();
    /****************************************************
     * Name: findOptimalSensorDirection                                                  
     * Description: Find the optimal sensor direction from a reading
     * Parameters: 
     *                   const exploration_sensor_model::SensorReading & reading - TODO
     * Return:  A new Reading contains the optimal sensor reading
     * Throws: 
     * Errors:
     ****************************************************/
    exploration_sensor_model::SensorReading findOptimalSensorDirection(const exploration_sensor_model::SensorReading & reading) const;
};

#endif
