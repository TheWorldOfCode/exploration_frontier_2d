#ifndef EXPLORATION
#define EXPLORATION

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include "../includes/vec2.h"
#include "../includes/sensor.h" 
#include "../includes/frontier.h" 
#include "../includes/map.h" 
#include "../includes/robot.h" 

#include <memory>

class Exploration 
{

  public:
    Exploration(const std::shared_ptr<Sensor> sensor, const std::shared_ptr<Frontier> frontier, const std::shared_ptr<Robot> robot);

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

  private:
    std::shared_ptr<Sensor> sensor;
    std::shared_ptr<Frontier> frontier;
    std::shared_ptr<Robot> robot;

    std::vector<geometry_msgs::Point> sensor_vising; // Point on the map that the sensor will scan.

    double cost(const vec2 center) const; 

    /****************************************************
     * Name: reward                                                  
     * Description: calculate the reward for travaling to a given point
     * Parameters: 
*                   const vec2 position - TODO
*                   const SensorReading reading - Optimal reading for a given position
     * Return:  The reward of moving to a position
     * Throws:  
     * Errors: 
     ****************************************************/
    double reward(const vec2 position, const SensorReading reading) const;

    /****************************************************
     * Name: findOptimalSensorDirection                                                  
     * Description: Find the optimal sensor direction from a reading
     * Parameters: 
*                   const SensorReading & reading - TODO
     * Return:  A new Reading contains the optimal sensor reading
     * Throws: 
     * Errors:
     ****************************************************/
    SensorReading findOptimalSensorDirection(const SensorReading & reading) const;
};

#endif
