#ifndef EXPLORATION_FRONTIER_2D_SENSOR_BASE
#define EXPLORATION_FRONTIER_2D_SENSOR_BASE

#include <geometry_msgs/Point.h>
#include <ros/ros.h> 

#include <memory>

#include "../includes/histogram.h" 
#include "../includes/map.h" 
#include "../includes/frontier.h" 

namespace exploration_sensor_model
{
  struct SensorReading
  {
    int total_count;
    std::vector<std::vector<geometry_msgs::Point>> cells;
    Histogram<int>count_of_angle_resolution;
    double upper_limit;
    double lower_limit;

    std::vector<geometry_msgs::Point> getAllCells() const
    {
      std::vector <geometry_msgs::Point> new_cells;
      for(size_t i=0; i < cells.size() ; i++)
      {
        for( geometry_msgs::Point point : cells[i]) 
          new_cells.push_back(point);
      }

      return new_cells;
    } 
    void append(std::vector<geometry_msgs::Point> & list) const 
    {
      for( std::vector<geometry_msgs::Point> p_list : cells) 
      {
        for( geometry_msgs::Point p : p_list) 
          list.push_back(p); 
      }
    } 
  }; 

  class SensorModelBase
  {

    public:
      /****************************************************
       * Name: initialize                                                  
       * Description: Used to initialze the sensor
       * Parameters: 
       *                   std::shared_ptr<ros::NodeHandle> h - TODO
       * Return: 
       * Throws:
       * Errors:
       ****************************************************/
      virtual void initialize(std::shared_ptr<ros::NodeHandle> h) = 0;

      /****************************************************
       * Name: calcNumberOfUnknownBetweenAngle                                                  
       * Description: Calculate the number of unknown cell between to angles
       * Parameters: 
       *                   const vec2 center              - The center of the scan
       *                   const double lower_angle_limit - The lower angle limit of the scan
       *                   const double upper_angle_limit - The upper angle limit of the scan
       *                   const Map & map                - The current map
       *                   const Frontier * mark          - TODO
       *                   const int display_size         - The size of display range of the sensor if - 1 when full ranges
       * Return: The count of cells
       * Throws:  
       * Errors: 
       ****************************************************/
      virtual SensorReading calcNumberOfUnknownBetweenAngle(const vec2 center, const double lower_angle_limit, const double upper_angle_limit, const Map & map, const Frontier * mark, const int display_size = -1) = 0;
      /****************************************************
       * Name: calcNumberOfUnknownCellTotal
       * Description: calculate the complete number of cell marked within range
       * Parameters: 
       *              const vec2 center      - The center of scan
       *                   const Map & map        - The current map
       *                   const int mark         - The mark that identify the cells to count
       *                   const int display_size - The size of display range of the sensor if - 1 when full ranges
       * Return: The count of cells
       * Return:  A Sensor  reading container the total number of unknown cell, position, and the amount of cell foreach angle
       * Throws:
       * Errors:
       ****************************************************/
      virtual SensorReading calcNumberOfUnknownCellTotal(const vec2 center, const Map & map, const Frontier * mark, const int display_size = -1) = 0;
      /****************************************************
       * Name: calcNumberOfUnknownCellInDirection                                                  
       * Description: Calculate the number of unknown cells the sensor will se in a specific direction
       * Parameters: 
       *                   const vec2 center      - The Center of the scan
       *                   const double direction - The direction in degress
       *                   const Map & map        - The current map
       *                   const Frontier * mark  - TODO
       *                   const int display_size - The size of display range of the sensor if - 1 when full ranges
       * Return:  A Sensor  reading container the total number of unknown cell, position, and the amount of cell foreach angle
       * Throws:  
       * Errors:  
       ****************************************************/

      virtual SensorReading calcNumberOfUnknownCellInDirection(const vec2 center, const double direction,  const Map & map, const Frontier * mark, const int display_size = -1) = 0;
      /****************************************************
       * Name: inRange                                                  
       * Description: Check if a point is in sensor range
       * Parameters: 
       *                   const geometry_msgs::Point center - TODO
       *                   const geometry_msgs::Point point - TODO
       * Return: True if the point is in range
       * Throws: 
       * Errors: 
       ****************************************************/
      virtual bool inRange(const vec2 center, const vec2 point) = 0;

      virtual ~SensorModelBase() {}  

      /****************************************************
       * Name: getForwardDirection                                                  
       * Description: Calculate the forward direction of a scan
       * Parameters: 
       *                   const SensorReading & reading - TODO
       * Return: Return the for diretion in degress
       * Throws: 
       * Errors: 
       ****************************************************/
      virtual double getForwardDirection(const SensorReading & reading) const = 0;
      /****************************************************
       * Name: getSensorDataAroundIndexInReading                                                  
       * Description: Detemine the cells used when center is at index 
       * Parameters: 
       *                   const SensorReading & reading - TODO
       *                   const int index - TODO
       * Return: 
       * Throws:
       * Errors:
       ****************************************************/
      virtual SensorReading getSensorDataAroundIndexInReading(const SensorReading & reading, const int index) const = 0;
    protected:
      SensorModelBase() {} 

  };

} 

#endif
