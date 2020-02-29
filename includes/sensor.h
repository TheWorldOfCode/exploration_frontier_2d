#ifndef SENSOR
#define SENSOR

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/GridCells.h>

#include "../includes/map.h" 
#include "../includes/vec2.h" 
#include "../includes/frontier.h" 
#include "../includes/histogram.h" 


struct SensorReading
{
   int total_count;
   std::vector<std::vector<geometry_msgs::Point>> cells;
   Histogram<int>count_of_angle_resolution;
   double upper_limit;
   double lower_limit;
   
   std::vector<geometry_msgs::Point> getAllCells() const;
   void append(std::vector<geometry_msgs::Point> & list) const;
}; 

class Sensor
{
  public:
    /****************************************************
     * Name: sensor                                                  
     * Description: Create a sensor object
     * Parameters: 
*                   double sensor_range - The range of the sensor in meters
*                   double sensor_span - The span for the sensor in radians 
*                   double angle_resolution - Angle step
     * Return: 
     * Throws: 
     * Errors:
     ****************************************************/
    Sensor(const double sensor_range, const double sensor_span, const double angle_resolution = 0.5);

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
    SensorReading calcNumberOfUnknownBetweenAngle(const vec2 center, const double lower_angle_limit, const double upper_angle_limit, const Map & map, const Frontier * mark, const int display_size = -1);
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
    SensorReading calcNumberOfUnknownCellTotal(const vec2 center, const Map & map, const Frontier * mark, const int display_size = -1);

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
    SensorReading calcNumberOfUnknownCellInDirection(const vec2 center, const double direction,  const Map & map, const Frontier * mark, const int display_size = -1);


    /****************************************************
     * Name: inRange                                                  
     * Description: Check if a point is in sensor range
     * Parameters: 
*                   const vec2 center - TODO
*                   const vec2 point - TODO
     * Return: True if the point is in range
     * Throws: 
     * Errors: 
     ****************************************************/
    bool inRange(const vec2 center, const vec2 point);


    ~Sensor(); 
    /****************************************************
     * Name: getForwardDirection                                                  
     * Description: Calculate the forward direction of a scan
     * Parameters: 
*                   const SensorReading & reading - TODO
     * Return: Return the for diretion in degress
     * Throws: 
     * Errors: 
     ****************************************************/
    double getForwardDirection(const SensorReading & reading) const;

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
    std::vector<geometry_msgs::Point> getSensorDataAroundIndexInReading2(const SensorReading & reading, const int index) const;
    SensorReading getSensorDataAroundIndexInReading(const SensorReading & reading, const int index) const;
  private:
    const double range;
    const double span;
    const double angle_resolution;

    double round_factor; // If the angle_resolution is 0.1 then the round factor is 10


    /****************************************************
     * Name: roundToAngleResolution                                                  
     * Description: Round the angle to the angle resolution
     * Parameters: 
*                   const double angle - Angle to be rounded   
*                   std::function<double(double)> round_method - ceil, floor, standard
     * Return: Rounded angle
     * Throws: 
     * Errors:
     ****************************************************/
    double roundToAngleResolution(const double angle, std::function<double(double)> round_method );
    /****************************************************
     * Overload of: roundToAngleResolution                                                  
     * Parameters: 
*                   const double angle - TODO
     ****************************************************/
    double roundToAngleResolution(const double angle);
}; 

#endif
