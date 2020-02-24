#ifndef SENSOR
#define SENSOR

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/GridCells.h>

#include "../includes/map.h" 
#include "../includes/vec2.h" 
#include "../includes/frontier.h" 

class Sensor
{
  public:
    /****************************************************
     * Name: sensor                                                  
     * Description: Create a sensor object
     * Parameters: 
*                   double sensor_range - The range of the sensor in meters
*                   double sensor_span - The span for the sensor in radians 
*                   double angle_resolution - TODO
     * Return: 
     * Throws: 
     * Errors:
     ****************************************************/
    Sensor(const double sensor_range, const double sensor_span, const double angle_resolution = 0.5);

    /****************************************************
     * Name: calcNumberOfCellTotal
     * Description: calculate the complete number of cell marked within range
     * Parameters: 
     *              center - The center of scan
*                   const Map & map - The current map
*                   const int mark - The mark that identify the cells to count
     * Return: The count of cells
     * Throws:
     * Errors:
     ****************************************************/
    int calcNumberOfCellTotal(const vec2 center, const Map & map, const Frontier * mark);

    nav_msgs::GridCells getExploredArea(); 

    ~Sensor(); 
  private:
    const double range;
    const double span;
    const double angle_resolution;

    nav_msgs::GridCells explored_area;
}; 

#endif
