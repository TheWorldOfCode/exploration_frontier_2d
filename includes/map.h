#ifndef MAP
#define MAP

#include "nav_msgs/OccupancyGrid.h" 
#include "geometry_msgs/Point.h" 

#include "../includes/vec2.h" 

struct Map
{ 
  Map(); 
  Map(const nav_msgs::OccupancyGrid & map );

  nav_msgs::OccupancyGrid map;

  /****************************************************
   * Name: translatePositionInToPosition                                                  
   * Description: Cell into global coordinate frame 
   * Parameters: 
*                   int x - The x column cell
*                   int y - The y row cell
   * Return:  Point in the global coordinate frame 
   * Throws:  
   * Errors: 
   ****************************************************/
  geometry_msgs::Point translateCellInToPosition(int x, int y) const;
  /****************************************************
   * Parameters:
   *                v - 2D vector
   ****************************************************/
  geometry_msgs::Point translateCellInToPosition(const vec2 v) const;
  /****************************************************
   * Name: translateCellInToPositionVec                                                  
   * Description: move the position from map coordinate to the world
   * Parameters: 
*                   const vec2 v - TODO
   * Return: A vector in world corrdinate
   * Throws:  Which exception does it throw ?
   * Errors:  Current errors
   ****************************************************/
  vec2 translateCellInToPositionVec(const vec2 v) const;
  
  /****************************************************
   * Name: translatePositionInToCell                                                  
   * Description:  What does the function do ?
   * Parameters: 
*                   const geometry_msgs::Point pos - TODO
*                   int & x - TODO
*                   int & y - TODO
   * Return:  What does the function return ?
   * Throws:  Which exception does it throw ?
   * Errors:  Current errors
   ****************************************************/
  void translatePositionInToCell(const geometry_msgs::Point pos, int & x, int & y) const;
  void translatePositionInToCell(const vec2 v,  vec2 & o) const;


  /****************************************************
   * Name: getCellData                                                  
   * Description: Get the cell data for a given index
   * Parameters: 
*                   const int index - TODO
   * Return: cell data
   * Throws: OutABound
   * Errors: 
   ****************************************************/
  int8_t getCellData(const int index) const;
  /****************************************************
   * Overload of: getCellData                                                  
   * Parameters: 
*                   const int x - TODO
*                   const int y - TODO
   ****************************************************/
  int8_t getCellData(const int x, const int y) const;
  /****************************************************
   * Overload of: getCellData                                                  
   * Parameters: 
*                   const vec2 v - TODO
   ****************************************************/
  int8_t getCellData(const vec2 v) const;
  
  /****************************************************
   * Name: getWidth                                                  
   * Description: Get the width of the map
   * Parameters: 
   * Return: Return the width
   * Throws: 
   * Errors: 
   ****************************************************/
  uint32_t getWidth() const;

  /****************************************************
   * Name: getHeight                                                  
   * Description: Get the height of the map
   * Parameters: 
   * Return: Return the height
   * Throws: 
   * Errors: 
   ****************************************************/
  uint32_t getHeight() const;

  /****************************************************
   * Name: getResolution                                                  
   * Description: Get the resolution of the map
   * Parameters: 
   * Return: return the resolution
   * Throws: 
   * Errors: 
   ****************************************************/
  float getResolution() const;

};


#endif
