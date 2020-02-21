#ifndef MAP
#define MAP

#include "nav_msgs/OccupancyGrid.h" 
#include "geometry_msgs/Point.h" 

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

};


#endif
