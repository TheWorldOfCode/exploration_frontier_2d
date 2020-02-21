#ifndef FRONTIER
#define FRONTIER

// ROS
#include "nav_msgs/OccupancyGrid.h" 
#include "nav_msgs/GridCells.h" 

// MY
#include "../includes/global_definition.h" 
#include "../includes/map.h" 

// CPP STANDARD
#include <vector>

class Frontier 
{
  public:
    Frontier(int8_t unknown_marker, int8_t occupied_above);
    /****************************************************
     * Name: search                                                  
     * Description:  Locate frontier cell in the map
     * Parameters: 
     *                   nav_msgs::OccupancyGrid map - The Current map
     * Return:  
     * Throws:  
     * Errors: NO 
     ****************************************************/
    void search(const Map & map);
    ~Frontier(); 

#if DEBUG == 1
    /****************************************************
     * Name: getExploredArea                                                  
     * Description: Returns the Grid cells that are explored 
     * Parameters: 
     * Return:  Grid cells
     * Throws:  
     * Errors: 
     ****************************************************/
    nav_msgs::GridCells getExploredArea();

    /****************************************************
     * Name: getFrontierCells                                                  
     * Description: Return the frontier cells 
     * Parameters: 
     * Return:  Return the frontier cells
     * Throws:  
     * Errors:  
     ****************************************************/
    nav_msgs::GridCells getFrontierCells();
#endif
  private:

#if DEBUG == 1
    nav_msgs::GridCells explored_area;
    nav_msgs::GridCells frontier_cells;
#endif

    int8_t unknown_marker;
    int8_t occupied_above;

    std::vector<geometry_msgs::Point> vec_frontier_cells;

    /****************************************************
     * Name: isFree                                                  
     * Description: Check if a cell is free space
     * Parameters: 
     *                   int8_t cell - Map cell
     * Return:  True if the cell is free space
     * Throws:  
     * Errors: 
     ****************************************************/
    bool isFree(int8_t cell);

    /****************************************************
     * Name: isOccupied                                                  
     * Description: Check if a cell is occupied
     * Parameters: 
     *                   int8_t cell - Map cell
     * Return: True if cell is occupied
     * Throws:
     * Errors: 
     ****************************************************/
    bool isOccupied(int8_t cell);

    /****************************************************
     * Name: isUnknown                                                  
     * Description: Chech if a cell is unknown (-1) 
     * Parameters: 
     *                   int8_t map - Map cell
     * Return:  True if cell is unknown       
     * Throws: 
     * Errors:
     ****************************************************/
    bool isUnknown(int8_t);
};

#endif
