
#include "../includes/frontier.h"
#include "geometry_msgs/Pose.h" 
#include "ros/ros.h" 

Frontier::Frontier(int8_t unknown_marker, int8_t occupied_above) : unknown_marker(unknown_marker), occupied_above(occupied_above)  
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
#if DEBUG == 1
  explored_area.header.frame_id = "map";
  explored_area.header.seq = -1;

  frontier_cells.header.frame_id = "map"; 
  frontier_cells.header.seq = -1;
#endif
}


void Frontier::search(const Map & map)
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  const uint32_t width    = map.map.info.width;
  const uint32_t height   = map.map.info.height;

#if DEBUG == 1
  explored_area.cell_width  = map.map.info.resolution;
  explored_area.cell_height = map.map.info.resolution;

  frontier_cells.cell_width  = map.map.info.resolution;
  frontier_cells.cell_height = map.map.info.resolution;

  std::vector<geometry_msgs::Point> free_cells;
#endif

  std::vector<geometry_msgs::Point> l_frontier_cells;
       
  int x = 0, x_offset = 0,y_offset = 0, z = 0;
  while(y_offset < height) 
  { 

    int8_t current_cell = map.map.data[x]; 
    geometry_msgs::Point cell_pos = map.translateCellInToPosition((x - (width * x_offset)), y_offset );

#if DEBUG == 1
    if(isFree(current_cell))
    {

      free_cells.push_back(cell_pos); 
    }  
#endif

/***********************************************
 * Detect if cell is a frontier cell           *
 ***********************************************/
    if(isFree(current_cell))
    {
      ROS_INFO("Data %i ", current_cell); 
      if(isUnknown(map.map.data[x - 1]) ) 
        l_frontier_cells.push_back(cell_pos);
      else if (isUnknown(map.map.data[x + 1]))
        l_frontier_cells.push_back(cell_pos);
      else if (isUnknown(map.map.data[x - width])) 
        l_frontier_cells.push_back(cell_pos);
      else if (isUnknown(map.map.data[x + width])) 
        l_frontier_cells.push_back(cell_pos);
    }  

    /***********************************************
     * Current to next cell                        *
     ***********************************************/
    x++;
    if(x == width * (x_offset + 1)) 
    {
      x_offset++;
      y_offset++; 
    } 
  }

#if DEBUG == 1
  explored_area.cells = free_cells;
  explored_area.header.seq++;

  frontier_cells.cells = l_frontier_cells;
  frontier_cells.header.seq++;
#endif

}


Frontier::~Frontier()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  /* code */
}


#if DEBUG == 1
nav_msgs::GridCells Frontier::getExploredArea( )
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return explored_area;
}

nav_msgs::GridCells Frontier::getFrontierCells( )
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return frontier_cells;
}
#endif 
bool Frontier::isFree(int8_t cell)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return cell < occupied_above && cell > -1 ? true : false;
}

bool Frontier::isOccupied(int8_t cell)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return cell > occupied_above ? true : false;
}

bool Frontier::isUnknown(int8_t cell)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return cell == -1 ? true : false;
}

