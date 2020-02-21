#include "../includes/map.h" 



Map::Map()
{}  
Map::Map(const nav_msgs::OccupancyGrid & map) : map(map)  
{ 
}   


geometry_msgs::Point Map::translateCellInToPosition(int x, int y) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  geometry_msgs::Point cell_pos;
  cell_pos.x = x * map.info.resolution + map.info.origin.position.x + map.info.resolution/2;  
  cell_pos.y = y * map.info.resolution + map.info.origin.position.y + map.info.resolution/2;
  cell_pos.z = map.info.origin.position.z;

  return cell_pos;
}


void Map::translatePositionInToCell(const geometry_msgs::Point pos, int & x, int & y) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  x = (pos.x - map.info.origin.position.x) / map.info.resolution - map.info.resolution/2; 
  y = (pos.y - map.info.origin.position.y) / map.info.resolution - map.info.resolution/2; 
}
