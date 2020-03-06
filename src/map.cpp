#include "../includes/map.h" 
#include "../includes/exception.h" 


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

geometry_msgs::Point Map::translateCellInToPosition(const vec2 v) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return translateCellInToPosition(v.getX(), v.getY()); 
}

vec2 Map::translateCellInToPositionVec(const vec2 v) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  geometry_msgs::Point cell_pos;
  cell_pos = translateCellInToPosition(v.getX(),v.getY()); 

  return vec2(cell_pos.x, cell_pos.y); 
}

void Map::translatePositionInToCell(const geometry_msgs::Point pos, int & x, int & y) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  x = (pos.x - map.info.origin.position.x) / map.info.resolution - map.info.resolution/2; 
  y = (pos.y - map.info.origin.position.y) / map.info.resolution - map.info.resolution/2; 
}

void Map::translatePositionInToCell(const vec2 v,  vec2 & o) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  int x = (v.getX()  - map.info.origin.position.x) / map.info.resolution - map.info.resolution/2; 
  int y = (v.getY()  - map.info.origin.position.y) / map.info.resolution - map.info.resolution/2; 

  o = vec2(x,y); 
}


int8_t Map::getCellData(const int index) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   if(index > map.info.height * map.info.width || 0 > index)
    throw(OutABound());

   return map.data[index];
}


int8_t Map::getCellData(const int x, const int y) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return getCellData(x + y * map.info.width); 
}


int8_t Map::getCellData(const vec2 v) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return getCellData(v.getX(), v.getY());  
}

uint32_t Map::getWidth() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return map.info.width;
}


uint32_t Map::getHeight() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return map.info.height;
}

float Map::getResolution() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   return map.info.resolution;
}
