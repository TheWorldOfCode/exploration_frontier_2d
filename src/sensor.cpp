#include "../includes/sensor.h"

#include <vector>
#include <cmath>

Sensor::Sensor(const double sensor_range, const double sensor_span, const double angle_resolution) : range(sensor_range), span(sensor_span), angle_resolution(angle_resolution) 
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  explored_area.header.frame_id       = "map";
  explored_area.header.seq            = -1;
}

int Sensor::calcNumberOfCellTotal(const vec2 center, const Map & map, const Frontier * mark)
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   int count = 0;
   int sensor_range_cell = std::floor(range / map.map.info.resolution); 
   std::vector<geometry_msgs::Point> p; 

   vec2 left_up, left_down, right_up, right_down;

   int y_offset = center.getY() + sensor_range_cell;
   int x_offset = center.getX() + sensor_range_cell;

   left_up      = center + vec2( x_offset, -y_offset);
   left_down    = center + vec2( x_offset,  y_offset);
   right_up     = center + vec2(-x_offset, -y_offset);
   right_down   = center + vec2(-x_offset,  y_offset);

   ROS_INFO("center %s, left up %s, left down %s, right up %s, right down %s", center.print().c_str(), left_up.print().c_str(), left_down.print().c_str(), right_up.print().c_str(), right_down.print().c_str()); 
   for(double i = 0; i < angle_resolution; i+= angle_resolution)
   {
     const vec2 end = center + vec2((int) (sensor_range_cell * std::cos(i * 180/M_PI)) , (int)( sensor_range_cell * std::sin(i * 180/M_PI)) ); 
     vec2 step = end - center;  
//     step.normalize(); 

     vec2 current_cell = center;
     for(vec2 move = center + step; move != end; move = move + step) 
     {
       if(current_cell != move.round()) { 
         ROS_INFO("CELL %s", move.round().print().c_str()); 
         p.push_back(map.translateCellInToPosition(move.getX(), move.getY()) );
         current_cell = move.round(); 
       }
     }

     p.push_back(map.translateCellInToPosition(end.getX(), end.getY()) );

   } 


   explored_area.header.seq++;
   explored_area.cells = p;
   explored_area.cell_height = map.map.info.resolution;
   explored_area.cell_width = map.map.info.resolution;

   return 0;
}

nav_msgs::GridCells Sensor::getExploredArea()
{
  return explored_area;
}  
Sensor::~Sensor()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  /* code */
}
