#include "../includes/sensor_lidar.h"
#include "../includes/exception.h" 

#include <vector>
#include <string>
#include <cmath>



exploration_sensor_model::LIDAR::LIDAR()
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  /* code */
}

void exploration_sensor_model::LIDAR::initialize(std::shared_ptr<ros::NodeHandle> h)
/*************************************************
 * See speficiation in the header 
 *************************************************/
{

  if(!h->getParam("sensor_range", range)) 
    throw ParameterNotInServer("sensor_range"); 

  if(!h->getParam("sensor_span", span ) ) 
    throw ParameterNotInServer("sensor_span"); 

  if(!h->getParam("angle_resolution", angle_resolution) ) 
    angle_resolution = 0.5;

  ROS_INFO("Param sensor_range %f"     , range);
  ROS_INFO("Param sensor_span %f"      , span);
  ROS_INFO("Param angle_resolution %f" , angle_resolution);

  std::string s = std::to_string(angle_resolution); 

  int decimals = 0;
  bool flag = false;
  for(int i=0; i < s.length() ; i++)
  {
    if(flag)
     decimals++; 
    else if (s[i] == '.' ) 
      flag = true;
  }
  
  round_factor = std::pow(10, decimals); 
  
}

exploration_sensor_model::SensorReading exploration_sensor_model::LIDAR::calcNumberOfUnknownBetweenAngle(const vec2 center, const double lower_angle_limit, const double upper_angle_limit, const Map & map, const Frontier * mark, const int display_size)
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   exploration_sensor_model::SensorReading scan;

   int count = 0;
   int sensor_range_cell = std::floor(range / map.map.info.resolution); 

   auto round_method_floor = [](double x) -> double {return std::floor(x); };
   auto round_method_ceil = [](double x) -> double {return std::ceil(x); };

   double lower_limit = roundToAngleResolution(lower_angle_limit, round_method_floor); 
   double upper_limit = roundToAngleResolution(upper_angle_limit , round_method_ceil); 

   std::vector<std::vector<geometry_msgs::Point>> point; 

//   ROS_WARN("%f %i %f %f" , angle_resolution, sensor_range_cell, lower_limit, upper_limit); 
   std::vector<int> angle_count;

   int tmp =0;
   for(double i = lower_limit; i < upper_limit; i+= angle_resolution)
   {
     tmp++;
     std::vector<geometry_msgs::Point> p;
     const vec2 dir = vec2(std::cos(i * M_PI /((double) 180)) , std::sin(i * M_PI /((double) 180))); 

     int count2 = 0;
     for(int j = 1; j < sensor_range_cell + 1; j++)
     {
       
       const vec2 point = center + dir * j;
       try
       {
         int8_t cell_data = map.getCellData(point); 

         if(mark->isOccupied(cell_data) ) 
           break;

         if(mark->isUnknown(cell_data) ) 
           count2++;
   
         if(j < display_size + 1 || display_size == -1) 
           p.push_back(map.translateCellInToPosition(point) );

       }
       catch (OutABound e )
       {
         ROS_ERROR("sensor %s", e.what()); 
         break;
       }   
     }  

     point.push_back(p); 

     angle_count.push_back(count2); 

     count += count2;
   } 


   scan.total_count = count;
   scan.cells = point;
   scan.count_of_angle_resolution = Histogram<int>(angle_count, count);
   scan.lower_limit = lower_limit;
   scan.upper_limit = upper_limit;


   return scan;

}

exploration_sensor_model::SensorReading exploration_sensor_model::LIDAR::calcNumberOfUnknownCellTotal(const vec2 center, const Map & map, const Frontier * mark, const int display_size)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return calcNumberOfUnknownBetweenAngle(center, 0, 360, map, mark, display_size); 
}

exploration_sensor_model::SensorReading exploration_sensor_model::LIDAR::calcNumberOfUnknownCellInDirection(const vec2 center, const double direction, const Map & map, const Frontier * mark, const int display_size)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return calcNumberOfUnknownBetweenAngle(center, direction - span/2, direction + span/2, map, mark, display_size); 
}

bool exploration_sensor_model::LIDAR::inRange(const vec2 center, const vec2 point)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return (point - center).squaredLength()  <= (range * range);  
}

exploration_sensor_model::LIDAR::~LIDAR()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  /* code */
}

std::vector<geometry_msgs::Point> exploration_sensor_model::LIDAR::getSensorDataAroundIndexInReading2(const exploration_sensor_model::SensorReading & reading, const int index) const
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  std::vector<geometry_msgs::Point> points;

  int lower_limit_index = index - std::round(span/(2 * angle_resolution) );
  int upper_limit_index = index + std::round(span/(2 * angle_resolution));

  for(int i=lower_limit_index; i < upper_limit_index + 1; i++)
  {
    int index = i;
    if(index < 0) 
      index += reading.cells.size(); 

    if(index >= reading.cells.size()) 
      index -= reading.cells.size(); 

    for(size_t j=0; j < reading.cells[index].size(); j++)
      points.push_back(reading.cells[index][j]); 
  }


  return points;
}

double exploration_sensor_model::LIDAR::getForwardDirection(const exploration_sensor_model::SensorReading & reading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  double heading = reading.lower_limit + span/2;
  return heading;
}

exploration_sensor_model::SensorReading exploration_sensor_model::LIDAR::getSensorDataAroundIndexInReading(const exploration_sensor_model::SensorReading & reading, const int index) const
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  exploration_sensor_model::SensorReading selected; 
  std::vector<std::vector<geometry_msgs::Point>> points;
  std::vector<int> data;

  selected.lower_limit = index * angle_resolution + reading.lower_limit - span/2;
  selected.upper_limit = index * angle_resolution + reading.lower_limit + span/2;

  int lower_limit_index = index - std::round(span/(2 * angle_resolution) );
  int upper_limit_index = index + std::round(span/(2 * angle_resolution));


  int total = 0;
  for(int i=lower_limit_index; i < upper_limit_index + 1; i++)
  {
    int index = i;
    if(index < 0) 
      index += reading.cells.size(); 

    if(index >= reading.cells.size()) 
      index -= reading.cells.size(); 

    try
    { 
      points.push_back(reading.cells[index]); 
      total += reading.count_of_angle_resolution.getData(index); 
      data.push_back(reading.count_of_angle_resolution.getData(index)); 
    }
    catch (std::out_of_range e) 
    {
      ROS_ERROR("exploration_sensor_model::LIDAR::getSensorDataAroundIndexInReading %s", e.what()); 
    } 
  }

  selected.total_count = total;
  selected.cells = points;
  selected.count_of_angle_resolution = Histogram<int>(data,total); 

  return selected;
}


double exploration_sensor_model::LIDAR::roundToAngleResolution(const double angle, std::function<double(double)> round_method)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  return round_method(angle * round_factor) / round_factor;  
}

double exploration_sensor_model::LIDAR::roundToAngleResolution(const double angle)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  auto round_method = [](double x) -> double {return x;};
  return roundToAngleResolution(angle, round_method ); 
}
