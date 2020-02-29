#include "../includes/exploration.h" 
#include "../includes/exception.h" 

#include<limits>

Exploration::Exploration(const std::shared_ptr<Sensor> sensor, const std::shared_ptr<Frontier> frontier, const std::shared_ptr<Robot> robot) : sensor(sensor), frontier(frontier), robot(robot)  
                                                                                                                                               /*************************************************
                                                                                                                                                * See speficiation in the header 
                                                                                                                                                *************************************************/
{
  /* code */
}

struct Tabel 
{
  vec2 position;
  double heading;
  double reward;
  double cost;
  double total;
};

geometry_msgs::Pose Exploration::explore(const Map & map)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  // Cleaning up
  sensor_vising.clear();  


  size_t frontiers_size = frontier->search(map); 

  if(frontiers_size == 0 ) 
    throw NoFrontier(); 

  size_t cluster_number = frontier->clustering(5); 

  if(cluster_number == 0) 
    throw NoFrontierCluster(); 

  frontier->calcCenter(map); 

  std::vector<vec2> cluster_centers = frontier->getClusterCenter(); 

  std::vector<Tabel> info; 
  for(vec2 center : cluster_centers) 
  {
    SensorReading reading;
    if(frontier->isUnknown(map.getCellData(center))) 
    {
      map.translatePositionInToCell(robot->getLocalization(),center ); 
      frontier->addClusterCenter(center); 
    } 

    reading = sensor->calcNumberOfUnknownCellTotal(center, map, frontier.get() , 20); 

    try 
    { 
      reading = findOptimalSensorDirection(reading); 
    }
    catch(std::out_of_range ex) 
    {
      ROS_WARN("Exploration::explore, cluster center %s, error %s", center.print().c_str() , ex.what() );
      continue;
    } 

    Tabel t;
    t.position = center;
    t.heading = sensor->getForwardDirection(reading); 
    t.reward = reward(center, reading);
    t.cost   = cost(map.translateCellInToPositionVec(center));
    t.total  = -t.cost + t.reward;

    ROS_INFO("Cluster center %s, Reward %f, Cost %f, Total %f", center.print().c_str(), t.reward, t.cost, t.total);

    info.push_back(t); 

    reading.append(sensor_vising); 
  }

  /* calc next goal */

  double max = std::numeric_limits<double>::min(); 
  Tabel max_location;
  for( Tabel element : info) 
  {
    if(element.cost == 0)
    {

      geometry_msgs::Point p = map.translateCellInToPosition(element.position);
      ROS_INFO("Robot is move in to x,y,theta  (%f %f %f), with total %f, but no cost", p.x, p.y , element.heading, element.total) ;
      return robot->moveToPosition(p, element.heading); 
    }  

    if(element.total > max) 
    {
      max_location = element;
    } 
  }


  geometry_msgs::Point p = map.translateCellInToPosition(max_location.position);
  ROS_INFO("Robot is move in to x,y,theta  (%f %f %f), with total %f", p.x, p.y , max_location.heading, max_location.total) ;
  return robot->moveToPosition(p, max_location.heading); 

}



Exploration::~Exploration()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  /* code */
}

std::vector<geometry_msgs::Point> Exploration::getSensorVising() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return sensor_vising;
}

double Exploration::cost(const vec2 center) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return robot->distanceToPositionSquared(center); 
}

double Exploration::reward(const vec2 position, const SensorReading reading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return reading.total_count;
}


/***********************************************
 * The idee is that the optimal direction will *
 * be at the center of the disbution which can *
 * be found be find the largeste group of index*
 * above the mean .                            *
 ***********************************************/
SensorReading Exploration::findOptimalSensorDirection(const SensorReading & reading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  double mean = reading.count_of_angle_resolution.calcMean();

  std::vector<size_t> indexs = reading.count_of_angle_resolution.getIndexAbove(mean); 

  std::vector<std::vector<size_t>> group_indexs(10) ;
  size_t index1 = 0;
  size_t max    = 0;

  for(size_t i=0; i < indexs.size(); i++)
  {
    if (i == indexs.size() - 1 ) 
    {
      if(indexs[i] - indexs[i - 1 ] == 1) 
        group_indexs[index1].push_back(indexs[i]); 
    } 
    else
    {
      if(indexs[i + 1] - indexs[i] == 1) 
        group_indexs[index1].push_back(indexs[i]); 
      else if(group_indexs[index1].size() > 0 )
      {
        // Keep tracked of the largest group.
        if(group_indexs[max].size()  < group_indexs[index1].size() ) 
          max = index1;
        index1++;
      } 
    } 

    // If the index1 presices the already allocated memory
    if(group_indexs.size() == index1 ) 
      group_indexs.resize(index1); 
  }

  // The size of the largeste group
  size_t max_size = group_indexs[max].size();  
  size_t direction_index;

  if(max_size == 0) 
    throw std::out_of_range("The direction index is out of range (" + std::to_string(direction_index) + ")"); // TODO
  else if(max_size == 1) 
    direction_index = group_indexs[max][0];
  else
    direction_index = group_indexs[max][max_size/2];

  if(direction_index >= reading.count_of_angle_resolution.bins)
    throw std::out_of_range("The direction index is out of range (" + std::to_string(direction_index) + ")");


  return sensor->getSensorDataAroundIndexInReading(reading, direction_index);
}
