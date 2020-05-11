#include "../includes/exploration.h" 
#include "../includes/exception.h" 
#include "../includes/global_definition.h" 
#include "../includes/math_extern.h" 

#include <ros/ros.h>

#include <limits>
#include <algorithm>

Exploration::Exploration(const boost::shared_ptr<exploration_sensor_model::SensorModelBase> sensor, const std::shared_ptr<Frontier> frontier, const std::shared_ptr<Robot> robot, const std::shared_ptr<ros::NodeHandle> h) : sensor(sensor), frontier(frontier), robot(robot)  
/*************************************************
* See speficiation in the header 
*************************************************/
{

  if(!h->getParam("cost_function_modifier", cost_modifier))
    cost_modifier = 1; 

  if(!h->getParam("cost_lambda", cost_lambda))
      cost_lambda = 1;

  if(!h->getParam("utility_function_modifier", utility_modifier ) ) 
    utility_modifier = 1;

  if(!h->getParam("goal_function_modifier", goal_modifier))
    goal_modifier = 0;

  if(!h->getParam("goal_wait", goal_wait))
    goal_wait = false;

  ROS_INFO("Param cost_function_modifier %f", cost_modifier);
  ROS_INFO("Param cost_lambda %f", cost_lambda);
  ROS_INFO("Param utility_function_modifier %f", utility_modifier);
  ROS_INFO("Param goal_function_modifier %f", goal_modifier);
  ROS_INFO("Param goal_wait %s", goal_wait ? "true" : "false");
}


geometry_msgs::Pose Exploration::explore(const Map & map)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{

  // Cleaning up
  sensor_vising.clear();  
  total_reading.clear(); 
  info.clear();

  size_t frontiers_size = frontier->search(map); 

  if(frontiers_size == 0 ) 
    throw NoFrontier(); 

  size_t cluster_number = frontier->clustering(exploration::g_minimum_cluster_size); 

  if(cluster_number == 0) 
    throw NoFrontierCluster(); 

  frontier->calcCenter(map); 

  std::vector<vec2> cluster_centers = frontier->getClusterCenter(); 

  if(goal_modifier != 0)
  {
    if(goal_wait && goal_list.size() == 0)
      throw NoGoal();

    if(goalAvailable(map))
    {
      ROS_INFO("Goal is reachable and now moving to the goal position");
      geometry_msgs::Pose p = goal_list.front();
      goal_list.pop_front(); 
      return p;
    }
  }

  size_t index = 0;
  double reward_max = 0, cost_max = 0, goal_max = 0;
  for(vec2 center : cluster_centers) 
  {
    Tabel t;


    if(frontier->isUnknown(map.getCellData(center))) 
    {
      center = frontier->findClusterFrontier(index); 
      frontier->addClusterCenter(center, map.translateCellInToPosition(center)); 
    } 

    t.position = center;

    if(cost_modifier != 0)
    {
      t.cost = cost(map.translateCellInToPositionVec(center), t.heading);

      if(t.cost > cost_max && t.cost < std::numeric_limits<double>::infinity())
        cost_max = t.cost;
    }
    else
      t.cost = 0;

    if(t.cost == std::numeric_limits<double>::infinity() )
      ROS_INFO("Cluster center %s, cost is infinity and the center is skipped", center.print().c_str());
    else
    {

      if(goal_modifier != 0)
      {
        t.goal = goalFunction(map.translateCellInToPositionVec(center));

        if(t.goal > goal_max)
          goal_max = t.goal;
      }
      else 
        t.goal = 0;

      if(t.goal == std::numeric_limits<double>::infinity())
        ROS_INFO("Goal reached"); 
      else 
      {
        double tmp = reward(center, map, t.heading);
        if(utility_modifier != 0)
        {
          t.reward = tmp;

          if(t.reward > reward_max)
            reward_max = t.reward;
        }
        else
          t.reward  = 0;



        if(t.reward > -1)
          info.push_back(t); 
      }
    }

    index++;
  }

  if(info.size() == 0)
    throw CostInf();

  for( Tabel & element : info) 
  {
    if(reward_max != 0)
      element.reward /= reward_max;
    else
      element.reward = 0;

    if(cost_max != 0)
      element.cost   /=   cost_max;
    else
      element.cost = 0;

    if( goal_max != 0)
      element.goal   /=   goal_max;
    else
      element.goal = 0;

    element.total = utility_modifier * element.reward - cost_modifier * element.cost + goal_modifier * element.goal;

    ROS_INFO("%f %f %f", reward_max, cost_max, goal_max);

    ROS_INFO("Cluster center %s, Reward %f, Cost %f, Goal %f, Total %f",
        element.position.print().c_str(),
        element.reward,
        element.cost,
        element.goal,
        element.total
        );
  }


  /* Sort the table */
  std::sort(info.begin(), info.end(), [](Tabel a, Tabel b) { return a.total > b.total; });

  index = 0;
  while(index < info.size())
  {
    size_t i = index;
    for( Tabel last : center_goal_list) 
    {
      if(last.position == info[index].position)
      {
        index++;
        break;
      }
    }

    if(i == index)
      break;
  }

  center_goal = info[index];
  geometry_msgs::Point p = map.translateCellInToPosition(info[index].position);

  ROS_INFO("Selected cluster, with center %s, Reward %f, Cost %f, Goal %f, Total %f", info[index].position.print().c_str(), info[index].reward, info[index].cost, info[index].goal, info[index].total);
  ROS_INFO("Robot is move in to x,y,theta  (%f %f %f), with total %f", p.x, p.y , info[index].heading, info[index].total) ;

  return robot->moveToPosition(p, info[index].heading * M_PI/((double) 180)); 

}

void Exploration::explore_debug(const Map & map)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  // Cleaning up
  sensor_vising.clear();  
  total_reading.clear(); 


  size_t frontiers_size = frontier->search(map); 

  if(frontiers_size == 0 ) 
    throw NoFrontier(); 

  size_t cluster_number = frontier->clustering(exploration::g_minimum_cluster_size); 

  if(cluster_number == 0) 
    throw NoFrontierCluster(); 

  frontier->calcCenter(map); 

  std::vector<vec2> cluster_centers = frontier->getClusterCenter(); 


  std::vector<Tabel> info; 
  size_t index = 0;
  for(vec2 center : cluster_centers) 
  {
    exploration_sensor_model::SensorReading reading;

    if(frontier->isUnknown(map.getCellData(center))) 
    {
      center = frontier->findClusterFrontier(index); 
      frontier->addClusterCenter(center, map.translateCellInToPosition(center)); 
    } 


    reading = sensor->calcNumberOfUnknownCellTotal(center, map, frontier.get() , 20); 
    total_reading.push_back(reading); 

    try { 
      reading = findOptimalSensorDirection(reading); 
    }
    catch(std::out_of_range ex) 
    {
      ROS_WARN("Exploration::explore, cluster center %s, error %s", center.print().c_str() , ex.what() );
      continue;
    } 

    reading.append(sensor_vising); 


    index++;
  }

}


void Exploration::getPoseArray(geometry_msgs::PoseArray ar )
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  if(ar.header.frame_id == "map")
  {
    ROS_INFO("New goal list received (size %i)", (int) ar.poses.size());
    std::vector<geometry_msgs::Pose> p = ar.poses;
    goal_list = std::list<geometry_msgs::Pose>(p.begin(), p.end());
  }
  else
    ROS_WARN("Goal list frame_id must be map");
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

std::vector<exploration_sensor_model::SensorReading> Exploration::getSensorReading( ) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return total_reading;
}

void Exploration::goalReached() 
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  center_goal_list.push_front(center_goal);

  if(center_goal_list.size() > 2)
    center_goal_list.pop_back();
}

double Exploration::cost(const vec2 center, const double heading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  double distance;
  std::vector<geometry_msgs::PoseStamped> p = robot->getPlan(center, heading); 

  if(p.size() != 0 ) 
  {
    distance = math_extern::sum<double, geometry_msgs::PoseStamped>(0, p.size() - 1, 1, p.size() , p, math_extern::manhatten_distance);

    distance = std::pow(distance, cost_lambda);
  }
  else
    distance = std::numeric_limits<double>::infinity(); 

  return distance;
}

double Exploration::reward(const vec2 position, const Map & map, double & heading)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  exploration_sensor_model::SensorReading reading = sensor->calcNumberOfUnknownCellTotal(position, map, frontier.get() , 20); 
  total_reading.push_back(reading); 

  try 
  { 
    reading = findOptimalSensorDirection(reading); 
    heading  = sensor->getForwardDirection(reading);
  }
  catch(std::out_of_range ex) 
  {
    ROS_WARN("Exploration::explore, cluster center %s, error %s", position.print().c_str() , ex.what() );
    return -1;
  } 

  reading.append(sensor_vising); 

  return reading.total_count;
}

double Exploration::reward(const vec2 position, const exploration_sensor_model::SensorReading reading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return reading.total_count;
}

double Exploration::goalFunction(const vec2 center) 
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  if(goal_list.size() == 0)
    return 0;

  geometry_msgs::Pose p = goal_list.front();
  vec2 goal(p.position.x, p.position.y);

  double distance = (goal - center).manhattanDistance();

  /*
   * Handle the distance increases insteed of decreases. 
   */

  if(distance == 0)
    return std::numeric_limits<double>::infinity();

  return std::exp(-distance);
}


bool Exploration::goalAvailable(const Map & map)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{

  vec2 goal;
  map.translatePositionInToCell(vec2(goal_list.front().position.x, goal_list.front().position.y), goal);
  if(goal_list.size() == 0)
    return false;

  if(!frontier->isFree(map.getCellData(goal)))
    return false;

  /*
     if(goal_list.size() != 1)
     {
     goal_list.pop_front();
     return true;
     }

*/
  //  goal_list.pop_front();

  return goal_wait;
}

/***********************************************
 * The idee is that the optimal direction will *
 * be at the center of the disbution which can *
 * be found be find the largeste group of index*
 * above the mean .                            *
 ***********************************************/
exploration_sensor_model::SensorReading Exploration::findOptimalSensorDirection(const exploration_sensor_model::SensorReading & reading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  double mean = reading.count_of_angle_resolution.calcMean();

  std::vector<size_t> indexs = reading.count_of_angle_resolution.getIndexAbove(mean); 

  if(mean < 1) 
    ROS_WARN("The mean of the current reading is below 1 (%f)", mean );

  std::vector<std::vector<size_t>> group_indexs(1);
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
      { 
        group_indexs[index1].push_back(indexs[i]); 
      }
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
      group_indexs.resize(index1 + 1); 

  }

  // Checking the last index 
  if(group_indexs[max].size()  < group_indexs[index1].size() ) 
    max = index1;

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


