#ifndef EXPLORATION_FRONTIER_2D_MATH_EXTERN
#define EXPLORATION_FRONTIER_2D_MATH_EXTERN

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include<cmath>
#include<vector>
#include<functional>

namespace math_extern
{

  template<typename R, typename T>
  R sum(const size_t begin, const size_t end, std::vector<T> data, std::function<R(T)> function    ) 
  {
    R result = 0;
    for(size_t i = begin; i < end; i++)
      result = result + function(data[i]); 

    return result;
  } 

  template<typename R, typename T>
  R sum(const size_t begin, const size_t end, const size_t begin_2, const size_t end_2, std::vector<T> data, std::function<R(T,T)> function    ) 
  {
    R result = 0;
    size_t j = begin_2; 
    for(size_t i = begin; i < end; i++)
    {
      result = result + function(data[i], data[j]); 
      j++;
    }

    return result;
  } 


  double manhatten_distance(geometry_msgs::PoseStamped p_i, geometry_msgs::PoseStamped p_j)
  {
    return std::abs(p_j.pose.position.x - p_i.pose.position.x) + std::abs((p_j.pose.position.y - p_i.pose.position.y)); 
  }
} 

#endif
