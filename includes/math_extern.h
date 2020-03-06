#ifndef EXPLORATION_FRONTIER_2D_MATH_EXTERN
#define EXPLORATION_FRONTIER_2D_MATH_EXTERN

#include <ros/ros.h>
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

} 

#endif
