#ifndef EXPLORATION
#define EXPLORATION

#include <geometry_msgs/Point.h>

class exploration 
{

  public:
    exploration();

    double cost_function(geometry_msgs::Point end); 


    ~exploration();  

  private:

};

#endif
