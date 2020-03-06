#include "../includes/robot.h" 

#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/ros.h>
#include <tf/tf.h>


Robot::Robot(boost::shared_ptr<exploration_navigation::NavigationBase> nav) : nav(nav) 
{
    /* code */
}  


vec2 Robot::getLocalization() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   return current_position;
}

geometry_msgs::Point Robot::convVecToPoint(const vec2 v) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  geometry_msgs::Point p;
  p.x = v.getX(); 
  p.y = v.getY(); 
  p.z = robot_pose.position.z;

  return p;
}

double Robot::distanceToPositionSquared(const vec2 position) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  return  (position - current_position).squaredLength();
}


void Robot::getOdom(nav_msgs::Odometry odom)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  if(odom.child_frame_id == "base_footprint")
  {
    current_position = vec2(odom.pose.pose.position.x, odom.pose.pose.position.x);
    orientation      = odom.pose.pose.orientation.z;
    robot_pose = odom.pose.pose;
  }  
}

std::vector<geometry_msgs::PoseStamped> Robot::getPlan(const vec2 goal, const double heading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{

  return nav->makePlan(robot_pose,  moveToPosition(convVecToPoint(goal) , heading)); 
}

geometry_msgs::Pose Robot::moveToPosition(const geometry_msgs::Point p, const double heading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  geometry_msgs::Pose goal; 
  goal.position = p;

  tf::Quaternion orig = tf::createQuaternionFromRPY(0,0,heading); 

  goal.orientation.w = orig.getW(); 
  goal.orientation.x = orig.getX(); 
  goal.orientation.y = orig.getY(); 
  goal.orientation.z = orig.getZ(); 

  return goal;
}



void Robot::move(const geometry_msgs::Pose pose, const Map * map, const std::shared_ptr<Frontier> frontier)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  std::vector<geometry_msgs::PoseStamped> path = nav->makePlan(robot_pose, pose); 

  if(path.size() == 0 ) { 
    ROS_INFO("Could not created a path to goal"); 
    return;
  }
  
  size_t step = path.size() /2  ; 


  if(step == 0) 
  { 
    ROS_INFO("Step size for navigation is 0 and are set to 1" );
    step = 1;
  }

  for(size_t i= step; i < path.size() ; i+= step)
  {
    int x,y; 
    map->translatePositionInToCell(path[i].pose.position, x,y);
    if(frontier->isOccupied(map->getCellData(x,y)))
    {
    
      ROS_INFO("Didn't reach the goal  ");
      return;
    } 

    nav->move(path[i].pose); 
    ros::spinOnce(); 
  } 

  ROS_INFO("Reached goal");


  ROS_INFO("Canceling all goals"); 
  nav->cancelAll(); 

}


Robot::~Robot()
{
  /* code */
}  
