#include "../includes/robot.h" 

#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/ros.h>
#include <tf/tf.h>


Robot::Robot()
{
}  


vec2 Robot::getLocalization() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   return current_position;
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
  }  
}


geometry_msgs::Pose Robot::moveToPosition(const geometry_msgs::Point p, const double heading)
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



move_base_msgs::MoveBaseGoal Robot::move(const geometry_msgs::Pose pose)
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  move_base_msgs::MoveBaseGoal goal; 

  goal.target_pose.header.frame_id = "map"; 
  goal.target_pose.header.stamp = ros::Time::now(); 

  goal.target_pose.pose.position = pose.position;
  goal.target_pose.pose.orientation = pose.orientation;

  return goal;
}


Robot::~Robot()
{
  /* code */
}  
