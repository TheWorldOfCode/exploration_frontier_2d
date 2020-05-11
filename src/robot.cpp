#include "../includes/robot.h" 

#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/ros.h>


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

    tf::StampedTransform transform;
    
    try{
      listener.lookupTransform("/map", "/base_footprint",
          ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    current_position = vec2(transform.getOrigin().getX(), transform.getOrigin().getY());

    tf::Matrix3x3 q(transform.getRotation());
    double roll, pitch;
    q.getRPY(roll, pitch, orientation);

    robot_pose.position.x = current_position.getX();
    robot_pose.position.y = current_position.getY();
    robot_pose.position.z = 0;
    robot_pose.orientation.x = transform.getRotation().getX();
    robot_pose.orientation.y = transform.getRotation().getY();
    robot_pose.orientation.z = transform.getRotation().getZ();
    robot_pose.orientation.w = transform.getRotation().getW();

//    vec2 tmp(odom.pose.pose.position.x, odom.pose.pose.position.y);

//    if((tmp - current_position).length() > 2)
 //     ROS_WARN("The odometry(%s) and position estimate (%s) is of by over 2 meter %f ",tmp.print().c_str(),current_position.print().c_str(),(tmp - current_position).length());
    /*
    tf::Quaternion orien(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    orientation = orien.getAngle();
    robot_pose = odom.pose.pose;
    */

//    ROS_INFO("Position %s Orientation %f", current_position.print().c_str(), orientation);
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

geometry_msgs::Pose Robot::moveRelative(const geometry_msgs::Point p, const double heading) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{

  geometry_msgs::Pose goal; 
  goal.position.x = p.x + robot_pose.position.x;
  goal.position.y = p.y + robot_pose.position.y;
  goal.position.z = p.z + robot_pose.position.z;

  tf::Quaternion orig = tf::createQuaternionFromRPY(0,0,heading); 
  tf::Quaternion orig_robot(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w) ;

  orig = orig + orig_robot;
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

  nav->move(pose); 
  nav_pos = pose;

  /*
  if(nav->reachedGoal()) 
  { 
    ROS_INFO("Reached goal");
    last_good_location = pose;
  }
  else
    ROS_INFO("Didn't reach the goal  ");

  nav->cancelAll();

    */
  /*
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

     for(size_t i=path.size() / 2 ; i < path.size() ; i+= step)
     {
     int x,y; 
     map->translatePositionInToCell(path[i].pose.position, x,y);
     if(frontier->isOccupied(map->getCellData(x,y)))
     {

     ROS_INFO("Didn't reach the goal  ");
     nav->cancelAll();
     return;
     } 

     nav->move(path[i].pose); 
     ros::spinOnce(); 
     } 

     ROS_INFO("Reached goal");


     ROS_INFO("Canceling all goals"); 
     nav->cancelAll(); 
     */

}

void Robot::cancelGoal() const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  nav->cancelAll();
}


bool Robot::isMoving( ) const
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   return nav->isMoving();
}

bool Robot::reachedGoal() 
/*************************************************
 * See speficiation in the header 
 *************************************************/
{

  if(nav->reachedGoal()) 
  { 
    ROS_INFO("Reached goal");
    last_good_location = nav_pos;
    nav->cancelAll();
    return true;
  }
  else
    ROS_INFO("Didn't reach the goal  ");

  return false;
}

Robot::~Robot()
{
  /* code */
}  
