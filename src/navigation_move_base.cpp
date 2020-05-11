
#include "../includes/navigation_move_base.h" 
#include <move_base/move_base.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

exploration_navigation::NavigationMoveBase::NavigationMoveBase()
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
   /* code */
}

void exploration_navigation::NavigationMoveBase::initialize(std::shared_ptr<ros::NodeHandle> h)
/*************************************************
 * See speficiation in the header 
 *************************************************/
{
  if(!h->getParam("move_base_global", frame_id_global ) )  
    frame_id_global = "map"; 

  ROS_INFO("Param move_base_global %s", frame_id_global.c_str());

  ros::NodeHandle n;

  planner = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan"); 

  nav_tolerance = 0.1;
  

  action(INITIALIZE); 
}

void exploration_navigation::NavigationMoveBase::move(const geometry_msgs::Pose position)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{

  goal.target_pose.header.frame_id = frame_id_global; 
  goal.target_pose.header.stamp = ros::Time::now(); 

  goal.target_pose.pose.position = position.position;
  goal.target_pose.pose.orientation = position.orientation;
  ROS_INFO("move_base goal position (%f %f %f) and orientation (%f %f %f %f)", position.position.x, position.position.y, position.position.z, position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w);
  action(MOVE);
}

void exploration_navigation::NavigationMoveBase::cancelAll() 
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
   action(CANCEL_ALL); 
}

bool exploration_navigation::NavigationMoveBase::isMoving()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  action(IS_MOVING);

  return moving;
}

bool exploration_navigation::NavigationMoveBase::reachedGoal()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  action(HAS_REACHED_GOAL);
  return reached;
}

std::vector<geometry_msgs::PoseStamped> exploration_navigation::NavigationMoveBase::makePlan(const geometry_msgs::Pose location, const geometry_msgs::Pose position)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  static uint32_t seq = 0;
  geometry_msgs::PoseStamped loc, goal;
  std_msgs::Header header;

  header.seq      = seq++;
  header.frame_id = frame_id_global;
  header.stamp    = ros::Time::now();

  loc.header = header;
  loc.pose   = location;

  goal.header = header;
  goal.pose   = position;
  

  nav_msgs::GetPlan plan;

  plan.request.start = loc;
  plan.request.goal = goal;
  plan.request.tolerance = nav_tolerance;

  std::vector<geometry_msgs::PoseStamped> result;
  if(planner.call(plan)) 
    result = plan.response.plan.poses;
  else
    ROS_WARN("The service call returned false"); 
 

  return result;
}

exploration_navigation::NavigationMoveBase::~NavigationMoveBase()
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  /* code */
}


void exploration_navigation::NavigationMoveBase::action(const MoveAction ac)
  /*************************************************
   * See speficiation in the header 
   *************************************************/
{
  static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>actionInterface("move_base", true); 


  switch(ac)
  {
    case INITIALIZE:
      while(!actionInterface.waitForServer(ros::Duration(5.0) ) )
      {
        ROS_INFO("Waiting for the move_base action server to come up" );
      } 
      return;
    case IS_MOVING:
      moving = !actionInterface.getState().isDone();  
      return;
    case MOVE:
      actionInterface.sendGoal(goal); 
      return;
    case CANCEL_ALL:
      actionInterface.cancelAllGoals(); 
      return;
    case HAS_REACHED_GOAL:
      reached = actionInterface.getState() == actionlib::SimpleClientGoalState::SUCCEEDED; 
      return;

  }  
}
