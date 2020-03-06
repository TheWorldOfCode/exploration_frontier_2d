#ifndef EXPLORATION_FRONTIER_2D_NAVIGATION_MOVE_BASE
#define EXPLORATION_FRONTIER_2D_NAVIGATION_MOVE_BASE

#include "../includes/navigation_base.h" 

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

namespace exploration_navigation
{
  class NavigationMoveBase : public NavigationBase 
  {
    public: 
      NavigationMoveBase(); 

      /****************************************************
       * Name: initialize                                                  
       * Description: Used to initialze the sensor
       * Parameters: 
       *                   std::shared_ptr<ros::NodeHandle> h - Node handler 
       * Return: 
       * Throws:
       * Errors:
       ****************************************************/
      void initialize(std::shared_ptr<ros::NodeHandle> h);

      /****************************************************
       * Name: move                                                  
       * Description: Move the robot to a global position 
       * Parameters: 
*                   const geometry_msgs:Pose position - Goal position
       * Return: 
       * Throws: 
       * Errors: 
       ****************************************************/
      void move(const geometry_msgs::Pose position);

      /****************************************************
       * Name: cancelAll
       * Description:  Cancel all goals 
       * Parameters: 
       * Return:  
       * Throws: 
       * Errors:
       ****************************************************/
      void cancelAll();

      /****************************************************
       * Name: isMoving                                                  
       * Description: Check if the robot is moving 
       * Parameters: 
       * Return:  Return true if the robot is moving 
       * Throws: ActionDisconnected 
       * Errors:  
       ****************************************************/
      bool isMoving();

      /****************************************************
       * Name: reachedGoal                                                  
       * Description: Check if the goal could be reached (robot is not moving) 
       * Parameters: 
       * Return:  Return true if goal could be reached
       * Throws:  Moving, ActionDisconnected
       * Errors:  
       ****************************************************/
      bool reachedGoal();
      /****************************************************
       * Name: makePlan                                                  
       * Description: Create a plan the robot can use to move from location to another
       * Parameters: 
*                   const geometry_msgs::Pose location - The location of the robot
*                   const geometry_msgs::Pose position - The location of the goal
       * Return: A list of position 
       * Throws: 
       * Errors:
       ****************************************************/
      std::vector<geometry_msgs::PoseStamped> makePlan(const geometry_msgs::Pose location, const geometry_msgs::Pose position);

      ~NavigationMoveBase(); 
    private:
      enum MoveAction {INITIALIZE, IS_MOVING, MOVE, CANCEL_ALL, HAS_REACHED_GOAL};
      /****************************************************
       * Name: action                                                  
       * Description: Control the action interface with the move_base node
       * Parameters: 
*                   const MoveAction ac - TODO
       * Return: 
       * Throws: 
       * Errors: 
       ****************************************************/
      void action(const MoveAction ac);

      std::string frame_id_global;
      bool moving;  // Contains the check result is the robot is moving 
      bool reached; // Contains the check result for if the robot reached the goal
      move_base_msgs::MoveBaseGoal goal; 

      // Used to generate plan
      ros::ServiceClient planner;
      float nav_tolerance;
  };
} 

#endif
