#ifndef EXPLORATION_FRONTIER_2D_NAVIGATION_BASE
#define EXPLORATION_FRONTIER_2D_NAVIGATION_BASE

#include <ros/ros.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <exception>

#include "../includes/vec2.h" 

namespace exploration_navigation
{
  
  struct Moving : public std::exception
  {
    virtual const char * what() const throw()  
    {
       return "Robot is still moving";
    } 
  };

  struct ActionDisconnected : public std::exception
  {
    virtual const char * what() const throw()  
    {
       return "Can't connect to server";
    } 
  };

  class NavigationBase
  {
    public:
      /****************************************************
       * Name: initialize                                                  
       * Description: Used to initialze the sensor
       * Parameters: 
       *                   std::shared_ptr<ros::NodeHandle> h - Node handler 
       * Return: 
       * Throws:
       * Errors:
       ****************************************************/
      virtual void initialize(std::shared_ptr<ros::NodeHandle> h) = 0;

      /****************************************************
       * Name: move                                                  
       * Description: Move the robot to a global position 
       * Parameters: 
*                   const geometry_msgs::Pose position - TODO
       * Return: 
       * Throws: 
       * Errors: 
       ****************************************************/
      virtual void move(const geometry_msgs::Pose position) = 0;

      /****************************************************
       * Name: cancelAll
       * Description:  Cancel all goals 
       * Parameters: 
       * Return:  
       * Throws: 
       * Errors:
       ****************************************************/
      virtual void cancelAll() = 0;

      /****************************************************
       * Name: isMoving                                                  
       * Description: Check if the robot is moving 
       * Parameters: 
       * Return:  Return true if the robot is moving 
       * Throws:  
       * Errors:  
       ****************************************************/
      virtual bool isMoving() = 0;

      /****************************************************
       * Name: reachedGoal                                                  
       * Description: Check if the goal could be reached (robot is not moving) 
       * Parameters: 
       * Return:  Return true if goal could be reached
       * Throws:  Moving
       * Errors:  
       ****************************************************/
      virtual bool reachedGoal() = 0;

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
      virtual std::vector<geometry_msgs::PoseStamped> makePlan(const geometry_msgs::Pose location, const geometry_msgs::Pose position) = 0;

    protected:
      NavigationBase() {}  
  
  };

} 

#endif
