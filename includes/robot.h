#ifndef ROBOT
#define ROBOT

#include "../includes/vec2.h" 

#include <memory>

#include <geometry_msgs/Pose.h> 
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


class Robot
{
  public:
    Robot();

    /****************************************************
     * Name: getLocalization                                                  
     * Description: get the current location of the robot
     * Parameters: 
     * Return: the current location of the robot
     * Throws: 
     * Errors: 
     ****************************************************/
    vec2 getLocalization() const;

    /****************************************************
     * Name: distanceToPosition                                                  
     * Description: Calculate the squared distance to transverse from current position to a new 
     * Parameters: 
*                   const vec2 position - TODO
     * Return: The distance squared
     * Throws: 
     * Errors: 
     ****************************************************/
    double distanceToPositionSquared(const vec2 position) const;

    /****************************************************
     * Name: getOdom                                                  
     * Description: Subscriber node for odem information 
     * Parameters: 
*                   nav_msgs::Odometry & odem - TODO
     * Return:  
     * Throws: 
     * Errors:
     ****************************************************/
    void getOdom(nav_msgs::Odometry odom);
    

    /****************************************************
     * Name: moveToPosition                                                  
     * Description: Calculate the point to move to
     * Parameters: 
     *              const geometry::Point p - TODO
*                   const double heading - TODO
     * Return: The Pose of the robot
     * Throws: 
     * Errors:
     ****************************************************/
    geometry_msgs::Pose moveToPosition(const geometry_msgs::Point p, const double heading);

    /****************************************************
     * Name: move                                                  
     * Description: 
     * Parameters: 
*                   const double x - TODO
*                   const double y - TODO
*                   const double orientation - TODO
     * Return:  
     * Throws: 
     * Errors: 
     ****************************************************/
    move_base_msgs::MoveBaseGoal move(const geometry_msgs::Pose pose);

    ~Robot();  

  private:
    vec2 current_position;
    double orientation;

};

#endif
