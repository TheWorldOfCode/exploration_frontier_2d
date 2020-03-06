#ifndef ROBOT
#define ROBOT

#include "../includes/vec2.h" 

#include <memory>

#include <geometry_msgs/Pose.h> 
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "../includes/navigation_base.h" 
#include "../includes/map.h" 
#include "../includes/frontier.h" 





class Robot
{
  public:
    Robot(boost::shared_ptr<exploration_navigation::NavigationBase> nav);

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
     * Name: convVecToPoint                                                  
     * Description: Convert a 2D vector to a Point
     * Parameters: 
*                   const vec2 v - TODO
     * Return: A Point
     * Throws: 
     * Errors: 
     ****************************************************/
    geometry_msgs::Point convVecToPoint(const vec2 v) const;

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
     * Name: getPlan                                                  
     * Description: Create and return the plan to move to a goal
     * Parameters: 
*                   const vec2 goal - TODO
*                   const double heading - TODO
     * Return:  Return a list of poses that gives the path 
     * Throws:  
     * Errors:  
     ****************************************************/
    std::vector<geometry_msgs::PoseStamped> getPlan(const vec2 goal, const double heading) const;

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
    geometry_msgs::Pose moveToPosition(const geometry_msgs::Point p, const double heading) const;

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
     void move(const geometry_msgs::Pose pose, const Map * map, const std::shared_ptr<Frontier> frontier);

    ~Robot();  

  private:
    boost::shared_ptr<exploration_navigation::NavigationBase> nav;
    vec2 current_position;
    geometry_msgs::Pose robot_pose;
    double orientation;

};

#endif
