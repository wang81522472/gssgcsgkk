#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "sample_waypoints.h"

using namespace std;

ros::Publisher pub1;
ros::Publisher pub2;
string waypoint_type = string("eight");
bool is_odom_ready = true;
nav_msgs::Odometry odom;
nav_msgs::Path waypoints;

void publish_waypoints()
{
  waypoints.header.frame_id = std::string("map");
  waypoints.header.stamp    = ros::Time::now();  
  pub1.publish(waypoints);
  geometry_msgs::PoseStamped init_pose;
  init_pose.header = odom.header;
  init_pose.pose   = odom.pose.pose;
  waypoints.poses.insert(waypoints.poses.begin(), init_pose);
  pub2.publish(waypoints);
  waypoints.poses.clear();   
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  is_odom_ready = true;
  odom = *msg;  
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_WARN("[WAYPOINTS] %s", waypoint_type.c_str());
  if (!is_odom_ready)
    return;
  if (waypoint_type == string("circle"))
  {
    waypoints = circle();
    publish_waypoints();
  }
  else if (waypoint_type == string("eight"))
  {
    waypoints = eight();  
    for (size_t idx = 0; idx < waypoints.poses.size(); ++ idx) 
    {
        waypoints.poses[idx].pose.position.x += 0.1;
        waypoints.poses[idx].pose.position.y += 0.1;
    }
    publish_waypoints();    
  }
  else
  {
    if (msg->pose.position.z > 0)
    {
      geometry_msgs::PoseStamped pt = *msg;
      if (waypoint_type == string("noyaw"))
      {
        double yaw = tf::getYaw(odom.pose.pose.orientation);
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);  
      }
      waypoints.poses.push_back(pt);
    }
    else if (waypoints.poses.size() >= 1)
    {
      publish_waypoints();    
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_generator");
  ros::NodeHandle n("~");
  n.param("waypoint_type", waypoint_type, string("eight"));  
  ros::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);      
  ros::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);    
  pub1 = n.advertise<nav_msgs::Path>("waypoints", 10);
  pub2 = n.advertise<nav_msgs::Path>("waypoints_vis", 10);  
  ros::spin();
  return 0;
}
