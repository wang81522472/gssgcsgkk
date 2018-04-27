#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

// Circle trajectory
nav_msgs::Path circle()
{
  // Circle parameters
  double r = 1.2;
  double h = 1.0;
  double l = 1.0;
  // Init msg
  nav_msgs::Path waypoints;
  geometry_msgs::PoseStamped pt;
  pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  // First loop    
  pt.pose.position.x =  r;
  pt.pose.position.y = -r;
  pt.pose.position.z =  h;
  //pt.pose.orientation = tf::createQuaternionMsgFromYaw(90*M_PI/180);      
  waypoints.poses.push_back(pt);      
  pt.pose.position.x =  r*2;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  //pt.pose.orientation = tf::createQuaternionMsgFromYaw(180*M_PI/180);        
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  r;
  pt.pose.position.y =  r;
  pt.pose.position.z =  h;
  //pt.pose.orientation = tf::createQuaternionMsgFromYaw(270*M_PI/180);          
  waypoints.poses.push_back(pt);   
  pt.pose.position.x =  0;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  //pt.pose.orientation = tf::createQuaternionMsgFromYaw(0*M_PI/180);              
  waypoints.poses.push_back(pt);  
  // Third loop        
  pt.pose.position.x =  r;
  pt.pose.position.y = -r;
  pt.pose.position.z =  h;
  //pt.pose.orientation = tf::createQuaternionMsgFromYaw(90*M_PI/180);        
  waypoints.poses.push_back(pt);      
  pt.pose.position.x =  r*2;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  //pt.pose.orientation = tf::createQuaternionMsgFromYaw(180*M_PI/180);          
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  r;
  pt.pose.position.y =  r;
  pt.pose.position.z =  h;
  //pt.pose.orientation = tf::createQuaternionMsgFromYaw(270*M_PI/180);            
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  0;
  pt.pose.position.y =  0;
  pt.pose.position.z =  l;
  //pt.pose.orientation = tf::createQuaternionMsgFromYaw(0*M_PI/180);              
  waypoints.poses.push_back(pt);     
  // Return
  return waypoints;
}
 

// Figure 8 trajectory
nav_msgs::Path eight()
{
  // Circle parameters
  double r = 0.9;
  double h = 1.0;
  double l = 1.0;
  // Init msg
  nav_msgs::Path waypoints;
  geometry_msgs::PoseStamped pt;
  pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);    
  // First loop
  pt.pose.position.x =  r;
  pt.pose.position.y = -r;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);      
  pt.pose.position.x =  r*2;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  r*3;
  pt.pose.position.y =  r;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  r*4;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);       
  pt.pose.position.x =  r*3;
  pt.pose.position.y = -r;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);      
  pt.pose.position.x =  r*2;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  r;
  pt.pose.position.y =  r;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  0;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);             
  // Second loop
  pt.pose.position.x =  r;
  pt.pose.position.y = -r;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);      
  pt.pose.position.x =  r*2;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  r*3;
  pt.pose.position.y =  r;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  r*4;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);       
  pt.pose.position.x =  r*3;
  pt.pose.position.y = -r;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);      
  pt.pose.position.x =  r*2;
  pt.pose.position.y =  0;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  r;
  pt.pose.position.y =  r;
  pt.pose.position.z =  h;
  waypoints.poses.push_back(pt);  
  pt.pose.position.x =  0;
  pt.pose.position.y =  0;
  pt.pose.position.z =  l;
  waypoints.poses.push_back(pt);      
  // Return
  return waypoints;   
}  

#endif
