//
// Created by chengdaqian on 17-11-29.
//

#ifndef UAV_SIM_CONTROLLER_UAV_SIM_CONTROLLER_H
#define UAV_SIM_CONTROLLER_UAV_SIM_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

namespace uav_sim_controller
{
    class UAVSimController
    {
    public:
        UAVSimController();
        bool sendCmdVelOnce();

    private:
        geometry_msgs::Point goal_location_;
        nav_msgs::Odometry uav_odom_;
        geometry_msgs::Twist uav_cmd_vel_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        tf::TransformListener tf_listener_;
        ros::Publisher uav_cmd_vel_pub_;
        ros::Subscriber goal_location_sub_;
        ros::Subscriber uav_odom_sub_;

        double kp_, max_vel_, min_vel_;
        double tolerance_;

        bool goal_flag_;

        void rcvGoalCb(const geometry_msgs::PointStamped& goal_location);
        void rcvOdomCb(const nav_msgs::Odometry& uav_odom);
        void transformPoint(geometry_msgs::PointStamped& point);
    };
}

#endif //UAV_SIM_CONTROLLER_UAV_SIM_CONTROLLER_H
