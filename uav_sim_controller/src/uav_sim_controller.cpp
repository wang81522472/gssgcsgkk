//
// Created by chengdaqian on 17-11-29.
//

#include <uav_sim_controller/uav_sim_controller.h>
#include <math.h>

namespace uav_sim_controller {
    UAVSimController::UAVSimController() :
            nh_(), private_nh_("~"), tf_listener_(ros::Duration(10.0))
    {
        uav_cmd_vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        goal_location_sub_ = private_nh_.subscribe("goal_location", 10, &UAVSimController::rcvGoalCb, this);
        uav_odom_sub_ = private_nh_.subscribe("odom", 10, &UAVSimController::rcvOdomCb, this);

        goal_flag_ = false;

        private_nh_.param<double>("kp", kp_, 0.5);
        private_nh_.param<double>("max_vel", max_vel_, 1.0);
        private_nh_.param<double>("tolerance", tolerance_, 0.2);
        private_nh_.param<double>("min_vel", min_vel_, 0.3);

        ROS_INFO_STREAM("Tolerance: " << tolerance_);
        ROS_INFO_STREAM("Min_vel" << min_vel_);
    }

    bool UAVSimController::sendCmdVelOnce(){
        if (!goal_flag_){
            ROS_INFO_ONCE("goal not set, unable to go.");
        }
        else {
            double dx, dy, dz, kp = kp_, abs_dx, abs_dy, abs_dz;

            dx = goal_location_.x - uav_odom_.pose.pose.position.x;
            dy = goal_location_.y - uav_odom_.pose.pose.position.y;
            dz = goal_location_.z - uav_odom_.pose.pose.position.z;
            abs_dx = fabs(dx);
            abs_dy = fabs(dy);
            abs_dz = fabs(dz);

            if (abs_dx * kp_ > max_vel_ || abs_dy * kp_ > max_vel_) {
                kp = (abs_dx > abs_dy) ? max_vel_ / abs_dx : max_vel_ / abs_dy;
            }

            if (abs_dx < tolerance_ && abs_dy < tolerance_){
                
                ROS_INFO_STREAM("Reached. abs_dx: " << abs_dx << ", abs_dy: " << abs_dy );
                goal_flag_ = false;
            }

            uav_cmd_vel_.linear.x = dx * kp;
            uav_cmd_vel_.linear.y = dy * kp;

            //uav_cmd_vel_.linear.z = dz * kp;

            double speed = sqrt(pow(uav_cmd_vel_.linear.x, 2.0) + pow(uav_cmd_vel_.linear.y, 2.0));
            if (speed < min_vel_){
                uav_cmd_vel_.linear.x *= min_vel_ / speed;
                uav_cmd_vel_.linear.y *= min_vel_ / speed;
            }
            uav_cmd_vel_pub_.publish(uav_cmd_vel_);
        }
    }

    void UAVSimController::transformPoint(geometry_msgs::PointStamped& point){
        point.header.frame_id = uav_odom_.header.frame_id;
        //point.point.x -= 1.0;
        //point.point.y -= 1.0;
    }
    void UAVSimController::rcvGoalCb(const geometry_msgs::PointStamped& goal_location){
        if (uav_odom_.header.frame_id.empty()){
            ROS_INFO_ONCE("uav's odom not received.");
        }else {

            geometry_msgs::PointStamped temp = goal_location;
            /*
            if (temp.header.frame_id != uav_odom_.header.frame_id) {
                tf_listener_.transformPoint(uav_odom_.header.frame_id, goal_location, temp);
            }
            */
            transformPoint(temp);

            goal_location_ = temp.point;
            goal_flag_ = true;
            ROS_INFO_STREAM_ONCE("goal set at x:" << goal_location_.x << " y:" << goal_location_.y
                                                  << " z:" << goal_location_.z);
        }
    }

    void UAVSimController::rcvOdomCb(const nav_msgs::Odometry& uav_odom){
        uav_odom_ = uav_odom;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_sim_controller");

    uav_sim_controller::UAVSimController controller;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        controller.sendCmdVelOnce();
        loop_rate.sleep();
    }
}
