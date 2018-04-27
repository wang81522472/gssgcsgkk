#include <ros/ros.h>
#include <frontier_exploration/MultiArrayWithHeader.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

namespace grad_field_ctrl
{
	class GradFieldCtrl{
    public:
        GradFieldCtrl() : nh_(), private_nh_("~")
        {
            odom_sub_ = nh_.subscribe("odom", 5, &GradFieldCtrl::odom_cb, this);
            grad_sub_ = private_nh_.subscribe("grad", 5, &GradFieldCtrl::grad_cb, this);
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 3);
            private_nh_.param<double>("tf_offset_x", tf_offset_x_, 0.0);
            private_nh_.param<double>("tf_offset_y", tf_offset_y_, 0.0);
            private_nh_.param<double>("ori_x", ori_x_, -5.0);
            private_nh_.param<double>("ori_y", ori_y_, -5.0);
            private_nh_.param<double>("resolution", resolution_, 0.1);
            private_nh_.param<double>("speed", speed_, 0.5);
        }

    private:
        ros::NodeHandle nh_, private_nh_;
        ros::Subscriber odom_sub_, grad_sub_;
        ros::Publisher cmd_vel_pub_;

        geometry_msgs::Point loc_; // all in map frame
        geometry_msgs::Twist cmd_vel_;

        double tf_offset_x_, tf_offset_y_, ori_x_, ori_y_, resolution_, speed_;
        unsigned int size_x_;
        std::vector<double> grad_;

        void odom_cb(const nav_msgs::Odometry& odom){
            loc_.x = odom.pose.pose.position.x - tf_offset_x_;
            loc_.y = odom.pose.pose.position.y - tf_offset_y_;

            unsigned int mx, my, idx;
            mx = (int)((loc_.x - ori_x_) / resolution_);
            my = (int)((loc_.y - ori_y_) / resolution_);
            idx = my * size_x_ + mx;

            if (!grad_.empty()){
                cmd_vel_.linear.x = -speed_ * cos(grad_[idx]);
                cmd_vel_.linear.y = -speed_ * sin(grad_[idx]);
                cmd_vel_pub_.publish(cmd_vel_);
            }

        }

        void grad_cb(const frontier_exploration::MultiArrayWithHeader& gradient){
            //ROS_INFO_STREAM("Rec grad: " << gradient.array.layout.data_offset);
            grad_.clear();
            for (auto it = gradient.array.data.begin(); it != gradient.array.data.end(); it++){
                grad_.push_back(*it);
            }
            size_x_ = gradient.array.layout.data_offset;
        }

    };
}

int main(int argc, char** argv){

	ros::init(argc, argv, "grad_field_ctrl");

    grad_field_ctrl::GradFieldCtrl controller;

    ros::spin();
}
