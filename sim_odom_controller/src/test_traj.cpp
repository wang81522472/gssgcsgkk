#include <iostream>
#include <trajectory_generator_waypoint.h>
#include <frontier_exploration/MultiArrayWithHeader.h>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;
using namespace std;


ros::Publisher coef_pub;

ros::Publisher traj_pub;

ros::Subscriber body_pose_sub;

ros::Subscriber trigger_sub;

visualization_msgs::Marker selected_marker_;

#define pt1x 1
#define pt1y 1

#define pt2x 2
#define pt2y -1

#define pt3x 3
#define pt3y 1

#define pt4x 4
#define pt4y -1

#define pt5x 5
#define pt5y 0

#define pt6x 4
#define pt6y 1

#define pt7x 3
#define pt7y -1

#define pt8x 2
#define pt8y 1

#define pt9x 1
#define pt9y -1

#define pt10x 0
#define pt10y 0

int cnt = -1;
int point_pass = 0;
double plan_time_start = 0;

Matrix<double, 10, 2> pts;


void traj_viz(Eigen::MatrixXd &traj_mat, double start_time){
	double des_x = 0, des_y = 0;
    geometry_msgs::Point parent, child;
    parent.z = child.z = 0;
    Eigen::MatrixXd coef=traj_mat.block(0,0,traj_mat.rows(),traj_mat.cols()-2);
    Eigen::VectorXd T    = traj_mat.col(coef.cols() +1);
    Eigen::MatrixXd coef_x,coef_y;
    
    selected_marker_.header.stamp = ros::Time::now();
    
	bool first=true;
            
	for(double dT = ros::Time::now().toSec();dT< T(T.rows()-1);dT+=0.1) {
        if(first){
            parent.x=coef(0,0);
            parent.y=coef(0,6);
                    
            parent.z = 2.0;
            selected_marker_.points.push_back(parent);
            first=false;
        }
                for (int i = 0; i < T.size(); i++) {
                    if (dT < T(i)) {
                        double tt = i > 0 ? dT - T(i - 1) : dT - start_time;

                        //std::cout<<"T(i-1): "<<T(i-1)<<'\n';

                        Eigen::Matrix<double, 1, 6> t_p;
                        t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);
                        Eigen::Matrix<double, 1, 6> t_v;
                        t_v << 0, 1, 2 * tt, 3 * pow(tt, 2), 4 * pow(tt, 3), 5 * pow(tt, 4);

                        Eigen::VectorXd coef_x;
                        Eigen::VectorXd coef_y;
                        coef_x = (coef.block(i, 0, 1, 6)).transpose();
                        coef_y = (coef.block(i, 6, 1, 6)).transpose();

                        des_x = t_p * coef_x;
                        des_y = t_p * coef_y;
                        child.x = des_x;
                        child.y = des_y;
                        parent.x=des_x;
                        parent.y=des_y;
                        
                        child.z = parent.z = 2.0;
                        
                        selected_marker_.points.push_back(child);
                        selected_marker_.points.push_back(parent);
                        break;
                    }
                }
        }

	    if (!selected_marker_.points.empty())
                selected_marker_.points.pop_back();
        


	selected_marker_.header.frame_id = "world";
    selected_marker_.action = visualization_msgs::Marker::ADD;
    selected_marker_.pose.orientation.w = 1.0;
    selected_marker_.id = 0;
    selected_marker_.type = visualization_msgs::Marker::LINE_LIST;
    selected_marker_.scale.x = 0.02;
    selected_marker_.color.b = selected_marker_.color.a = 1.0;
	traj_pub.publish(selected_marker_);
    selected_marker_.points.clear();

}

void body_pose_call_back(const nav_msgs::Odometry &msg){
    //std::cout<<"cnt:"<<cnt<<"\n";
	if(cnt < 0) return;
	
	if(cnt > 0){
		cnt++;
		//cnt=cnt%1600;

	}
	else{
        ros::Time time0=ros::Time::now();
        cnt++;
        if(point_pass>9) point_pass=9;
        Eigen::MatrixXd uav_coef;
        frontier_exploration::MultiArrayWithHeader uav_mat;

        plan_time_start = msg.header.stamp.toSec();
        TrajectoryGeneratorWaypoint T;
        Eigen::MatrixXd Path = Eigen::MatrixXd::Zero(11-point_pass,3);
        Eigen::MatrixXd Vel = Eigen::MatrixXd::Zero(2,3);
        Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(2,3);
        Eigen::VectorXd Time = Eigen::VectorXd::Zero(10-point_pass);
                    
        Path(0,0) = msg.pose.pose.position.x;
        Path(0,1) = msg.pose.pose.position.y;
        Path.block(1,0,10-point_pass,2)=pts.block(point_pass,0,10-point_pass,2);
                    
        Vel(0,0) = msg.twist.twist.linear.x;
        Vel(0,1) = msg.twist.twist.linear.y;
        Vel(1,0) = 0;//uav_pos(1);
        Vel(1,1) = 0;//uav_pos(3);

        Acc(0,0) = msg.twist.twist.angular.x;
        Acc(0,1) = msg.twist.twist.angular.y;

        
        for(int i=0; i<10-point_pass; i++)        
            Time(i) = 4; 
                    
        uav_coef = T.PolyQPGeneration(Path,Vel,Acc,Time);
        ros::Time time1_1=ros::Time::now();
        cout<<"time consume in solver : "<<(time1_1 - time0).toSec()<<endl;

        Eigen::MatrixXd temp=uav_coef;
        uav_coef.resize(temp.rows(),temp.cols()+2);
        uav_coef.block(0,0,temp.rows(),temp.cols())=temp;

        uav_coef(0,uav_coef.cols()-1)=Time(0)+plan_time_start;
        for(int i=1; i<10-point_pass; i++)
            uav_coef(i,uav_coef.cols()-1)=4+uav_coef(i-1,uav_coef.cols()-1);

        if (uav_mat.array.layout.dim.size() != 2)
            uav_mat.array.layout.dim.resize(2);
                
        uav_mat.array.layout.dim[0].stride = uav_coef.rows() * uav_coef.cols();
        uav_mat.array.layout.dim[0].size = uav_coef.rows();
        uav_mat.array.layout.dim[1].stride = uav_coef.cols();
        uav_mat.array.layout.dim[1].size = uav_coef.cols();
         
        if ((int)uav_mat.array.data.size() != uav_coef.size())
            uav_mat.array.data.resize(uav_coef.size());
        int ii = 0;
        for (int i = 0; i < uav_coef.rows(); ++i)
            for (int j = 0; j < uav_coef.cols(); ++j)
                uav_mat.array.data[ii++] = uav_coef.coeff(i, j);

        uav_mat.header.stamp=ros::Time(plan_time_start);
        coef_pub.publish(uav_mat);
        ros::Time time1=ros::Time::now();
        cout<<"time consume : "<<(time1 - time0).toSec()<<endl;

    traj_viz(uav_coef,plan_time_start);
    point_pass++;
    }

}

void traj_trigger_callback( const geometry_msgs::PoseStamped::ConstPtr& trigger_msg ){
           cnt=0; 
}


int main(int argc, char *argv[]){
	ros::init(argc, argv, "test_traj");
	ros::NodeHandle nh;
	body_pose_sub = nh.subscribe("/vins_estimator/imu_propagate", 10,body_pose_call_back);
	trigger_sub = nh.subscribe( "/traj_start_trigger", 100, traj_trigger_callback);
	coef_pub = nh.advertise<frontier_exploration::MultiArrayWithHeader>("/explore_server/explore_costmap/air_ground_explore/uav_mat", 1);

	traj_pub= nh.advertise<visualization_msgs::Marker>("traj_viz", 1);

	pts<< pt1x,pt1y,pt2x,pt2y,pt3x,pt3y,pt4x,pt4y,pt5x,pt5y,pt6x,pt6y,pt7x,pt7y,pt8x,pt8y,pt9x,pt9y,pt10x,pt10y;

	ros::spin();
	return 0;
}

