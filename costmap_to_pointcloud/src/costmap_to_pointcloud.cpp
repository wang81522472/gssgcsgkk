//
// Created by chengdaqian on 18-2-17.
//

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace costmap_to_pointcloud
{
    class CostmapToPointCloud{

    public:
        CostmapToPointCloud() : nh_("~")
        {
            costmap_sub_ = nh_.subscribe("costmap_topic", 1, &CostmapToPointCloud::costmap_cb, this);
            pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("costmap_pointcloud", 1);
            nh_.param("obstacle_height", obstacle_height_, 1.0);
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher  pointcloud_pub_;
        ros::Subscriber costmap_sub_;

        double obstacle_height_;

        void costmap_cb(const nav_msgs::OccupancyGrid &costmap_rec){
            ROS_INFO("Received Map!");

            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            pcl::PointXYZ point_holder;

            unsigned int size = costmap_rec.info.height * costmap_rec.info.width, mx, my;
            float wx, wy;

            unsigned int debug_cnt = 0;

            for (unsigned int idx = 0; idx < size; idx++){
                mx = idx % costmap_rec.info.width;
                my = idx / costmap_rec.info.width;
                wx = float(mx * costmap_rec.info.resolution + costmap_rec.info.origin.position.x);
                wy = float(my * costmap_rec.info.resolution + costmap_rec.info.origin.position.y);

                debug_cnt++;

                point_holder.x = wx;
                point_holder.y = wy;

                if (costmap_rec.data[idx] == -2){
                    for (int tmp_idx = -1; tmp_idx < obstacle_height_ * 10; tmp_idx++){
                        point_holder.z = tmp_idx / 10.0;
                        point_cloud.push_back(point_holder);
                    }
                }

		/*
                if (costmap_rec.data[idx] == 0){
                    point_holder.z = -0.1;
                    point_cloud.push_back(point_holder);
                }
		*/
            }

            std::cout << "total cnt: " << debug_cnt << std::endl;

            sensor_msgs::PointCloud2 pointcloud_output;
            pcl::toROSMsg(point_cloud, pointcloud_output);
            pointcloud_output.header.frame_id = costmap_rec.header.frame_id;
            pointcloud_output.header.stamp = ros::Time::now();
            pointcloud_pub_.publish(pointcloud_output);

        }

    };
}

int main(int argc, char** argv){

    ros::init(argc, argv, "costmap_to_pointcloud");

    costmap_to_pointcloud::CostmapToPointCloud converter;

    ros::spin();

}
