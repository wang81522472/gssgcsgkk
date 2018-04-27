#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace costmap_to_gridmap{

    using namespace grid_map;

    class Costmap2Gridmap{

    public:
        Costmap2Gridmap() : nh_("~")
        {
            costmap_sub_ = nh_.subscribe("costmap_topic", 1, &Costmap2Gridmap::costmap_cb, this);
            grid_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("grid_map",1);
        }


    private:
        GridMapRosConverter converter_;
        GridMap grid_map_;

        ros::Publisher  grid_map_pub_;
        ros::Subscriber costmap_sub_;
        ros::NodeHandle nh_;

        void costmap_cb(const nav_msgs::OccupancyGrid grid_rec){
            converter_.fromOccupancyGrid(grid_rec, "costmap", grid_map_);
            converter_.toPointCloud()
        }


    };

}

int main(int argc, char** argv){

    ros::init(argc, argv, "costmap_to_gridmap");

    costmap_to_gridmap::Costmap2Gridmap converter_obj;

    ros::spin();

}