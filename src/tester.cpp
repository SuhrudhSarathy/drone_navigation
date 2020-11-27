#include "drone_navigation/trajectory_optimiser.h"
#include "drone_navigation/voxblox_perception.h"
#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "tester_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    map_manager::MapManagerVoxblox mmp(nh, nh_private);
    topt::NonLinearOptimiser opt(nh, nh_private);
    ros::spin();
}