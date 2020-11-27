#include "drone_navigation/commander.h"
#include <ros/ros.h>

using namespace drone_navigation;

int main(int argv, char** argc){
	ros::init(argv, argc, "commander");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    Commander drone_commander(nh, nh_private);

    ros::spin();
    return 0;
}