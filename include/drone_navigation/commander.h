#ifndef COMMANDER_H
#define COMMANDER_H

#include "drone_navigation/rrt.h"
#include "drone_navigation/trajectory_optimiser.h"
#include "drone_navigation/voxblox_perception.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Dense>

namespace drone_navigation{
class Commander{
    /*Main commander class for the drone*/
    public:
        Commander(const ros::NodeHandle&, const ros::NodeHandle&);
        ~Commander();
        void odom_callback(const nav_msgs::OdometryConstPtr& msg);
        void goal_callback(const geometry_msgs::PointConstPtr& msg);
        void publish_trajectory();
        void publish_path();
        void monitor_collision();
        void plan_path();
        void optimise_trajectory();
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_private;
        ros::Subscriber odom_sub;
        ros::Subscriber goal_sub;
        ros::Publisher path_pub;
        ros::Publisher trajectory_pub;


        geometry_msgs::Point start;
        geometry_msgs::Point goal;
        geometry_msgs::Point position;
        geometry_msgs::Quaternion orientation;
        nav_msgs::Path path;
        trajectory_msgs::MultiDOFJointTrajectory trajectory;

        MapManagerVoxblox map_manager;
        NonLinearOptimiser optimiser;
        RRT planner;

        bool goal_recieved = false;
        
};
}// namespace drone_navigation
#endif