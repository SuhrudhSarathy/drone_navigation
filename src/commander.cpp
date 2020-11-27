#include "drone_navigation/commander.h"
#include <ros/ros.h>
#include <Eigen/Dense>

namespace drone_navigation{
Commander::Commander(const ros::NodeHandle& nh_, const ros::NodeHandle& nh_private_) : nh(nh_), nh_private(nh_private_), map_manager(nh_, nh_private), optimiser(nh_, nh_private_), planner(nh_, nh_private_){
    path_pub = nh.advertise<nav_msgs::Path>("/path", 1);
    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory", 1);
    goal_sub = nh.subscribe("/goal_for_drone",1 ,&Commander::goal_callback, this);
    odom_sub = nh.subscribe("/odometry",1 ,&Commander::odom_callback, this);
    ROS_INFO("Commander Initiated");
}
Commander::~Commander(){}

void Commander::odom_callback(const nav_msgs::OdometryConstPtr& msg){
    position = msg->pose.pose.position;
    orientation = msg->pose.pose.orientation;
    monitor_collision();
}

void Commander::goal_callback(const geometry_msgs::PointConstPtr& msg){
    goal.x = msg->x;
    goal.y = msg->y;
    goal.z = msg->z;

    goal_recieved = true;
}

void Commander::publish_path(){
    path_pub.publish(path);
}

void Commander::publish_trajectory(){
    trajectory_pub.publish(trajectory);
}

void Commander::monitor_collision(){
    // Check collision status of the current position

    const Eigen::Vector3d point(position.x, position.y, position.z);
    if(abs(map_manager.get_distance_at_point(point)) < 1.0){
        // replan from the current location 
        plan_path();
        optimise_trajectory();
        publish_path();
        publish_trajectory();
    }
}

void Commander::plan_path(){
    // plan path from the current location to the goal obtained
    // TODO: RRT gives out the crude part now.
    // 1. Develop utility functions to do plain LOS Optimiser,
    // 2. Include yaw in planning and optimisation.
    if(goal_recieved){
        Node start(position.x, position.y, position.z);
        Node goal(goal.x, goal.y, goal.z);

        planner.call(start, goal, path);
    }
    else{
        ROS_WARN("Goal not recieved yet planner called");
    }
}

void Commander::optimise_trajectory(){
    // optimise the whole path for now
    // TODO: select n waypoints to optimise
    optimiser.optimise_trajectory(path, trajectory);
}
} // namespace drone_navigation
