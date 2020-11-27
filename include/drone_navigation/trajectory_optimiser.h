#ifndef TRAJECTORY_OPTIMISER_H
#define TRAJECTORY_OPTIMISER_H

#include<ros/ros.h>
#include<trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include<trajectory_msgs/MultiDOFJointTrajectory.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Pose.h>
#include<mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include<mav_trajectory_generation/polynomial_optimization_linear.h>
#include<mav_trajectory_generation_ros/ros_visualization.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>
#include<mav_trajectory_generation/trajectory.h>
#include<mav_trajectory_generation/trajectory_sampling.h>
#include<mav_msgs/conversions.h>
#include<mav_msgs/eigen_mav_msgs.h>
#include<eigen_conversions/eigen_msg.h>
#include<Eigen/Dense>

// TODO: Use map manager to validate the trajectory formed

namespace drone_navigation{
class NonLinearOptimiser{
    public:
        NonLinearOptimiser(const ros::NodeHandle&, const ros::NodeHandle&);
        ~NonLinearOptimiser();
        void publish_trajectory_markers(bool);
        void set_dimension(const int);
        void set_derivative(const int);
        void set_frame_id(std::string);
        void optimise_trajectory(const nav_msgs::Path&, trajectory_msgs::MultiDOFJointTrajectory&);
    private:
        // ros Nodehandles
        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        const float a_max = 2.0;
        const float v_max = 2.0;

        float vis_distance = 0.5;
        int dimension = 3;
        int derivative = 3;

        std::string frame_id;
        bool publish_traj_markers = true;

        ros::Publisher marker_pub;

};
}// drone_navigation
#endif