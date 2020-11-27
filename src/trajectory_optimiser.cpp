#include "drone_navigation/trajectory_optimiser.h"

namespace drone_navigation{
NonLinearOptimiser::NonLinearOptimiser(const ros::NodeHandle& nh_, const ros::NodeHandle& nh_private_) : nh(nh_), nh_private(nh_private_){
	marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);
}
NonLinearOptimiser::~NonLinearOptimiser(){}

void NonLinearOptimiser::set_dimension(const int dimension_){
	dimension = dimension_;
}
void NonLinearOptimiser::set_derivative(const int derivative_){
	derivative = derivative_;
}
void NonLinearOptimiser::publish_trajectory_markers(bool des_){
	publish_traj_markers = des_;
}
void NonLinearOptimiser::set_frame_id(std::string frame_id_){
	frame_id = frame_id_;
}
void NonLinearOptimiser::optimise_trajectory(const nav_msgs::Path& path, trajectory_msgs::MultiDOFJointTrajectory& trajectory)
{	const int derv = derivative;
	
	// vertices to use for trajectory optimisation;
	mav_trajectory_generation::Vertex::Vector vertices;

	for(int i = 0; i < path.poses.size(); i++){
		Eigen::Affine3d pose;
		tf::poseMsgToEigen(path.poses[i].pose, pose);
		mav_trajectory_generation::Vertex vert(dimension);
		if(i == 0 || i == path.poses.size() - 1){
			vert.makeStartOrEnd(pose.translation(), derv);
			vertices.push_back(vert);
		}
		else {
			vert.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pose.translation());
			vertices.push_back(vert);
		}
	}
	std::vector<double> segment_times;
	segment_times = estimateSegmentTimes(vertices, this->v_max, this->a_max);

	// set up optimisation parameters
	mav_trajectory_generation::NonlinearOptimizationParameters params;

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, params);

	opt.setupFromVertices(vertices, segment_times, derv);

	opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
	opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

	opt.optimize();

	mav_trajectory_generation::Trajectory mav_trajectory;
	
	opt.getTrajectory(&mav_trajectory);
	

	// obtain MultiDOFJointTrajectory
	mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
	bool status = mav_trajectory_generation::sampleWholeTrajectory(mav_trajectory, 0.01, &trajectory_points);

	msgMultiDofJointTrajectoryFromEigen(trajectory_points, &trajectory);
	//ROS_INFO("Trajectory optimized");

	if(publish_traj_markers){
		// publish markers
	visualization_msgs::MarkerArray markers;
	double distance = 0.5;
	std::string frame_id = "world";

	mav_trajectory_generation::drawMavTrajectory(mav_trajectory, distance, frame_id, &markers);

	marker_pub.publish(markers);
	}  		
}
}// namespace drone_navigation

