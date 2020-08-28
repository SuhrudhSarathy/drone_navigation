#include<ros/ros.h>
#include<Eigen/Dense>
#include<nav_msgs/Odometry.h>
#include<trajectory_msgs/MultiDOFJointTrajectory.h>
#include<drone_navigation/TrajectoryOptimiser.h>
#include<mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include<mav_trajectory_generation_ros/ros_visualtisation.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>
#include<mav_trajectory_generation/trajectory.h>


class Optimiser{
private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	ros::ServiceServer optimiser_service;
	ros::Publisher marker_pub

	Eigen::Affine3d current_pose;
	Eigen::Vector3d current_velocity;
	Eigen::Vector3d current_angular_velocity;

	double v_max = 2.0;
	double a_max = 2.0;
	double max_ang_vel;
	double max_ang_accel;

public:
	Optimiser(){
		odom_sub = nh.subscribe("/odometry", 1, &Optimiser::odom_callback, this);
		marker_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1);
		optimiser_service = nh.advertiseService("trajectory_optimiser", &Optimiser::traj_serivice_callback, this);

	}
	~Optimiser(){}

	void odom_callback(const nav_msgs::OdometryConstPtr& msg){
		tf::poseMsgToEigen(msg->pose.pose, this->current_pose);
		tf::vectorMsgToEigen(msg->twist.twist.linear, this->current_velocity);
	}
	bool get_trajectory(const geometry_msgs::Path& path, trajectory_msgs::MultiDOFJointTrajectory& trajectory){
		const int dimension = 3;
		const int derv = mav_trajectory_generation::derivative_order::SNAP;

		// vertices to use for trajectory optimisation;
		mav_trajectory_generation::Vertex::Vector vertices;

		for(int i = 0; i < path.poses.size(); i++){
			Eigen::Affine3d pose;
			tf::poseMsgToEigen(path.poses[i], pose);
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
		segment_times = estimateSegementTimes(vertices, this->v_max, this->a_max);

		// set up optimisation parameters
		mav_trajectory_generation::NonlinearOptimizationParameters params;

		const int N = 10;
		mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, params);

		opt.setupFromvertices(vertices, segment_times, derv);

		opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  		opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  		opt.optimize();

  		mav_trajectory_generation::Trajectory mav_trajectory;
  		
  		opt.getTrajectory(&mav_trajectory);
  		

  		// publish whole trajectory
  		mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
  		mav_trajectory_generation::sampleWholeTrajectory(mav_trajectory, 0.01, &trajectory_points);

  		mav_trajectory_generation::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &trajectory);

  		return true;
	}
	bool traj_service_callback(drone_navigation::TrajectoryOptimiser::Request& req, drone_navigation::TrajectoryOptimiser::Response& resp){
		trajectory_msgs::MultiDOFJointTrajectory trajectory;
		this->getTrajectory(req.path, trajectory);
		resp.trajectory = trajectory;
		return resp;
	}
};
int main(int argv, char** argc){
	ros::init(argc, argv, "trajectory_optimiser_external");
	Optimiser optim;
	ros::spin();
}
