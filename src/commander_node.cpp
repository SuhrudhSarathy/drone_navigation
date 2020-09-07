#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<drone_navigation/TrajectoryQuery.h>
#include<drone_navigation/Planner.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Transform.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/PoseStamped.h>
#include<trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include<trajectory_msgs/MultiDOFJointTrajectory.h>
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
#include<iostream>

class Commander{
private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	//ros::Publisher pose_pub;
	ros::Publisher trajectory_pub;
	ros::ServiceClient planner_service;
	ros::ServiceClient collision_checker_service;
	ros::Subscriber goal_sub;
	ros::Publisher marker_pub;
	float v_max = 2;
	float a_max = 2;
public:
	geometry_msgs::Point start;
	geometry_msgs::Point goal;
	geometry_msgs::Point position;
	geometry_msgs::Quaternion orientation;
	bool goal_recieved = false;
	// a variable to keep track of current position in the path
	int current_index = 0;
	nav_msgs::Path path;

	Commander(){

		odom_sub = nh.subscribe("/firefly/vi_sensor/ground_truth/odometry", 1, &Commander::odom_callback, this);
		trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 50);
		//pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);
		planner_service = nh.serviceClient<drone_navigation::Planner>("rrt_planner");
		collision_checker_service = nh.serviceClient<drone_navigation::TrajectoryQuery>("voxblox_trajectory_collision_check");
		goal_sub = nh.subscribe("/goal_for_drone", 1, &Commander::goal_callback, this);
		marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);
	}
	~Commander(){};

	void stop_motion(){
		// publish present position as a command/trajectory
		trajectory_msgs::MultiDOFJointTrajectory trajectory;
		trajectory.header.frame_id = "world";
		trajectory.header.stamp = ros::Time::now();
		Eigen::Vector3d position(this->position.x, this->position.y, this->position.z);
		double yaw = 0;
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position, yaw, &trajectory); 
		trajectory_pub.publish(trajectory);
	}

	void odom_callback(const nav_msgs::OdometryConstPtr& msg){
		this->position = msg->pose.pose.position;
		this->orientation = msg->pose.pose.orientation;
		if(this->goal_recieved){
			this->monitor_collisions();
		}
	}
	void goal_callback(const geometry_msgs::PointConstPtr& msg){
		this->goal.x = msg->x;
		this->goal.y = msg->y;
		this->goal.z = msg->z;
		this->goal_recieved = true;
		this->call_planner();
		this->publish_trajectory();
		// set current index to zero because new path is obtained
		this->current_index = 0;
		ROS_INFO("Recieved goal and called the planner");
	}
	void call_planner(){
		// function to call the planner service and store the path 
		drone_navigation::Planner srv;
		srv.request.start = this->position;
		srv.request.goal = this->goal;

		if(planner_service.call(srv)){
			this->path = srv.response.path;
			if(srv.response.reached){
				ROS_INFO("Planner Reached Goal Endpoint");
			}
			else if(!srv.response.reached){
				ROS_WARN("Goal Not Reached by Planner");
			}
		}
		else {
			ROS_ERROR("Path Planner service couldnot be called");
		}

	}

	void publish_trajectory(){
		// function to publish the commanded trajectory
		// check if you recieved a non-empty path
		if(this->path.poses.size() > 0){

			trajectory_msgs::MultiDOFJointTrajectory trajectory;
			trajectory.header.frame_id = "world";
			trajectory.header.stamp = ros::Time::now();
			try{
				this->optimise_trajectory(this->path, trajectory);
				this->trajectory_pub.publish(trajectory);
				ROS_INFO("Publishing Trajectory");
			}
			catch(...){
				this->call_planner();
				this->optimise_trajectory(this->path, trajectory);
				ROS_ERROR("There has been an error");
			}			
		}
		else {
			ROS_WARN("Recieved an empty path");
		}
		
	}

	bool check_for_collision(){
		// Working perfectly
		drone_navigation::TrajectoryQuery srv;
		//srv.request.point = this->position;
		geometry_msgs::PoseArray waypoints;
		for(int i = 0 ; i <this->path.poses.size(); i++){
			waypoints.poses.push_back(this->path.poses[i].pose);
		}
		srv.request.waypoints = waypoints;
		if(collision_checker_service.call(srv)){
			return srv.response.collision;
		}
		else{
			ROS_ERROR("Couldnot call Collision Checker");
			return false;
		}
	}
	 void monitor_collisions(){
	 	if(this->check_for_collision()){
	 		//this->stop_motion();
	 		this->call_planner();
	 		this->publish_trajectory();
	 		ROS_INFO("Collision Detected");
	 	}
	 }
	 void optimise_trajectory(const nav_msgs::Path& path, trajectory_msgs::MultiDOFJointTrajectory& trajectory){
	 	const int dimension = 3;
		const int derv = mav_trajectory_generation::derivative_order::JERK;

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
  		

  		// publish whole trajectory
  		mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
  		bool status = mav_trajectory_generation::sampleWholeTrajectory(mav_trajectory, 0.01, &trajectory_points);

  		msgMultiDofJointTrajectoryFromEigen(trajectory_points, &trajectory);
  		//ROS_INFO("Trajectory optimized");

  		// publish markers
  		visualization_msgs::MarkerArray markers;
  		double distance = 0.5;
  		std::string frame_id = "world";

  		mav_trajectory_generation::drawMavTrajectory(mav_trajectory, distance, frame_id, &markers);

  		marker_pub.publish(markers);
	 }
};

int main(int argv, char** argc){
	ros::init(argv, argc, "commander");
	Commander commander;
	ROS_INFO("Initialised Commander");
	ros::spin();
}