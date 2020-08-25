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
		trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);
		//pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);
		planner_service = nh.serviceClient<drone_navigation::Planner>("rrt_planner");
		collision_checker_service = nh.serviceClient<drone_navigation::TrajectoryQuery>("trajectory_collision_status");
		goal_sub = nh.subscribe("/goal_for_drone", 1, &Commander::goal_callback, this);
	}
	~Commander(){};

	void stop_motion(){
		// publish present position as a command/trajectory
		trajectory_msgs::MultiDOFJointTrajectory trajectory;
		trajectory_msgs::MultiDOFJointTrajectoryPoint point;
		geometry_msgs::Transform transform;
		geometry_msgs::Twist vel;
		geometry_msgs::Twist accel;

		transform.translation.x = this->position.x;
		transform.translation.y = this->position.y;
		transform.translation.z = this->position.z;

		transform.rotation = this->orientation;

		point.transforms.push_back(transform);
		point.velocities.push_back(vel);
		point.accelerations.push_back(accel);
		
		trajectory.header.stamp = ros::Time::now();
		trajectory.header.frame_id = "world";

		trajectory.points.push_back(point);

		trajectory_pub.publish(trajectory);
		ros::Duration(1.0).sleep();
	}

	void odom_callback(const nav_msgs::OdometryConstPtr& msg){
		this->position = msg->pose.pose.position;
		this->orientation = msg->pose.pose.orientation;
		if(this->goal_recieved){
			this->monitor_collisions();
			geometry_msgs::Point curr_target = this->path.poses[current_index].pose.position;
			float distance = sqrt(pow(curr_target.x - this->position.x, 2) + pow(curr_target.y - this->position.y, 2) + pow(curr_target.z - this->position.z, 2));
			if(distance < 0.25){
				if(current_index < this->path.poses.size()){
					current_index ++;
				}
				else{
					ROS_INFO("Reached Final Goal");
				}
			}
			std::cout << distance << " " << current_index << std::endl;
			this->publish_trajectory();
		}
	}
	void goal_callback(const geometry_msgs::PointConstPtr& msg){
		this->goal.x = msg->x;
		this->goal.y = msg->y;
		this->goal.z = msg->z;
		this->goal_recieved = true;
		this->call_planner();
		// set current index to zero because new path is obtained
		this->current_index = 0;
		this->publish_trajectory();
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
				ROS_INFO("Goal Reached");
			}
			else if(!srv.response.reached){
				ROS_WARN("Goal Not Reached");
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
			trajectory_msgs::MultiDOFJointTrajectoryPoint point;
			geometry_msgs::Transform transform;
			geometry_msgs::Twist vel, accel;

			trajectory.header.frame_id = "world";
			trajectory.header.stamp = ros::Time::now();

			transform.translation.x = this->path.poses[current_index].pose.position.x;
			transform.translation.y = this->path.poses[current_index].pose.position.y;
			transform.translation.z = this->path.poses[current_index].pose.position.z;

			transform.rotation.x = 0;
			transform.rotation.y = 0;
			transform.rotation.z = 0;
			transform.rotation.w = 1;

			point.transforms.push_back(transform);
			point.velocities.push_back(vel);
			point.accelerations.push_back(accel);
			
			trajectory.points.push_back(point);
			this->trajectory_pub.publish(trajectory);
		}
		else {
			ROS_WARN("Recieved an empty path");
		}
		
	}

	bool check_for_collision(){
		// Working perfectly
		drone_navigation::TrajectoryQuery srv;
		geometry_msgs::PoseArray pose_array;
		for(int i = 0;i < this->path.poses.size();i++){
			pose_array.poses.push_back(this->path.poses[i].pose);
		}
		srv.request.waypoints = pose_array;
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
	 		this->stop_motion();
	 		ros::Duration(1).sleep();
	 		this->call_planner();
	 		this->publish_trajectory();
	 		ROS_INFO("Collision Detected");
	 	}
	 	else {
	 		this->publish_trajectory();
	 	}
	 }
};

int main(int argv, char** argc){
	ros::init(argv, argc, "commander");
	Commander commander;
	ROS_INFO("Initialised Commander");
	ros::spin();
}