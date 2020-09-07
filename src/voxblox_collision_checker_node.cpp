#include<ros/ros.h>
#include<drone_navigation/PointQuery.h>
#include<drone_navigation/TrajectoryQuery.h>
#include<voxblox_ros/esdf_server.h>
#include<Eigen/Dense>
#include<iostream>
#include<cmath>
#include<tf2_ros/transform_listener.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/PoseStamped.h>

class CollisionChecker{
private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;
	voxblox::EsdfServer voxblox_server;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
public:
	double robotClearance = 0.3;

	CollisionChecker(const ros::NodeHandle nh_, const ros::NodeHandle nh_private_) : nh(nh_), nh_private(nh_private_), voxblox_server(nh, nh_private), tfListener(tfBuffer)
	{		
		voxblox_server.setTraversabilityRadius(robotClearance);
		voxblox_server.publishTraversable();
		
		ROS_INFO("Server Initiated");
	}
	~CollisionChecker(){}

	double getDistanceToClosestObstacle(const Eigen::Vector3d& position) const{
		if(!voxblox_server.getEsdfMapPtr()){
			return 0.0;
		}
		double distance = 100.0;
		if(!voxblox_server.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)){
			return 0.0;
		}
		return distance;
	}

	bool point_service_callback(drone_navigation::PointQuery::Request& req, drone_navigation::PointQuery::Response& resp){
		geometry_msgs::PoseStamped point_in;

		point_in.header.frame_id = "world";
		point_in.header.stamp = ros::Time::now();

		point_in.pose.position.x = req.point.x;
		point_in.pose.position.y = req.point.y;
		point_in.pose.position.z = req.point.z;

		transform_point(point_in);
		std::cout << point_in.pose.position.x << point_in.pose.position.y << point_in.pose.position.z;
		Eigen::Vector3d position(point_in.pose.position.x, point_in.pose.position.y, point_in.pose.position.z);
		double distance = getDistanceToClosestObstacle(position);
		if(abs(distance) < this->robotClearance){
			resp.collision = true;
			ROS_INFO("TRUE");
		}
		else {
			resp.collision = false;
			ROS_INFO("False");
		}
		return true;
	}
	bool trajectory_service_callback(drone_navigation::TrajectoryQuery::Request& req, drone_navigation::TrajectoryQuery::Response& resp){
		std::vector<geometry_msgs::Pose> waypoints = req.waypoints.poses;
		//std::cout << "Service Called" << std::endl;

		for(int i = 0; i < waypoints.size() - 1; i++){
			float k = 0;
			resp.collision = false;
			while(k<=1.1 && !resp.collision){
				float x1, y1, z1, x2, y2, z2;
				x2 = waypoints[i+1].position.x;
				y2 = waypoints[i+1].position.y;
				z2 = waypoints[i+1].position.z;
				x1 = waypoints[i].position.x;
				y1 = waypoints[i].position.y;
				z1 = waypoints[i].position.z;

				double x = k * (x2 - x1) + x1;
				double y = k * (y2 - y1) + y1;
				double z = k * (z2 - z1) + z1;

				geometry_msgs::PoseStamped point_in;
				
				point_in.header.frame_id = "world";
				point_in.header.stamp = ros::Time::now();

				point_in.pose.position.x = x;
				point_in.pose.position.y = y;
				point_in.pose.position.z = z;

				transform_point(point_in);
				// use x, y, z obtained after transforming
				Eigen::Vector3d position(point_in.pose.position.x, point_in.pose.position.y, point_in.pose.position.z);
				double distance = getDistanceToClosestObstacle(position);
				if(abs(distance) < this->robotClearance){
					resp.collision = true;
					return true;
				}
				k += 0.1;
			}
		}

	}
	void transform_point(geometry_msgs::PoseStamped& point_in){
		try{
			geometry_msgs::TransformStamped transform;
			transform = tfBuffer.lookupTransform("firefly/vi_sensor/base_link", "world", ros::Time(0), ros::Duration(3.0));
			tf2::doTransform(point_in, point_in, transform);

		}
		catch (tf2::TransformException &ex){
			ROS_WARN("Could not transform");
		}
	}
};
int main(int argv, char** argc){
	ros::init(argv, argc, "voxblox_collision_checker_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;
	CollisionChecker ccl(nh, nh_private);
	ros::ServiceServer collisionCheck = nh.advertiseService("voxblox_point_collision_check", &CollisionChecker::point_service_callback, &ccl);
	ros::ServiceServer trajcollisionCheck = nh.advertiseService("voxblox_trajectory_collision_check", &CollisionChecker::trajectory_service_callback, &ccl);
	ros::spin();
	return 0;
}
