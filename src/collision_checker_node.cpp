#include<ros/ros.h>
#include<iostream>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/octree/octree_search.h>
#include<vector>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/TransformStamped.h>
#include<drone_navigation/PointQuery.h>
#include<drone_navigation/TrajectoryQuery.h>
#include<tf/transform_listener.h>
#include<pcl_ros/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
void pcl_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
	cloud = msg;
}
bool point_status_callback(drone_navigation::PointQuery::Request &req, drone_navigation::PointQuery::Response &resp){
	// Octree for collision checking
	float resolution = 128.0f;

	//octree rep
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	// serach for neighbours in a given radius
	pcl::PointXYZ searchPoint;
	searchPoint.x = req.point.x;
	searchPoint.y = req.point.y;
	searchPoint.z = req.point.z;

	ROS_INFO("Server called for [%f], [%f], [%f]", req.point.x, req.point.y, req.point.z);

	std::vector<int> idx;
	std::vector<float> distances;
	float radius = 0.3;

	if(octree.radiusSearch(searchPoint, radius, idx, distances) > 0){
		resp.collision = true;
		ROS_INFO("Sent TRUE");
	}
	else{
		resp.collision = false;
		ROS_INFO("Sent FALSE");
	}
	
	return true;
}
bool trajectory_status_callback(drone_navigation::TrajectoryQuery::Request &req, drone_navigation::TrajectoryQuery::Response &resp){
	float resolution = 128.0f;

	//octree rep
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	ROS_INFO_ONCE("Trajectory Status Server Called");

	std::vector<geometry_msgs::Pose> waypoints = req.waypoints.poses;
	float radius = 0.3;

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

			pcl::PointXYZ searchPoint;
			searchPoint.x = k * (x2 - x1) + x1;
			searchPoint.y = k * (y2 - y1) + y1;
			searchPoint.z = k * (z2 - z1) + z1;

			std::vector<int> idx;
			std::vector<float> dist;
			if(octree.radiusSearch(searchPoint, radius, idx, dist) > 0){
				//std::cout<< "Collision detected at " << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << std::endl;
				resp.collision = true;
				return true;
			}
			k += 0.1; 
		}
	}
	return true;
}
//void get_points(geometry_msgs::PoseArray&)
int main(int argv, char** argc){
	ros::init(argv, argc, "collision_Checker");
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub = nh.subscribe("input", 10, pcl_callback);
	ros::ServiceServer point_coll_service = nh.advertiseService("point_collision_status", point_status_callback);
	ros::ServiceServer traj_coll_service = nh.advertiseService("trajectory_collision_status", trajectory_status_callback);
	ROS_INFO("Collision Checker Server initiated");
	ros::spin();

}