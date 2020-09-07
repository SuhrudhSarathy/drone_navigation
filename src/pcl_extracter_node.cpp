#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>
#include<pcl/filters/voxel_grid.h>
#include<vector>
#include<tf2_ros/transform_listener.h>
#include<pcl_ros/transforms.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf/transform_datatypes.h>

ros::Publisher point_pub;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt (new pcl::PointCloud<pcl::PointXYZ>);

	cloud = msg;
	
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 3.0);

	pass.filter(*cloud_pt);

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_pt);
	sor.setLeafSize(0.1, 0.1, 0.1);
	sor.filter(*cloud_voxel);

	point_pub.publish(cloud_voxel);
}

int main(int argv, char** argc){
	ros::init(argv, argc, "point_cloud_extraction");
	ros::NodeHandle nh;
	ros::Subscriber point_sub = nh.subscribe("input", 10, callback);
	point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("points", 10);
	ros::spin();
}