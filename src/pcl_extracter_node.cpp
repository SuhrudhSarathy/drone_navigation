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
	sor.setLeafSize(0.05, 0.05, 0.05);
	sor.filter(*cloud_voxel);

	pcl::PointCloud<pcl::PointXYZ> cloud_out;
	try{
		geometry_msgs::TransformStamped transform;
		transform = tfBuffer.lookupTransform("world", msg->header.frame_id ,ros::Time(0), ros::Duration(3.0));
		tf::Transform tftrans;
		tf::transformMsgToTF(transform.transform, tftrans);
		pcl_ros::transformPointCloud(*cloud_voxel, cloud_out, tftrans);
	}
	catch(tf2::TransformException &ex){
		ROS_WARN("%s", ex.what());
	}
	point_pub.publish(cloud_out);
}

int main(int argv, char** argc){
	ros::init(argv, argc, "point_cloud_extraction");
	ros::NodeHandle nh;
	ros::Subscriber point_sub = nh.subscribe("input", 10, callback);
	point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("points", 10);
	ros::spin();
}