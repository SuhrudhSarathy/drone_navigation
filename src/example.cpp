#include<ros/ros.h>
//PCL specific includes
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<mav_trajectory_generation/polynomial_optimisation_nonlinear.h>
ros::Publisher pub;


void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}


int main(int argc, char** argv){

    ros::init(argc, argv, "pcl_tutorials");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/camera/depth/points", 1, callback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/depth/image", 1);
    ROS_INFO("Publishing point cloud"); 
    ros::spin();
}