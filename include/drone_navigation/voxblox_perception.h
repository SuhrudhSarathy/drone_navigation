#ifndef VOXBLOX_PERCEPTION_H
#define VOXBLOX_PERCEPTION_H

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox/utils/planning_utils.h>
#include <voxblox/utils/planning_utils_inl.h>
#include <Eigen/Dense>
#include <cmath>
#include <string>


namespace drone_navigation{
class MapManagerVoxblox{
    public:
        MapManagerVoxblox(const ros::NodeHandle&, const ros::NodeHandle&);
        ~MapManagerVoxblox();
        enum VoxelStatus{kUnknown = -1, kFree = 0, kOccupied = 1};
        enum VoxelStatus get_voxel_status(const Eigen::Vector3d& position) const;
        double get_distance_at_point(const Eigen::Vector3d& position) const;
        bool check_collision_at_point(const Eigen::Vector3d& position);
        
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        voxblox::EsdfServer server;
        const float dist_threshold = 0.3;
        
};
} //namespace drone_navigation

#endif