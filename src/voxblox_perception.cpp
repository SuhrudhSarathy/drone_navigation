#include <drone_navigation/voxblox_perception.h>

namespace drone_navigation{
MapManagerVoxblox::MapManagerVoxblox(const ros::NodeHandle& nh_, const ros::NodeHandle& nh_private_) : nh(nh_), nh_private(nh_private_), server(nh_, nh_private_)
{
    ROS_INFO("Initiated Voxblox Map Manager");
}
MapManagerVoxblox::~MapManagerVoxblox(){}
// Check voxel status
enum MapManagerVoxblox::VoxelStatus MapManagerVoxblox::get_voxel_status(const Eigen::Vector3d& position) const{
    if(!server.getEsdfMapPtr()->isObserved(position)){
        return VoxelStatus::kUnknown;
    }
    else{
        double distance = 0.0;
        if(!server.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)){
            return VoxelStatus::kUnknown;
        }
        else{                
            if(abs(distance) <= dist_threshold){
                return VoxelStatus::kOccupied;
            }
            else {
                return VoxelStatus::kFree;
            }
        }
        
    }
}

// Obtain the nearest distance to obstacle at a given point
double MapManagerVoxblox::get_distance_at_point(const Eigen::Vector3d& position) const{
    if(get_voxel_status(position) == VoxelStatus::kUnknown){
        return 0.0;
    }
    else {
        double distance = 0.0;
        if(!server.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)){
            return 0.0;
        }
        return distance;
    }
}

// Explicitly check for collision
bool MapManagerVoxblox::check_collision_at_point(const Eigen::Vector3d& position){
    if(get_voxel_status(position) == VoxelStatus::kOccupied || get_voxel_status(position) == VoxelStatus::kUnknown){
        return true;
    }
    else{
        return false;
    } 
}
}// nnamespace ends heres