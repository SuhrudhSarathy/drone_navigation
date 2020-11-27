#include "drone_navigation/rrt.h"

namespace drone_navigation{
void sample_node(Node& node_in, const Node& goal, const float prob){
	
	
	float random_prob = distribution_biased(generator);
	if(random_prob >prob){
		node_in.x = distribution_x(generator);
		node_in.y = distribution_y(generator);
		node_in.z = distribution_z(generator);
	}
	else{
		node_in.x = goal.x;
		node_in.y = goal.y;
		node_in.z = goal.z;
	}
	
}
float distance(const Node& node1, const Node& node2){
	return sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2) + pow(node1.z - node2.z, 2));
}
// comparing two nodes
bool compare(const Node& node1, const Node& node2){
	return (node1.dist < node2.dist);
}

// Node functions
// 1. Constructor
Node::Node(float x_, float y_, float z_){
	x = x_;
	y = y_;
	z = z_;
}
// 2. Destructor
Node::~Node(){}

// RRT Functions
RRT::RRT(const ros::NodeHandle nh_, const ros::NodeHandle nh_private_) :nh(nh_), nh_private(nh_private), mapManager(nh_, nh_private_)
{
	ROS_INFO("Server Initiated");
}
RRT::~RRT(){}
void RRT::set_sampling_radius(float r){
	sampling_radius = r;
}
void RRT::set_sampling_probability(float prob){
	sampling_probability = prob;
}
bool RRT::check_collision(Node& node1, Node& node2){
	bool collision = false;
	float x1, y1, z1, x2, y2, z2;
	x2 = node1.x;
	y2 = node1.y;
	z2 = node1.z;
	x1 = node2.x;
	y1 = node2.y;
	z1 = node2.z;
	float k = 0;
	while(k<=1.1){
		double x = k * (x2 - x1) + x1;
		double y = k * (y2 - y1) + y1;
		double z = k * (z2 - z1) + z1;
		const Eigen::Vector3d position(x, y, z);
		double distance = mapManager.check_collision_at_point(position);
		if(abs(distance) < sampling_radius){
			collision = true;
			break;
		}
		k += 0.5;
	}
	return collision;
}
int RRT::find_nearest_node(){
	// brute search algorithm for finding the nearest node
	// optimisation reqd
	int position = -1;
	float dist;
	float prev_distance = 100000;
	for(int i = 0; i < tree.size();i++){
		dist = distance(tree[i], current);
		if(dist < prev_distance){
			prev_distance = dist;
			position = i;
		}
	}
	return position;
}
// Call function for RRT
void RRT::call(Node& start_, Node& goal_, nav_msgs::Path& path){
	std::vector<Node> waypoints;
	this->goal = goal_;
	this->start = start_;
	n_iters = 1000;
	tree.push_back(start);

	while(n_iters > 0){
		// main loop
		// step 1: sample a node
		sample_node(current, goal, sampling_probability);

		// find the closest neighbour in the tree;
		int position = find_nearest_node();
		// std::cout << position << std::endl;
		Node neighbour = tree[position];

		// check for distance between the nodes
		float dist = distance(current, neighbour);

		if(dist < sampling_radius){
			// check for collision
			if(!check_collision(current, neighbour)){
				current.parent = position;
				tree.push_back(current);
				// std::cout << position << std::endl;
				if(distance(current, goal) < 0.01){
					std::cout << "goal reached" << std::endl;
					break;
				}
			}
			
		}
		else {
			// make the node within the sampling radius
			current.x = neighbour.x + sampling_radius * ((current.x - neighbour.x)/distance(current, neighbour));
			current.y = neighbour.y + sampling_radius * ((current.y - neighbour.y)/distance(current, neighbour));
			current.z = neighbour.z + sampling_radius * ((current.z - neighbour.z)/distance(current, neighbour));

			if(!check_collision(current, neighbour)){
				current.parent = position;
				tree.push_back(current);
				// std::cout << position << std::endl;
				if(distance(current, goal) < 0.01){
					std::cout << "goal reached" << std::endl;
					break;
				}
			}
		}		
		n_iters--;
	}
	// obtain path and store it in waypoints	

	// search the path by back tracking
	Node present = tree[tree.size() - 1];
	int n = tree.size();
	while(n > 0){
		waypoints.push_back(present);
		if(present.parent == -1){
			std::cout << "goal found in path" << std::endl;
			break;
		}
		else{
			//std::cout << present.parent << std::endl;
			present = tree[present.parent];
		}
		n--;
	}
	// convert the waypoints to Path
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "world";

	for(int i = 0; i < waypoints.size();i++){
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "world";
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		pose.pose.position.x = waypoints[i].x;
		pose.pose.position.y = waypoints[i].y;
		pose.pose.position.z = waypoints[i].z;

		path.poses.push_back(pose);
	}
}
}// namespace drone_navigation
