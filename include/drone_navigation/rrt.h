#ifndef RRT_H
#define RRT_H


// ros and voxblox
#include <ros/ros.h>
#include "drone_navigation/voxblox_perception.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// end
#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <cmath>
#include <algorithm>
// class Node 

namespace drone_navigation{
	class Node{
	public:
		float x, y, z;
		float dist = 0;
		Node(float = 0, float = 0, float = 0);
		~Node();
		int parent = -1;
	};

	// samplers
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 generator(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<float> distribution_x(-2.0, 10.0);
	std::uniform_real_distribution<float> distribution_y(-2.0, 10.0);
	std::uniform_real_distribution<float> distribution_z(1, 2);
	std::uniform_real_distribution<float> distribution_biased(0.0, 1.0);
	// functions
	void sample_node(Node& node_in, const Node& goal, const float prob);
	bool check_collision(const Node& node1, const Node& node2);
	float distance(const Node&, const Node&);
	bool compare(const Node&, const Node&);

	class RRT{
	public:
		RRT(const ros::NodeHandle, const ros::NodeHandle);
		~RRT();
		void set_sampling_radius(float r);
		void set_sampling_probability(float prob);
		void call(Node& start, Node& goal, nav_msgs::Path& path);
		int find_nearest_node();
		bool check_collision(Node&, Node&);

	private:
		ros::NodeHandle nh;
		ros::NodeHandle nh_private;
		MapManagerVoxblox mapManager;
		float sampling_radius = 0.5;
		float sampling_probability = 0.1;
		Node goal;
		Node start;
		Node current;
		std::vector<Node> tree;
		int n_iters = 1000;
	};
}// namespace drone_navigation
#endif
