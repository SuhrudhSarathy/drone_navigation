#!/usr/bin/env python
import rospy
from drone_navigation.srv import TrajectoryOptimiser, TrajectoryOptimiserRequest, TrajectoryOptimiserResponse
from drone_navigation.srv import TrajectoryQuery, TrajectoryQueryRequest, TrajectoryQueryResponse
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, PoseStamped
import numpy as np


def collision(point1, point2):
    ## Collision service call
    rospy.wait_for_service("trajectory_collision_status")
    try:
        request = TrajectoryQueryRequest()
        if isinstance(point1, Point):
        	pose1 = Pose(position = Point(point1.x, point1.y, point1.z), orientation = Quaternion(0, 0, 0, 1))
        	pose2 = Pose(position = Point(point2.x, point2.y, point2.z), orientation = Quaternion(0, 0, 0, 1))
        elif isinstance(point1, tuple):
            pose1 = Pose(position = Point(point1[0], point1[1], point1[2]), orientation = Quaternion(0, 0, 0, 1))
            pose2 = Pose(position = Point(point2[1], point2[2], point2[2]), orientation = Quaternion(0, 0, 0, 1))

        request.waypoints = PoseArray(poses = [pose1, pose2])
        service = rospy.ServiceProxy("trajectory_collision_status", TrajectoryQuery)
        resp = service(request)
        if resp.collision == True:
            return True
        else:
            return False
    except rospy.ServiceException as e:
        print("Service call failed %s"%e)

def LOS_Optimiser(req):
	resp = TrajectoryOptimiserResponse()
	resp.optimised_path.header = req.crude_path.header
	path = req.crude_path.poses
	optimised_path = [path[0]]
	current_index = 0
	while current_index < len(path) - 1:
		ind_updated = False
		for ind2 in range(len(path) - 1, current_index, -1):
			if not collision(path[current_index].pose.position, path[ind2].pose.position):
				optimised_path.append(path[ind2])
				current_index = ind2
				ind_updated = True
				break
		if not ind_updated:
			resp.optimised_path.poses = BREAK_Optimiser(optimised_path)
			#resp.optimised_path = BREAK_Optimiser(resp.optimised_path)
			
			return resp
	resp.optimised_path.poses = BREAK_Optimiser(optimised_path)
	#resp.optimised_path = BREAK_Optimiser(resp.optimised_path)
	
	return resp

def BREAK_Optimiser(poses):
	poses_new = []
	for i in range(len(poses) - 1):
		pres_point = poses[i].pose.position
		next_point = poses[i+1].pose.position
		dist = np.sqrt((pres_point.x - next_point.x)**2 + (pres_point.y - next_point.y)**2 + (pres_point.z - next_point.z)**2)
		n = dist/0.5
		incr = 1/n 
		a = np.arange(0, 1 + incr, incr)
		print(a)
		a[a >= 1] = 1
		print(a)
		for k in a:
			pose = PoseStamped()
			pose.header = poses[0].header
			x = (1 - k) * pres_point.x + k * next_point.x
			y = (1 - k) * pres_point.y + k * next_point.y
			z = (1 - k) * pres_point.z + k * next_point.z
			pose.pose.position.x = x
			pose.pose.position.y = y
			pose.pose.position.z = z
			pose.pose.orientation.x = 0
			pose.pose.orientation.y = 0
			pose.pose.orientation.z = 0
			pose.pose.orientation.w = 1

			poses_new.append(pose)
	#print poses_new
	return poses_new

if __name__ == "__main__":
	rospy.init_node("trajectory_optimiser")
	traj_optimser = rospy.Service("trajectory_optimiser", TrajectoryOptimiser, LOS_Optimiser)
	rospy.spin()	




