#!/usr/bin/env python
import time
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from drone_navigation.srv import PointQuery, PointQueryRequest, PointQueryResponse
from drone_navigation.srv import TrajectoryQuery, TrajectoryQueryRequest, TrajectoryQueryResponse
from drone_navigation.srv import Planner, PlannerRequest, PlannerResponse
from drone_navigation.srv import TrajectoryOptimiser, TrajectoryOptimiserRequest
import rospy
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler as qfe


class Node:
    def __init__(self, x, y,z=0):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
    def __str__(self):
        return "{}, {}, {}".format(self.x, self.y, self.z)


def generate_node(sample_area, alpha, goal, dim=2):
    sample_area_x, sample_area_y, sample_area_z = sample_area
    num = np.random.random()
    if num < alpha:
        #print('goal', goal)
        return goal
    else:
        if dim == 2:
            return Node(
                np.random.uniform(sample_area_x[0], sample_area_x[1]),
                np.random.uniform(sample_area_y[0], sample_area_y[1]),
                #np.random.uniform(sample_area[0], sample_area[1])
            )
        else:
            return Node(
                np.random.uniform(sample_area_x[0], sample_area_x[1]),
                np.random.uniform(sample_area_y[0], sample_area_y[1]),
                abs(np.random.uniform(sample_area_z[0], sample_area_z[1]))
            )

def lineTo(node, nearest_node, delta, dim=2):
    if dim == 3:
        vec = np.array([node.x - nearest_node.x, node.y-nearest_node.y, node.z-nearest_node.z])
        vec = (vec / np.linalg.norm(vec)) * delta
        node = Node(nearest_node.x + vec[0], nearest_node.y + vec[1], nearest_node.z + vec[2])
        return node
    elif dim == 2:
        vec = np.array([node.x - nearest_node.x, node.y-nearest_node.y])
        vec = (vec / np.linalg.norm(vec)) * delta
        node = Node(nearest_node.x + vec[0], nearest_node.y + vec[1])
        return node


def collision(point1, point2):
    ## Collision service call
    rospy.wait_for_service("trajectory_collision_status")
    try:
        request = TrajectoryQueryRequest()
        if isinstance(point1, Node):
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

def nn(node, tree):
    return sorted(tree, key=lambda n: distance(n, node))[0]


def distance(node1, node2, dim=2):
    if dim==3:
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2 + (node1.z - node2.z)**2)
    else:
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)


class RRT():
    def __init__(self, animation=False, dim=3):
        self.tree = []
        self.delta = 0.3
        self.alpha = 0.5
        self.animation = animation
        self.dim = dim
        self.planning_service = rospy.Service("rrt_planner", Planner, self.__call__)
        self.path_pub = rospy.Publisher("/rrt_path", Path, queue_size=10)
        self.optimiser = rospy.ServiceProxy("trajectory_optimiser", TrajectoryOptimiser)
        self.optim_path_pub = rospy.Publisher("optim_path", Path, queue_size=10)
        

    def __call__(self, req):
        self.start = Node(req.start.x, req.start.y, req.start.z)
        self.goal = Node(req.goal.x, req.goal.y, req.goal.z)
        self.tree = [self.start]

        response = PlannerResponse()
        
        rospy.loginfo("Planner called from [%f %f %f] to [%f %f %f]"%(req.start.x, req.start.z, req.start.z, req.goal.x, req.goal.y, req.goal.z))
        n_iters = 1000
        sample_area = [[-5 + self.start.x, 5 + self.goal.x], [-5 + self.start.y, 5 + self.goal.y], [-2 + self.start.z,  2 + self.goal.z]]
        #Run till tne number of iterations are not reached
        time1 = time.time()
        while n_iters >= 0:
            # genertae goal node, biased to return goal as the node with a probability alpha
            node = generate_node(sample_area, self.alpha, self.goal ,dim=self.dim)

            #look for the nearest node in the tree 
            nearest_node = nn(node, self.tree)

            # If distacne to the the nearest node is greater than the threshold

            if distance(node, nearest_node,dim=self.dim) > self.delta:
                
                # Make a new node and then check for collision
                node = lineTo(node, nearest_node, self.delta,dim=self.dim)
                
                # if no collision , then add it to the tree
                if not collision(node, nearest_node):
                    #print("added")
                    node.parent = nearest_node
                    self.tree.append(node)
            
            # Else, 
            else:
                # Check for collision, If no collision add it to the tree
                if not collision(node, nearest_node):
                    #print("added")
                    node.parent = nearest_node
                    self.tree.append(node)

            # If you approach the goal node, break the loop
            if abs(node.x - self.goal.x) < 0.01 and abs(node.y - self.goal.y) < 0.01 and abs(node.z - self.goal.z) < 0.01:
                #print("goal reached")
                response.reached = True
                break
            if self.animation:
                self.animate()
            # Count iteration as done
            n_iters -= 1

        #print([(node.x, node.y, node.z) for node in self.tree])
        
        path = self.get_path()
        path = path[::-1]
        #optim_path = self.optimised_path(path)        

        path_msg = Path()
        path_msg.header = Header(frame_id="world", stamp=rospy.Time.now())

        #angle = np.arctan2(self.goal.y - self.start.y, self.goal.x - self.start.x)

        for point in path:
            pose = PoseStamped()
            pose.header = Header(frame_id="world", stamp=rospy.Time.now())
            pose.pose.position = Point(point.x, point.y, point.z)
            '''if angle < 0:
                quat = qfe(0, 0, 180)
                print(quat)
                pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
            else:
                pose.pose.orientation = Quaternion(0, 0, 0, 1)'''
            pose.pose.orientation = Quaternion(0, 0, 0, 1)
            path_msg.poses.append(pose)

        

        optim_req = TrajectoryOptimiserRequest()
        optim_req.crude_path = path_msg;
        optim_resp = self.optimiser(optim_req)
        response.path = optim_resp.optimised_path
        
        """optim_path_msg = Path()
        optim_path_msg.header = Header(frame_id="world", stamp=rospy.Time.now())

        for point in optim_path:
            pose = PoseStamped()
            pose.header = Header(frame_id="world", stamp=rospy.Time.now())
            pose.pose.position = Point(point.x, point.y, point.z)
            pose.pose.orientation = Quaternion(0, 0, 0, 1)
            path_msg.poses.append(pose)
        """
        self.path_pub.publish(path_msg)
        self.optim_path_pub.publish(optim_resp.optimised_path)
        return response #, optim_path[::-1]

    def get_path(self):
        
        current = self.tree[-1]        
        path = [current]
        while current.parent != None:
            path.append(current.parent)
            current = current.parent
        return path

    def optimised_path(self, path):
        optimized_path = [path[0]]
        current_index = 0
        while current_index < len(path) - 1:

            # Keep track of whether index has been updated or not
            index_updated = False

            # Loop from last point in path to the current one, checking if
            # any direct connection exists.
            for lookahead_index in range(len(path) - 1, current_index, -1):
                if not collision(path[current_index], path[lookahead_index]):
                    # If direct connection exists then add this lookahead point to optimized
                    # path directly and skip to it for next iteration of while loop
                    optimized_path.append(path[lookahead_index])
                    current_index = lookahead_index
                    index_updated = True
                    break

            # If index hasnt been updated means that there was no LOS shortening
            # and the edge between current and next point passes through an obstacle.
            if not index_updated:
                # In this case we return the path so far
                return optimized_path

        return optimized_path
    def animate(self):
        pass

if __name__ == "__main__":
    rospy.init_node("planner")
    path_planner = RRT(dim=3)
    rospy.loginfo("Path Planning Service Initiated")
    rospy.spin()