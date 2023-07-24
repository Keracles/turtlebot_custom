#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Created By  : Khéo Albarede-Barte
# Created Date: 03.06.2023
# version ='1.1'
# CS470 Introduction to Artificial Intelligence: final project
# ---------------------------------------------------------------------------
"""This file is used for the turtle bot 3 to explore the unknown environment. The algorithm is based on a publication by Brian Yamauchi: A frontier-based approach for autonomous exploration""" 
# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
import rospy
import actionlib
import tf
import numpy as np
import statistics
from skimage.measure import find_contours
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray

class UnionFind:
    """Union-Find algorithm to find clusters for each frontier"""
    def __init__(self):
        """Create a new empty union-find structure."""
        self.weights = {}
        self.parents = {}
    def __getitem__(self, obj):
        """X[item] will return the token object of the set which contains `item`"""

        if obj not in self.parents:
            self.parents[obj] = obj
            self.weights[obj] = 1
            return obj

        path = [obj]
        root = self.parents[obj]
        while root != path[-1]:
            path.append(root)
            root = self.parents[root]

        for ancestor in path:
            self.parents[ancestor] = root
        return root
    def union(self, obj1, obj2):
        """Merges sets containing obj1 and obj2."""
        roots = [self[obj1], self[obj2]]
        heavier = max([(self.weights[r],r) for r in roots])[1]
        for r in roots:
            if r != heavier:
                self.weights[heavier] += self.weights[r]
                self.parents[r] = heavier

def calculate_frontiers():
    global map_raw
    global map_origin
    global map_res
    global cluster_trashhole
    global min_cluster_size

    saved_map = np.copy(map_raw)

    contours_negative = find_contours(saved_map, -1.0, fully_connected='high')
    contours_positive = find_contours(saved_map,  1.0, fully_connected='high')

    if len(contours_negative) == 0 or len(contours_positive) == 0:
        rospy.sleep(1)
        controller()

    contours_negative = np.concatenate(contours_negative, axis=0)
    for index in range(len(contours_negative)):
        contours_negative[index][0] = round(contours_negative[index][0]\
                                    * map_res + map_origin[0], 2)
        contours_negative[index][1] = round(contours_negative[index][1]\
                                    * map_res + map_origin[1], 2)

    contours_positive = np.concatenate(contours_positive, axis=0)
    for index in range(len(contours_positive)):
        contours_positive[index][0] = round(contours_positive[index][0]\
                                    * map_res + map_origin[0], 2)
        contours_positive[index][1] = round(contours_positive[index][1]\
                                    * map_res + map_origin[1], 2)

    set_negative = set([tuple(x) for x in contours_negative])
    set_positive = set([tuple(x) for x in contours_positive])

    cand = set_negative.difference(set_positive)

    cand = [x for x in cand]

    cand = cluster(cand, cluster_trashhole)

    frontiers = []
    c = 0
    for i in range(len(cand)):
        if (len(cand[i])>min_cluster_size): 
            frontiers.append(np.array(cand[i]))
            c += 1

    print((i-c+1), " clusters not considered due to their size")

    return frontiers

def cluster(frontier_points, distance=1):

    U = UnionFind()

    for (i, x) in enumerate(frontier_points):
        for j in range(i + 1, len(frontier_points)):
            y = frontier_points[j]
            if max(abs(x[0] - y[0]), abs(x[1] - y[1])) <= distance:
                U.union(x, y)

    dSets = {}
    for x in frontier_points:
        s = dSets.get(U[x], set())
        s.add(x)
        dSets[U[x]] = s

    return [list(x) for x in dSets.values()]

def calculate_frontier_centers(list_of_arrays):

    global map_res

    frontier_centers = []
    for index in range(len(list_of_arrays)):

        length = list_of_arrays[index].shape[0]

        real_length = np.round(length * map_res * 100)

        x = statistics.median(list_of_arrays[index][:, 0])

        y = statistics.median(list_of_arrays[index][:, 1])

        frontier_centers.append([x, y, real_length])

    frontier_centers = np.array(frontier_centers)

    return frontier_centers

def goal_value(frontier_centers):

    value_array = np.zeros((frontier_centers.shape[0], frontier_centers.shape[1]))

    value_array = np.copy(frontier_centers)

    actual_position, current_quaternion = get_current_position('/map', '/odom')

    for index in range(len(frontier_centers)):

        dx = abs(actual_position[0] - frontier_centers[index][0])
        dy = abs(actual_position[1] - frontier_centers[index][1])
        man_dist = (dx + dy) + min(dx, dy)

        utility = frontier_centers[index][2] / man_dist

        value_array[index][2] = utility

    index = np.argsort(value_array[:, 2])
    value_array[:] = value_array[index]

    value_array = value_array[::-1]

    goals = []
    for i in range(3):
        coordanate = []

        if i < len(value_array):
            coordanate = [value_array[i][0], value_array[i][1]]
            goals.append(coordanate)

    return np.array(goals)

def get_current_position(target_frame, source_frame):
    position, quaternion = tflistener.lookupTransform(
                            target_frame, source_frame, rospy.Time(0))
    return position, quaternion

def callback_map(OccupancyGrid):
    global map_raw
    global map_origin
    global map_res

    info = OccupancyGrid.info
    data = OccupancyGrid.data

    map_raw = np.array(data)
    map_raw = map_raw.reshape(info.height, info.width)

    map_origin = np.array([info.origin.position.x, info.origin.position.y,\
                 info.origin.position.z])

    map_res = info.resolution

    rospy.sleep(1)

def move_to_point(x_target, y_target, theta_target = 0):
    """ Move to a location relative to the indicated frame """
    rospy.loginfo("navigating to: ({},{},{})".format(x_target, y_target, theta_target))

    quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = x_target
    goal.target_pose.pose.position.y = y_target

    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    move_base.send_goal(goal)

    success = move_base.wait_for_result(rospy.Duration(60))

    if not success:
        move_base.cancel_goal()

def terminate():

    rospy.loginfo("Stop TurtleBot")
    rospy.sleep(1)

def controller():
    while not rospy.is_shutdown():
        frontiers = calculate_frontiers() 
        frontier_centers = calculate_frontier_centers(frontiers) 
        print("Amount of frontiers found: ", len(frontier_centers))

        if (len(frontier_centers) == 0):
            print("No frontiers found, returning to start position and terminating search")
            move_to_point(init_position[0], init_position[1])
            terminate()

        goals = goal_value(frontier_centers) 

        move_to_point(goals[0][1], goals[0][0])

np.set_printoptions(suppress=True)

rospy.init_node('nav_node')

rospy.Subscriber('/map', OccupancyGrid, callback_map)
rospy.Subscriber('/odom', Odometry)

publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

tflistener = tf.TransformListener()

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
move_base.wait_for_server()
print("Server connection stablished")
rospy.loginfo("To stop TurtleBot CTRL + C")

rospy.on_shutdown(terminate)
rospy.sleep(1)

rviz_id = 0
graph_id = 0

cluster_trashhole = 0.1 
min_cluster_size = 10 

init_position, init_quat = get_current_position('/map', '/odom')

controller()
