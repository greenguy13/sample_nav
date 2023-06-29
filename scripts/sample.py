#!/usr/bin/env python

import math
import numpy as np
import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

SUCCEEDED = 3 #GoalStatus ID for succeeded, http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html

class Navigator:
    def __init__(self, nrobots, areas, tolerance):
        print("we got inside")

        #Service client to GetPlan
        self.get_plan_services = dict()
        for robot_id in range(nrobots):
            server = '/robot_'+str(robot_id)+'/move_base_node/make_plan'
            rospy.wait_for_service(server)
            self.get_plan_services['robot'+str(robot_id)] = rospy.ServiceProxy(server, GetPlan)

        #Send robot to goal
        self.robot_goal_clients = dict()
        for robot_id in range(nrobots):
            self.robot_goal_clients['robot'+str(robot_id)] = actionlib.SimpleActionClient('/robot_'+str(robot_id)+'/move_base', MoveBaseAction)
            self.robot_goal_clients['robot'+str(robot_id)].wait_for_server()

        self.tolerance = tolerance
        self.areas = areas #list of areas
        self.dist_matrix = np.zeros((len(self.areas), len(self.areas)))
        self.available = True
        self.on_mission = False

    #METHODS: Build distance matrix
    def get_plan_request(self, robot_id, start_pose, goal_pose, tolerance):
        """
        Sends a request to GetPlan service to create a plan for path from start to goal without actually moving the robot
        :param start_pose:
        :param goal_pose:
        :param tolerance:
        :return:
        """
        req = GetPlan()
        req.start = start_pose
        req.goal = goal_pose
        req.tolerance = tolerance
        server = self.get_plan_services['robot'+str(robot_id)]
        result = server(req.start, req.goal, req.tolerance)
        path = result.plan.poses
        return path

    def decouple_path_poses(self, path):
        """
        Decouples a path of PoseStamped poses; returning a list of x,y poses
        :param path: list of PoseStamped
        :return:
        """
        list_poses = list()
        for p in path:
            x, y = p.pose.position.x, p.pose.position.y
            list_poses.append((x, y))
        return list_poses

    def compute_path_total_dist(self, list_poses):
        """
        Computes the total path distance
        :return:
        """
        total_dist = 0
        for i in range(len(list_poses)-1):
            dist = math.dist(list_poses[i], list_poses[i+1])
            total_dist += dist
        return total_dist

    def compute_dist_bet_areas(self, robot_id, area_i, area_j, tolerance):
        """
        Computes the distance between area_i and area_j:
            1. Call the path planner between area_i and area_j
            2. Decouple the elements of path planning
            3. Compute the distance then total distance
        :param area_i: PoseStamped
        :param area_j: PoseStamped
        :return:
        """
        path = self.get_plan_request(robot_id, area_i, area_j, tolerance)
        list_poses = self.decouple_path_poses(path)
        total_dist = self.compute_path_total_dist(list_poses)
        return total_dist

    def build_dist_matrix(self, robot_id):
        """
        Builds the distance matrix among areas
        :return:
        """
        for i in range(len(self.areas)):
            for j in range(len(self.areas)):
                area_i, area_j = self.areas[i], self.areas[j]
                if area_i != area_j:
                    dist = self.compute_dist_bet_areas(robot_id, area_i, area_j, self.tolerance)
                    self.dist_matrix[i, j] = dist

        print("Dist matrix:", self.dist_matrix)

    #METHODS: Send robot to area
    def send_robot_goal(self, robot_id, goal):
        """
        Sends robot to goal via action client
        :param robot:
        :param goal: PoseStamped object
        :return:
        """
        client = self.robot_goal_clients['robot'+str(robot_id)]

        movebase_goal = MoveBaseGoal()
        movebase_goal.target_pose = goal
        print("Movebase goal:", movebase_goal)
        self.on_mission = True
        action_goal_cb = (lambda state, result : self.action_send_done_cb(state, result, robot_id))
        client.send_goal(movebase_goal, done_cb=action_goal_cb, active_cb=self.action_send_active_cb)

    def action_send_active_cb(self):
        """
        Sets robot as unavailable when pursuing goal
        :return:
        """

        if self.on_mission:
            self.available = False
        print("Avail:", self.available)

    def action_send_done_cb(self, state, result, robot_id):
        """

        :param msg:
        :return:
        """
        #Note: Currently there is a tiny bug on the lambda
        print("Robot id: {}. Succeeded: {}".format(robot_id, state==SUCCEEDED))
        if state == SUCCEEDED:
            self.on_mission = False
            self.available = True
        print("Avail:", self.available)


    def run_operation(self, robot_id):
        area = 0
        while not rospy.is_shutdown():
            #Build the distance matrix
            if np.sum(self.dist_matrix) == 0:
                self.build_dist_matrix(robot_id)

            #Send the robot to all the areas
            if self.available:
                self.send_robot_goal(robot_id, self.areas[area])
                area += 1
            if area>=len(self.areas):
                area = 0
            rospy.sleep(1)


if __name__== '__main__':
    rospy.init_node('sample_nav')

    area1 = PoseStamped()
    area1.header.seq = 0
    area1.header.frame_id = "map"
    area1.header.stamp = rospy.Time.now()
    area1.pose.position.x = 0.0
    area1.pose.position.y = 0.0
    area1.pose.orientation.w = 1.0

    area2 = PoseStamped()
    area2.header.seq = 0
    area2.header.frame_id = "map"
    area2.header.stamp = rospy.Time.now()
    area2.pose.position.x = 2.0
    area2.pose.position.y = 2.0
    area2.pose.orientation.w = 1.0

    area3 = PoseStamped()
    area3.header.seq = 0
    area3.header.frame_id = "map"
    area3.header.stamp = rospy.Time.now()
    area3.pose.position.x = -2.0
    area3.pose.position.y = -2.0
    area3.pose.orientation.w = 1.0

    areas = [area1, area2, area3]
    tolerance = 0.5
    navigator = Navigator(2, areas, tolerance)
    navigator.run_operation(robot_id=0)