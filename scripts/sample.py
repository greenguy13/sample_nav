#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class Navigator:
    def __init__(self, node, nrobots):
        rospy.init_node(node)
        print("we got inside")

        #Service client to GetPlan
        #Interesting: For multi-robot, shouldn't this be robot-specific; i.e., either robot 0 or 1
        #But if we rossrv list there is only nav_msgs/GetPlan
        rospy.wait_for_service('nav_msgs/GetPlan')
        self.get_plan = rospy.ServiceProxy('nav_msgs/GetPlan', GetPlan)

        #Send robot to goal
        #Option 1: Action client to move base
        #Robot 0 or 1 for nrobots=2
        self.robot_goal_clients = dict()
        for robot_id in range(nrobots):
            self.robot_goal_clients['robot'+str(robot_id)] = actionlib.SimpleActionClient('/robot_'+str(robot_id)+'/move_base/goal', MoveBaseAction)
            self.robot_goal_clients['robot'+str(robot_id)].wait_for_server()

        #Option 2: Subscribed topic
        self.robot_goal_topics = dict()
        for robot_id in range(nrobots):
            self.robot_goal_topics['robot'+str(robot_id)] = rospy.Publisher('/robot_'+str(robot_id)+'/move_base_simple/goal', PoseStamped, queue_size=1)

    def get_plan_cb(self, start_pose, goal_pose, tolerance):
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
        resp = self.get_plan(req.start, req.goal, req.tolerance)
        return resp

    def send_robot_goal_cb(self, robot_id, goal):
        """
        Sends robot to goal via action client
        :param robot:
        :param goal: PoseStamped object
        :return:
        """
        client = self.robot_goal_clients['robot'+str(robot_id)]

        movebase_goal = MoveBaseGoal()
        movebase_goal.target_pose = goal
        client.send_goal(movebase_goal)
        client.wait_for_result()
        result = client.get_result()
        return result

    def pub_robot_goal_cb(self, robot_id, goal):
        """
        Publishes a goal for robot to go to
        :param robot_id:
        :param goal: PoseStamped
        :return:
        """
        self.robot_goal_topics['robot'+str(robot_id)].publish(goal)


    def spin(self):

        while not rospy.is_shutdown():

            rospy.sleep(1)


if __name__== '__main__':
    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0

    goal = PoseStamped()
    goal.header.seq = 0
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time(0)
    goal.pose.position.x = 2.0
    goal.pose.position.y = 2.0

    navigator = Navigator('sample_nav', 2)

    #Subscribe to GetPlan service
    #Uncomment below
    # tolerance = 0.5
    # plan = navigator.get_plan_cb(start, goal, tolerance)
    # print(plan)

    #Action client to move_base: Send robot to goal
    #Uncommment below
    # navigator.send_robot_goal_cb(0, goal)

    #Publish topic to send robot to goal
    navigator.pub_robot_goal_cb(0, goal)

    navigator.spin()
