This file launches two robots within a world where we send the robots to goal poses via <code>move_base</code>.

To launch the environment after <code>catkin_make</code>: <code>roslaunch sample_nav mission.launch</code>. 
Now, to run the script that sends the robot to goal poses: <code>python ~/catkin_ws/src/sample_nav/scripts/sample.py</code>.
Note that in this exercise, we have three objectives:
1. Request from <code>nav_msgs/GetPlan</code> service to plan a path from start to goal pose without actually moving the robot.
2. Send a robot to goal pose via action client, <code>/robot_<robot_id>/move_base/goal</code>.
3. Send a robot to goal pose by publishing to <code>/robot_<robot_id>/move_base_simple/goal</code>.