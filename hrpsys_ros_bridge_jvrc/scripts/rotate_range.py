#!/usr/bin/env python

import roslib
import rospy
import actionlib
import math
# import tf

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
import trajectory_msgs.msg
import sensor_msgs.msg

initial_position = None

def callback (msg):
    global initial_position
    idx = msg.name.index('RANGE_JOINT')
    initial_position = msg.position[idx]

if __name__ == '__main__':
    rospy.init_node('testrange', anonymous=False)

    rangeAct = actionlib.SimpleActionClient("/range_controller/follow_joint_trajectory_action",
                                            FollowJointTrajectoryAction)
    rospy.loginfo("wait for server");
    rangeAct.wait_for_server()
    rospy.loginfo("server found");

    rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, callback)

    rospy.loginfo("wait initialposition");
    while not initial_position:
        rospy.sleep(0.01)
    rospy.loginfo("initialposition: %f"%(initial_position));

    tm = 5.0 ## parameter for tm

    rate = rospy.Rate(1/(4*tm)) # 4 round
    range_goal = FollowJointTrajectoryGoal()
    range_goal.trajectory.joint_names.append("RANGE_JOINT")

    rangeAngle = initial_position + 8*math.pi
    while not rospy.is_shutdown():
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [ rangeAngle ]
        point.time_from_start = rospy.Duration(tm*4.05)
        range_goal.trajectory.points = [ point ]
        rangeAct.send_goal(range_goal)
        rangeAngle = rangeAngle + 8*math.pi
        rate.sleep()
