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
import hrpsys_ros_bridge.srv
import hrpsys_ros_bridge.msg

initial_position = None

def callback (msg):
    global initial_position
    idx = msg.name.index('motor_joint')
    initial_position = msg.position[idx]

if __name__ == '__main__':
    rospy.init_node('rotate_range', anonymous=False)
    ### params
    try:
        rotate_cycle = rospy.get_param('~rotate_cycle')
    except KeyError:
        rotate_cycle = 4.0
    try:
        rotate_times = rospy.get_param('~rotate_times')
    except KeyError:
        rotate_times = 4

    ### service
    rospy.loginfo('wait for service /SequencePlayerServiceROSBridge/setInterpolationMode')
    rospy.wait_for_service('/SequencePlayerServiceROSBridge/setInterpolationMode')
    try:
        sprox = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/setInterpolationMode',
                                   hrpsys_ros_bridge.srv.OpenHRP_SequencePlayerService_setInterpolationMode)
        res = sprox(hrpsys_ros_bridge.msg.OpenHRP_SequencePlayerService_interpolationMode.LINEAR)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    ### action
    rangeAct = actionlib.SimpleActionClient('/range_controller/follow_joint_trajectory_action',
                                            FollowJointTrajectoryAction)
    rospy.loginfo("wait for server /range_controller/follow_joint_trajectory_action");
    rangeAct.wait_for_server()
    rospy.loginfo("server found");

    ### read initial position
    sub = rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, callback)
    rospy.loginfo("wait initial_position");
    while not initial_position:
        rospy.sleep(0.01)
    rospy.loginfo("initial_position: %f"%(initial_position));
    sub.unregister()

    tm = rotate_cycle ## parameter for tm

    rate = rospy.Rate(1.0/(rotate_times*tm)) # 4 round
    range_goal = FollowJointTrajectoryGoal()
    range_goal.trajectory.joint_names.append("motor_joint")

    rangeAngle = initial_position + 2*rotate_times*math.pi
    while not rospy.is_shutdown():
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [ rangeAngle ]
        point.time_from_start = rospy.Duration(tm*rotate_times)
        range_goal.trajectory.points = [ point ]
        rangeAct.send_goal(range_goal)
        rangeAngle = rangeAngle + 2*rotate_times*math.pi
        rate.sleep()
