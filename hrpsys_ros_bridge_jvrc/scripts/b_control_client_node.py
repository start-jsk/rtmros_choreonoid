#!/usr/bin/env python

import rospy, imp
imp.find_module('jsk_teleop_joy')
from jsk_teleop_joy.b_control_status import BControl2Status
import std_msgs.msg, sensor_msgs.msg, geometry_msgs.msg
import std_srvs.srv, drc_task_common.srv

class BControlClient():
    def __init__(self):
        rospy.init_node('b_control_client')
        self.prev_status = False
        self.status = False
        # subscriber
        rospy.Subscriber('input_joy', sensor_msgs.msg.Joy, self.b_control_joy_cb)
        # publisher
        self.go_pos_pub = rospy.Publisher('/go_pos_command', std_msgs.msg.Empty, queue_size=1)
        self.menu_variable_pub = rospy.Publisher('/rviz_menu_variable', std_msgs.msg.Float32, queue_size=1)
        self.set_robot_pose_pub = rospy.Publisher('/set_robot_pose', std_msgs.msg.Empty, queue_size=1)
        self.set_robot_pose_with_av_pub = rospy.Publisher('/set_robot_pose_with_av', std_msgs.msg.Empty, queue_size=1)
        self.robot_menu_pub = rospy.Publisher('/robot_menu_command', std_msgs.msg.Empty, queue_size=1)
        self.send_angle_pub = rospy.Publisher('/send_angle_command', std_msgs.msg.Empty, queue_size=1)
        self.gait_menu_pub = rospy.Publisher('/gait_menu_command', std_msgs.msg.Empty, queue_size=1)
        # service proxy
        self.menu_up_srv = rospy.ServiceProxy('/rviz_menu_up', std_srvs.srv.Empty)
        self.menu_down_srv = rospy.ServiceProxy('/rviz_menu_up', std_srvs.srv.Empty)
        self.menu_select_srv = rospy.ServiceProxy('/rviz_menu_select', drc_task_common.srv.RvizMenuSelect)
        self.menu_cancel_srv = rospy.ServiceProxy('/rviz_menu_cancel', std_srvs.srv.Empty)

    def b_control_joy_cb(self, msg):
        self.prev_status = self.status
        self.status = BControl2Status(msg)
        if not(self.prev_status):
            # we have to push button twice first time because of this
            self.prev_status = self.status
        # robot
        if self.status.buttonU1 != self.prev_status.buttonU1:
            self.gait_menu_pub.publish()
        if self.status.buttonU4 != self.prev_status.buttonU4:
            self.set_robot_pose_pub.publish()
        if self.status.buttonU5 != self.prev_status.buttonU5:
            self.go_pos_pub.publish()
        if self.status.buttonL5 != self.prev_status.buttonL5:
            self.send_angle_pub.publish()
        if self.status.buttonL6 != self.prev_status.buttonL6:
            self.robot_menu_pub.publish()
        if self.status.buttonL1 != self.prev_status.buttonL1:
            self.set_robot_pose_with_av_pub.publish()
        # rviz menu
        if self.status.buttonU8 != self.prev_status.buttonU8:
            self.menu_down_srv()
        if self.status.buttonL8 != self.prev_status.buttonL8:
            self.menu_select_srv(variable = self.status.slide8)
        menu_v = std_msgs.msg.Float32(self.status.slide8)
        self.menu_variable_pub.publish(menu_v)

    def main(self):
        # wait for service
        for i in ['/rviz_menu_up', '/rviz_menu_select', '/rviz_menu_cancel']:
            rospy.wait_for_service(i)
        # start
        rospy.loginfo("[%s] start main loop", rospy.get_name())
        rospy.spin()

if __name__ == '__main__':
    ins = BControlClient()
    ins.main()
