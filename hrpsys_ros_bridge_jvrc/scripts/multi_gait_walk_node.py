#!/usr/bin/env python

import rospy, tf
import std_msgs.msg, jsk_footstep_msgs.msg, visualization_msgs.msg, geometry_msgs.msg, jsk_recognition_msgs.msg, hrpsys_ros_bridge_jvrc.msg
import hrpsys_ros_bridge.srv, drc_task_common.srv, hrpsys_ros_bridge_jvrc.srv

class MultiGaitWalkingClass(object):
    def __init__(self, node_name='multi_gait_walking_node'):
        rospy.init_node(node_name)
        # subscriber
        rospy.Subscriber("/multi_go_pos_order", hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPosArray, self.multi_go_pos_order_callback)
        rospy.Subscriber("/multi_gait_go_pos_command", std_msgs.msg.Empty, self.multi_gait_go_pos_callback)
        # service proxy
        self.go_pos = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/goPos',
                                         hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_goPos)
        self.wait_foot_steps = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/waitFootSteps',
                                                  hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_waitFootSteps)
        self.move_quadruped = rospy.ServiceProxy('/move_quadruped',
                                                 hrpsys_ros_bridge_jvrc.srv.StringRequest)
        self.move_biped = rospy.ServiceProxy('/move_biped',
                                             hrpsys_ros_bridge_jvrc.srv.StringRequest)
        self.get_abc_param = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/getAutoBalancerParam',
                                                hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_getAutoBalancerParam)
        self.fsm_call_srv = rospy.ServiceProxy('/call_gait_state_event', hrpsys_ros_bridge_jvrc.srv.StringRequest)
        # keep order
        self.multi_go_pos_order = None

    def multi_go_pos_order_callback(self, msg):
        self.multi_go_pos_order = msg
        rospy.logwarn("[%s] I've got order", rospy.get_name())

    def multi_gait_go_pos_callback(self, msg):
        if self.multi_go_pos_order == None:
            rospy.logwarn("[%s] I do not have any order", rospy.get_name())
            return
        for idx, order in enumerate(self.multi_go_pos_order.orders):
            gait_type = self.get_abc_param().i_param.default_gait_type
            first_event  = 'biped walk' if gait_type == 0 else 'quadruped walk'
            second_event = 'biped stop' if gait_type == 0 else 'quadruped stop'
            first_res = self.fsm_call_srv(hrpsys_ros_bridge_jvrc.srv.StringRequestRequest(first_event))
            if first_res.result == False:
                return
            req = hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_goPosRequest(x=order.x, y=order.y, th=order.th)
            self.go_pos(req)
            rospy.loginfo("[%s] start go pos %s %s %s and waiting...", rospy.get_name(), order.x, order.y, order.th)
            self.wait_foot_steps()
            # check fsm
            self.fsm_call_srv(hrpsys_ros_bridge_jvrc.srv.StringRequestRequest(second_event))
            rospy.loginfo("[%s] finish go pos %s %s %s", rospy.get_name(), order.x, order.y, order.th)
            if idx != len(self.multi_go_pos_order.orders) - 1:
                if gait_type == 0: # now biped
                    trans_res = self.move_quadruped(hrpsys_ros_bridge_jvrc.srv.StringRequestRequest(""))
                    if trans_res == False:
                        return
                elif gait_type == 1: # now quadruped
                    trans_res = self.move_biped(hrpsys_ros_bridge_jvrc.srv.StringRequestRequest(""))
                    if trans_res == False:
                        return
            else:
                self.multi_go_pos_order = None

    def main(self):
        # wait for service
        for i in ['/AutoBalancerServiceROSBridge/goPos',
                  '/AutoBalancerServiceROSBridge/waitFootSteps',
                  '/move_quadruped',
                  '/move_biped']:
            rospy.wait_for_service(i)
        # start
        rospy.loginfo("[%s] start main loop", rospy.get_name())
        rospy.spin()

if __name__ == "__main__":
    ins = MultiGaitWalkingClass()
    ins.main()
