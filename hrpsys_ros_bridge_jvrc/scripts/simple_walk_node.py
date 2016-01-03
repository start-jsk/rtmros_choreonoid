#!/usr/bin/env python

import rospy, tf
import std_msgs.msg
import hrpsys_ros_bridge.srv, drc_task_common.srv, hrpsys_ros_bridge_jvrc.srv

class MultiWalkingClass(object):
    def __init__(self, node_name='multi_walking_node'):
        rospy.init_node(node_name)
        # param
        self.reference_frame_id = rospy.get_param("~reference_frame_id", "ground")
        self.target_frame_id = rospy.get_param("~target_frame_id", "robot_marker_root")
        # subscriber
        rospy.Subscriber("/go_pos_command", std_msgs.msg.Empty, self.go_pos_callback)
        # service proxy
        self.go_pos = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/goPos',
                                         hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_goPos)
        self.wait_foot_steps = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/waitFootSteps',
                                                  hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_waitFootSteps)
        self.get_abc_param = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/getAutoBalancerParam',
                                                hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_getAutoBalancerParam)
        self.menu_call_srv = rospy.ServiceProxy('/rviz_menu_call', drc_task_common.srv.RvizMenuCall)
        self.fsm_call_srv = rospy.ServiceProxy('/call_gait_state_event', hrpsys_ros_bridge_jvrc.srv.StringRequest)
        # tf
        self.tfl = tf.TransformListener()

    def go_pos_callback(self, msg):
        ts = rospy.Time.now()
        try:
            self.tfl.waitForTransform(self.reference_frame_id, self.target_frame_id, ts, rospy.Duration(1))
            (trans, rot) = self.tfl.lookupTransform(self.reference_frame_id, self.target_frame_id, ts)
        except:
            rospy.logwarn("[%s] failed to solve tf from %s to %s", rospy.get_name(), self.reference_frame_id, self.target_frame_id)
            return
        x = trans[0]
        y = trans[1]
        yaw = tf.transformations.math.degrees(tf.transformations.euler_from_quaternion(rot)[2])
        gait_type = self.get_abc_param().i_param.default_gait_type

        title = "Go pos to (" + ("%4.3f, " % x) + ("%4.3f, " % y) + ("%4.3f) " % yaw) + ("in %s " % ['biped', 'trot', 'pace'][gait_type])  + "OK? "
        menu_list = ["cancel", "yes"]
        rviz_req = drc_task_common.srv.RvizMenuCallRequest(title = title,
                                                           menu_list = menu_list)
        res = self.menu_call_srv(rviz_req)
        if menu_list[res.index] == "yes":
            # check fsm
            first_event  = 'biped walk' if gait_type == 0 else 'quadruped walk'
            second_event = 'biped stop' if gait_type == 0 else 'quadruped stop'
            first_res = self.fsm_call_srv(hrpsys_ros_bridge_jvrc.srv.StringRequestRequest(first_event))
            if first_res.result == False:
                return
            req = hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_goPosRequest(x=x, y=y, th=yaw)
            self.go_pos(req)
            rospy.loginfo("[%s] start go pos %s %s %s and waiting...", rospy.get_name(), x, y, yaw)
            self.wait_foot_steps()
            # check fsm
            self.fsm_call_srv(hrpsys_ros_bridge_jvrc.srv.StringRequestRequest(second_event))
            rospy.loginfo("[%s] finish go pos %s %s %s", rospy.get_name(), x, y, yaw)

    def main(self):
        # wait for service
        for i in ['/AutoBalancerServiceROSBridge/goPos',
                  '/AutoBalancerServiceROSBridge/waitFootSteps',
                  '/rviz_menu_call',
                  '/call_gait_state_event']:
            rospy.wait_for_service(i)
        # start
        rospy.loginfo("[%s] start main loop", rospy.get_name())
        rospy.spin()

if __name__ == "__main__":
    ins = MultiWalkingClass()
    ins.main()
