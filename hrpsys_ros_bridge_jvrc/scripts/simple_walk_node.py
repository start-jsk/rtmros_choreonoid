#!/usr/bin/env python

import rospy, tf
import std_msgs.msg, jsk_footstep_msgs.msg, visualization_msgs.msg, geometry_msgs.msg, jsk_recognition_msgs.msg
import hrpsys_ros_bridge.srv, drc_task_common.srv, hrpsys_ros_bridge_jvrc.srv

class MultiWalkingClass(object):
    def __init__(self, node_name='multi_walking_node'):
        rospy.init_node(node_name)
        # param
        self.reference_frame_id = rospy.get_param("~reference_frame_id", "ground")
        self.target_frame_id = rospy.get_param("~target_frame_id", "robot_marker_root")
        # Publisher
        self.go_pos_footsteps_pub = rospy.Publisher('/go_pos_footsteps', jsk_footstep_msgs.msg.FootstepArray, queue_size=1)
        # subscriber
        rospy.Subscriber("/go_pos_command", std_msgs.msg.Empty, self.go_pos_callback)
        rospy.Subscriber("/urdf_control_marker/feedback", visualization_msgs.msg.InteractiveMarkerFeedback, self.marker_callback)
        # service proxy
        self.go_pos = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/goPos',
                                         hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_goPos)
        self.wait_foot_steps = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/waitFootSteps',
                                                  hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_waitFootSteps)
        self.get_abc_param = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/getAutoBalancerParam',
                                                hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_getAutoBalancerParam)
        self.get_go_pos_footsteps = rospy.ServiceProxy('/AutoBalancerServiceROSBridge/getGoPosFootstepsSequence',
                                                       hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_getGoPosFootstepsSequence)
        self.menu_call_srv = rospy.ServiceProxy('/rviz_menu_call', drc_task_common.srv.RvizMenuCall)
        self.fsm_call_srv = rospy.ServiceProxy('/call_gait_state_event', hrpsys_ros_bridge_jvrc.srv.StringRequest)
        # tf
        self.tfl = tf.TransformListener()
        # for marker_callback
        self.prev_event_type = 0
        # get vertices
        rospy.wait_for_service("/StabilizerServiceROSBridge/getParameter")
        self.st_param = rospy.ServiceProxy("/StabilizerServiceROSBridge/getParameter", hrpsys_ros_bridge.srv.OpenHRP_StabilizerService_getParameter)()
        # st_param.i_param.eefm_support_polygon_vertices_sequence[0].vertices[0].pos -> (0.182, 0.07112)

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

    def marker_callback(self, msg):
        cur_event_type = msg.event_type
        if self.prev_event_type == 1 and cur_event_type == 5:
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
            req = hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_getGoPosFootstepsSequenceRequest(x=x, y=y, th=yaw)
            res = self.get_go_pos_footsteps(req)
            if res.operation_return == True:
                fsll_msg = self._hrpsys_footsteps_to_jsk_footstep_msgs(res.o_footstep, msg.header.stamp)
                self.go_pos_footsteps_pub.publish(fsll_msg)
        self.prev_event_type = cur_event_type

    def _hrpsys_footsteps_to_jsk_footstep_msgs(self, hrpsys_footsteps, ts):
        fsll_msg = jsk_footstep_msgs.msg.FootstepArray()
        fsll_msg.header.stamp = ts
        fsll_msg.header.frame_id = "odom"
        for fsl in hrpsys_footsteps:
            for fs in fsl.fs:
                fsl_msg = jsk_footstep_msgs.msg.Footstep()
                if fs.leg == "rleg":
                    fsl_msg.leg = jsk_footstep_msgs.msg.Footstep.RLEG
                elif fs.leg == "lleg":
                    fsl_msg.leg = jsk_footstep_msgs.msg.Footstep.LLEG
                elif fs.leg == "rarm":
                    fsl_msg.leg = jsk_footstep_msgs.msg.Footstep.RARM
                elif fs.leg == "larm":
                    fsl_msg.leg = jsk_footstep_msgs.msg.Footstep.LARM
                fsl_msg.pose = geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=fs.pos[0], y=fs.pos[1], z=fs.pos[2]),
                                                      orientation=geometry_msgs.msg.Quaternion(w=fs.rot[0], x=fs.rot[1], y=fs.rot[2], z=fs.rot[3]))
                i_param = self.st_param.i_param
                fsl_msg.dimensions.x = i_param.eefm_leg_front_margin + i_param.eefm_leg_rear_margin
                fsl_msg.dimensions.y = i_param.eefm_leg_inside_margin + i_param.eefm_leg_outside_margin
                fsl_msg.dimensions.z = 0.05
                fsll_msg.footsteps.append(fsl_msg)
        return fsll_msg

    def main(self):
        # wait for service
        for i in ['/AutoBalancerServiceROSBridge/goPos',
                  '/AutoBalancerServiceROSBridge/waitFootSteps',
                  '/AutoBalancerServiceROSBridge/getAutoBalancerParam',
                  '/AutoBalancerServiceROSBridge/getGoPosFootstepsSequence',
                  '/rviz_menu_call',
                  '/call_gait_state_event']:
            rospy.wait_for_service(i)
        # start
        rospy.loginfo("[%s] start main loop", rospy.get_name())
        rospy.spin()

if __name__ == "__main__":
    ins = MultiWalkingClass()
    ins.main()
