#!/usr/bin/env python

import rospy, tf
import std_msgs.msg, jsk_footstep_msgs.msg, visualization_msgs.msg, geometry_msgs.msg, jsk_recognition_msgs.msg, sensor_msgs.msg
import hrpsys_ros_bridge.srv, drc_task_common.srv, hrpsys_ros_bridge_jvrc.srv

class GaitPlannerClass(object):
    def __init__(self, node_name='gait_planner_node'):
        rospy.init_node(node_name)
        # Publisher
        self.go_pos_footsteps_bb_pub = rospy.Publisher('/go_pos_footsteps_boundingbox', jsk_recognition_msgs.msg.BoundingBoxArray, queue_size=1)
        # subscriber
        rospy.Subscriber('/go_pos_footsteps', jsk_footstep_msgs.msg.FootstepArray, self.footstep_callback)
        rospy.Subscriber('/latest_complete_heightmap/output', sensor_msgs.msg.Image, self.footstep_callback)
        self.fsm_call_srv = rospy.ServiceProxy('/call_gait_state_event', hrpsys_ros_bridge_jvrc.srv.StringRequest)
        rospy.wait_for_service("/StabilizerServiceROSBridge/getParameter")
        self.st_param = rospy.ServiceProxy("/StabilizerServiceROSBridge/getParameter", hrpsys_ros_bridge.srv.OpenHRP_StabilizerService_getParameter)()

    def footstep_callback(self, msg):
        bb_msg = self._jsk_footstep_msgs_to_bounding_box_array(msg)
        self.go_pos_footsteps_bb_pub.publish(bb_msg)

    def _jsk_footstep_msgs_to_bounding_box_array(self, fsl):
        box_array = jsk_recognition_msgs.msg.BoundingBoxArray()
        box_array.header = fsl.header
        for fs in fsl.footsteps:
            box = jsk_recognition_msgs.msg.BoundingBox()
            box.header = fsl.header
            box.pose = fs.pose
            box.dimensions  = fs.dimensions
            # expand region
            box.dimensions.x += 0.2
            box.dimensions.y += 0.2
            box.dimensions.z += 0.5
            # fix position because bounding box origin is the center of bounding box and x / y /z is edge length
            i_param = self.st_param.i_param
            box.pose.position.x += (i_param.eefm_leg_front_margin - i_param.eefm_leg_rear_margin) / 2.0
            box.pose.position.y += (i_param.eefm_leg_inside_margin - i_param.eefm_leg_outside_margin) / 2.0
            box_array.boxes.append(box)
        return box_array

    def main(self):
        # wait for service
        for i in ['/call_gait_state_event']:
            rospy.wait_for_service(i)
        # start
        rospy.loginfo("[%s] start main loop", rospy.get_name())
        rospy.spin()

if __name__ == "__main__":
    ins = GaitPlannerClass()
    ins.main()
