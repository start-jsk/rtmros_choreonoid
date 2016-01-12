#!/usr/bin/env python

import rospy, tf, cv_bridge
import cv2, numpy
import std_msgs.msg, jsk_footstep_msgs.msg, visualization_msgs.msg, geometry_msgs.msg, jsk_recognition_msgs.msg, sensor_msgs.msg, hrpsys_ros_bridge_jvrc.msg
import hrpsys_ros_bridge.srv, drc_task_common.srv, hrpsys_ros_bridge_jvrc.srv

class GaitPlannerClass(object):
    def __init__(self, node_name='gait_planner_node'):
        rospy.init_node(node_name)
        # Publisher
        self.go_pos_footsteps_bb_pub = rospy.Publisher('/go_pos_footsteps_boundingbox', jsk_recognition_msgs.msg.BoundingBoxArray, queue_size=1)
        self.masked_heightmap_pub = rospy.Publisher('/masked_heightmap', sensor_msgs.msg.Image, queue_size=1)
        self.multi_go_pos_order_pub = rospy.Publisher('/multi_go_pos_order', hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPosArray, queue_size=1)
        # subscriber
        rospy.Subscriber('/go_pos_footsteps', jsk_footstep_msgs.msg.FootstepArray, self.footstep_callback)
        rospy.Subscriber('~input/heightmap', sensor_msgs.msg.Image, self.heightmap_callback)
        ### https://github.com/jsk-ros-pkg/jsk_visualization/pull/539
        rospy.Subscriber('~input/contact_plane_candidate', geometry_msgs.msg.PolygonStamped, self.contact_plane_candidate_callback)
        self.fsm_call_srv = rospy.ServiceProxy('/call_gait_state_event', hrpsys_ros_bridge_jvrc.srv.StringRequest)
        rospy.wait_for_service("/StabilizerServiceROSBridge/getParameter")
        self.st_param = rospy.ServiceProxy("/StabilizerServiceROSBridge/getParameter", hrpsys_ros_bridge.srv.OpenHRP_StabilizerService_getParameter)()
        rospy.wait_for_service("/AutoBalancerServiceROSBridge/getGaitGeneratorParam")
        self.get_gg_param = rospy.ServiceProxy("/AutoBalancerServiceROSBridge/getGaitGeneratorParam", hrpsys_ros_bridge.srv.OpenHRP_AutoBalancerService_getGaitGeneratorParam)
        self.gg_param = None
        # tf
        self.tfl = tf.TransformListener()
        # store heightmap
        self.bridge = cv_bridge.CvBridge()
        self.my_heightmap_header = None
        self.my_heightmap = None
        # store contact plane candidate
        self.contact_plane_candidate = None

    def footstep_callback(self, msg):
        bb_msg = self._jsk_footstep_msgs_to_bounding_box_array(msg)
        self.go_pos_footsteps_bb_pub.publish(bb_msg)
        if self.my_heightmap == None:
            rospy.logwarn("[%s] my_heightmap : %s", rospy.get_name(), (self.my_heightmap != None))
            return
        else:
            transformed_bb_msg = self._tf_transform_bounding_box_array(bb_msg)
            vertices_list = self._vertices_of_bounding_box_array(transformed_bb_msg)
            x_min = rospy.get_param("/latest_heightmap/min_x")
            x_max = rospy.get_param("/latest_heightmap/max_x")
            y_min = rospy.get_param("/latest_heightmap/min_y")
            y_max = rospy.get_param("/latest_heightmap/max_y")
            x_pix = rospy.get_param("/latest_heightmap/resolution_x")
            y_pix = rospy.get_param("/latest_heightmap/resolution_y")
            vertices_list_in_pixel = self._meter_to_pixel(vertices_list, x_min, x_max, y_min, y_max, x_pix, y_pix)
            # generate mask image
            thre, occlusion_mask_image = cv2.threshold(self.my_heightmap, -1e+3, 255, cv2.THRESH_BINARY)
            occlusion_mask_image = numpy.uint8(occlusion_mask_image)
            # calc MultiGoPos order
            self.gg_param = self.get_gg_param()
            origin = self._calc_reference_pose(msg.footsteps[0])
            prev_origin = self._calc_reference_pose(msg.footsteps[0])
            gait_type = hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPos.BIPED
            mgg_array = hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPosArray(header=msg.header)
            for idx, (vs, fs) in enumerate(zip(vertices_list_in_pixel, msg.footsteps)):
                footstep_mask_image = numpy.zeros((y_pix, x_pix, 1), numpy.uint8)
                cv2.fillConvexPoly(footstep_mask_image, numpy.int32(numpy.array([vs[0], vs[1], vs[2], vs[3]])), 255)
                mask_image = cv2.bitwise_and(occlusion_mask_image, occlusion_mask_image, mask=footstep_mask_image)
                rospy.logwarn("minMaxLoc : %s", cv2.minMaxLoc(self.my_heightmap, mask=mask_image))
                rospy.logwarn("meanStdDev : %s", cv2.meanStdDev(self.my_heightmap, mask=mask_image))
                mean, stddev = cv2.meanStdDev(self.my_heightmap, mask=mask_image)
                if stddev[0,0] > 0.3 and gait_type == hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPos.BIPED:
                    mgg_msg = self._get_multi_gait_go_pos(origin, prev_origin)
                    mgg_msg.gait_type = gait_type
                    mgg_array.orders.append(mgg_msg)
                    origin = prev_origin
                    gait_type = hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPos.TROT
                elif stddev[0,0] < 0.3 and gait_type == hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPos.TROT:
                    new_origin = self._calc_reference_pose(fs)
                    mgg_msg = self._get_multi_gait_go_pos(origin, new_origin)
                    mgg_msg.gait_type = gait_type
                    mgg_array.orders.append(mgg_msg)
                    origin = new_origin
                    gait_type = hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPos.BIPED
                if idx == len(msg.footsteps) - 1: # final step
                    fin_origin = self._calc_reference_pose(fs)
                    mgg_msg = self._get_multi_gait_go_pos(origin, fin_origin)
                    mgg_msg.gait_type = gait_type
                    mgg_array.orders.append(mgg_msg)
                prev_origin = self._calc_reference_pose(fs) # keep the previous foot step
            self.multi_go_pos_order_pub.publish(mgg_array)

    def heightmap_callback(self, msg):
        self.my_heightmap_header = msg.header
        self.my_heightmap = self.bridge.imgmsg_to_cv2(msg) # second argument should be not specified

    def contact_plane_candidate_callback(self, msg):
        self.contact_plane_candidate = msg

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


    def _tf_transform_bounding_box_array(self, bb_array):
        if self.tfl.frameExists(bb_array.header.frame_id) and self.tfl.frameExists(self.my_heightmap_header.frame_id):
            for box in bb_array.boxes:
                tmp_pose_stamped = geometry_msgs.msg.PoseStamped(header=box.header, pose=box.pose)
                new_pose_stamped = self.tfl.transformPose(self.my_heightmap_header.frame_id, tmp_pose_stamped)
                box.pose = new_pose_stamped.pose
            return bb_array
        else:
            rospy.logwarn("[%s] failed to solve tf from %s (%s) to %s (%s)", rospy.get_name(), bb_array.header.frame_id, self.tfl.frameExists(bb_array.header.frame_id), self.my_heightmap_header, self.tfl.frameExists(self.my_heightmap_header.frame_id))
            return

    def _vertices_of_bounding_box_array(self, bb_array):
        ret = []
        for box in bb_array.boxes:
            position = box.pose.position
            orientation = box.pose.orientation
            dimensions = box.dimensions
            mat = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
            directions = [numpy.array([+1*dimensions.x/2, +1*dimensions.y/2, 0, 0]),
                          numpy.array([+1*dimensions.x/2, -1*dimensions.y/2, 0, 0]),
                          numpy.array([-1*dimensions.x/2, -1*dimensions.y/2, 0, 0]),
                          numpy.array([-1*dimensions.x/2, +1*dimensions.y/2, 0, 0])]
            vertices = [numpy.dot(mat, d)[:2] + numpy.array([position.x, position.y]) for d in directions]
            ret.append(vertices)
        # rospy.logwarn("%s, %s, %s, %s",
        #               [int(x) for x in ret[0][0]*1e3],
        #               [int(x) for x in ret[0][1]*1e3],
        #               [int(x) for x in ret[0][2]*1e3],
        #               [int(x) for x in ret[0][3]*1e3])
        return ret

    def _meter_to_pixel(self, vertices_list, x_min, x_max, y_min, y_max, x_pix, y_pix):
        for vs in vertices_list:
            for v in vs:
                v[0] = int((v[0] - x_min) / (x_max - x_min) * x_pix)
                v[1] = int((v[1] - y_min) / (y_max - y_min) * y_pix)
        return vertices_list

    def _calc_reference_pose(self, fs):
        ret = geometry_msgs.msg.Pose()
        position = fs.pose.position
        orientation = fs.pose.orientation
        leg = fs.leg
        if leg == jsk_footstep_msgs.msg.Footstep.RLEG:
            offset = self.gg_param.i_param.leg_default_translate_pos.data[0*3+1]
        elif leg == jsk_footstep_msgs.msg.Footstep.LLEG:
            offset = self.gg_param.i_param.leg_default_translate_pos.data[1*3+1]
        elif leg == jsk_footstep_msgs.msg.Footstep.RARM:
            offset = self.gg_param.i_param.leg_default_translate_pos.data[2*3+1]
        elif leg == jsk_footstep_msgs.msg.Footstep.LARM:
            offset = self.gg_param.i_param.leg_default_translate_pos.data[3*3+1]
        mat = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        new_position = numpy.dot(mat, numpy.array([0, -1 * offset, 0, 0]))[:3] + numpy.array([position.x, position.y, position.z])
        ret.position = geometry_msgs.msg.Point(x=new_position[0], y=new_position[1], z=new_position[2])
        ret.orientation = orientation
        return ret

    def _get_multi_gait_go_pos(self, start, goal):
        mgg_msg = hrpsys_ros_bridge_jvrc.msg.MultiGaitGoPos()
        diff_p = numpy.array([goal.position.x - start.position.x,
                              goal.position.y - start.position.y,
                              goal.position.z - start.position.z,
                              0.0]) # dummy
        start_mat = tf.transformations.quaternion_matrix([start.orientation.x,
                                                          start.orientation.y,
                                                          start.orientation.z,
                                                          start.orientation.w])
        goal_mat = tf.transformations.quaternion_matrix([goal.orientation.x,
                                                         goal.orientation.y,
                                                         goal.orientation.z,
                                                         goal.orientation.w])
        yaw = tf.transformations.euler_from_matrix(numpy.dot(start_mat.transpose(), goal_mat))[2]
        mgg_msg.x = numpy.dot(start_mat, diff_p)[0]
        mgg_msg.y = numpy.dot(start_mat, diff_p)[1]
        mgg_msg.th = tf.transformations.math.degrees(yaw)
        return mgg_msg


    def _heat_color(self, v):
        ratio = 2 * v
        r = int(max(0.0, 255 * (ratio - 1.0)))
        b = int(max(0, 255 * (1.0 - ratio)))
        g = 255 - b - r
        return [x / 255.0 for x in [r, g, b]]

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
