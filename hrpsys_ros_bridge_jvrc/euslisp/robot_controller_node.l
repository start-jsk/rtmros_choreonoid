#!/usr/bin/env roseus

(ros::roseus "robot_controller")
(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://hrpsys_ros_bridge_jvrc/euslisp/gait_controller.l")

(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-srvs "hrpsys_ros_bridge_jvrc")

(defclass robot-menu
  :super propertied-object
  :slots (robot-name tfl reference-frame-id
                     use-robot-interface-flag
                     potentio-vector joint-name-list
                     mg)
  )

(defmethod robot-menu
  (:init
   ()
   ;; set slot variables
   (setq robot-name (string-upcase (if (unix::getenv "ROBOT") (unix::getenv "ROBOT") "jaxon_jvrc"))
         tfl (instance ros::transform-listener :init)
         reference-frame-id (ros::get-param "~reference_frame_id" "/ground")
         use-robot-interface-flag (ros::get-param "~use_robot_interface" t))
   ;; generate *robot*
   (when (or (not (boundp '*robot*)) (not *robot*))
     (load (robot-file robot-name))
     (setq *robot* (if use-robot-interface-flag
                       (init-robot-from-name robot-name)
                     (make-robot-model-from-name robot-name))))
   (setq mg (instance multi-gait :init :robot robot-name :make-robot-model nil))
   ;; subscriber
   (ros::subscribe "/joint_states" sensor_msgs::JointState #'send self :joint-states-callback)
   (ros::subscribe "/send_angle_command" std_msgs::empty #'send self :send-angle-to-real-robot-one-cb)
   (ros::subscribe "/robot_menu_command" std_msgs::empty #'send self :robot-menu-command-cb)
   (ros::subscribe "/set_robot_pose" std_msgs::empty #'send self :set-robot-pose-cb)
   (ros::subscribe "/set_robot_pose_with_av" std_msgs::empty #'send self :set-robot-pose-with-av-cb)
   (ros::subscribe "/gait_menu_command" std_msgs::empty #'send self :gait-menu-command-cb)
   ;; publisher
   (ros::advertise "/jsk_model_marker_interface/robot/reset_joint_states" sensor_msgs::JointState 1)
   (ros::advertise "/urdf_control_marker/set_pose" geometry_msgs::PoseStamped 1)
   (warning-message 2 (format nil "[~A] initialize robot-menu.~%" (ros::get-name)))
   )
  (:joint-states-callback
   (msg)
   (when (<= (length (send *robot* :joint-list))
             (length (send msg :name)))
     (setq potentio-vector (coerce (mapcar #'rad2deg (coerce (send msg :position) cons)) float-vector)
           joint-name-list (send msg :name))
     )
   )
  (:send-joint-states-to-marker
   ()
   (let ((msg (instance sensor_msgs::JointState :init
                        :header (instance std_msgs::Header :init :stamp (ros::time 0.0))
                        :name (send-all (send *robot* :joint-list) :name)
                        :position (mapcar #'deg2rad (concatenate cons (send *robot* :angle-vector))))))
     (ros::publish "/jsk_model_marker_interface/robot/reset_joint_states" msg)
     ))
  (:set-robot-pose
   ()
   (let ((msg (instance geometry_msgs::PoseStamped :init
                        :header (instance std_msgs::Header :init :frame_id reference-frame-id :stamp (ros::time 0.0))
                        :pose (instance geometry_msgs::Pose :init :orientation (instance geometry_msgs::Quaternion :init :x 0 :y 0 :z 0 :w 1)))))
     (ros::publish "/urdf_control_marker/set_pose" msg)
     ))
  (:send-angle-to-real-robot-one-cb
   (msg)
   (let* ((interpolation-time 5000)
          (title (format nil "Send angle-vector to real robot. Time : ~A [msec]~%" interpolation-time))
          (menu-list (list "cancel" "yes"))
          (req (instance drc_task_common::RvizMenuCallRequest :init
                         :title title
                         :menu_list menu-list))
          res res-value)
     (setq res (ros::service-call "/rviz_menu_call" req)
           res-value (elt menu-list (send res :index)))
     (when (equal res-value "yes")
       (ros::ros-info "[~A] send angle-vector to real robot." (ros::get-name))
       (if use-robot-interface-flag
           (progn
             (send *ri* :angle-vector (send *robot* :angle-vector) interpolation-time)
             (send *ri* :wait-interpolation))
         (let ((req_set_jointangles
                (instance hrpsys_ros_bridge::OpenHRP_SequencePlayerService_setJointAnglesRequest :init
                          :jvs (mapcar #'(lambda (a n)
                                           (if (send *robot* :joint n)
                                               (deg2rad (send (send *robot* :joint n) :joint-angle))
                                             (deg2rad a)))
                                       (concatenate cons potentio-vector) joint-name-list)
                          :tm (* interpolation-time 1e-3)
                          ))
               (req_wait_interpolation
                (instance hrpsys_ros_bridge::OpenHRP_SequencePlayerService_waitInterpolationRequest :init)))
           (ros::service-call "/SequencePlayerServiceROSBridge/setJointAngles" req_set_jointangles)
           (ros::service-call "/SequencePlayerServiceROSBridge/waitInterpolation" req_wait_interpolation)))))
   (ros::ros-info "[~A] finished sending angle-vector." (ros::get-name))
   )
  (:robot-menu-command-cb
   (msg)
   (ros::ros-info "[~A] open robot menu in rviz." (ros::get-name))
   (let* ((title (format nil "Robot Menu~%"))
          (menu-list (list "cancel"
                           "potentio-vector"
                           ":reset-pose"
                           ":reset-manip-pose"
                           ":init-pose"
                           ":quadruped-pose"))
          (req (instance drc_task_common::RvizMenuCallRequest
                         :init
                         :title title
                         :menu_list menu-list))
          res res-value)
     (setq res (ros::service-call "/rviz_menu_call" req)
           res-value (elt menu-list (send res :index)))
     (cond ((equal res-value "potentio-vector")
            (if use-robot-interface-flag
                (send *robot* :angle-vector (send *ri* :state :potentio-vector))
              (send *robot* :angle-vector potentio-vector)))
           ((find-method *robot* (read-from-string res-value))
            (eval (list 'send '*robot* '(read-from-string res-value))))
           (t
            (send *robot* :angle-vector (mg . quadruped-pose)))
           )
     (send self :send-joint-states-to-marker)
     ))
  (:set-robot-pose-cb
   (msg)
   (send self :set-robot-pose)
   )
  (:set-robot-pose-with-av-cb
   (msg)
   (if use-robot-interface-flag
       (send *robot* :angle-vector (send *ri* :state :potentio-vector))
     (send *robot* :angle-vector potentio-vector))
   (send self :send-joint-states-to-marker)
   (send self :set-robot-pose)
   )
  (:gait-menu-command-cb
   (msg)
   (ros::ros-info "[~A] open gait menu in rviz." (ros::get-name))
   (let* ((title (format nil "Gait Menu~%"))
          (menu-list (list "cancel"
                           "move biped"
                           "move quadruped"
                           "release inner force"))
          (req (instance drc_task_common::RvizMenuCallRequest
                         :init
                         :title title
                         :menu_list menu-list))
          res res-value)
     (setq res (ros::service-call "/rviz_menu_call" req)
           res-value (elt menu-list (send res :index)))
     (let (fsm-res)
       (cond ((equal res-value "move biped")
              (when use-robot-interface-flag
                (setq fsm-res (ros::service-call "/call_gait_state_event" (instance hrpsys_ros_bridge_jvrc::StringRequestRequest :init :data "to biped")))
                (when (send fsm-res :result)
                  (send mg :quadruped->biped)
                  (ros::service-call "/call_gait_state_event" (instance hrpsys_ros_bridge_jvrc::StringRequestRequest :init :data "finish biped")))))
             ((equal res-value "move quadruped")
              (when use-robot-interface-flag
                (setq fsm-res (ros::service-call "/call_gait_state_event" (instance hrpsys_ros_bridge_jvrc::StringRequestRequest :init :data "to quadruped")))
                (when (send fsm-res :result)
                  (send mg :biped->quadruped)
                  (ros::service-call "/call_gait_state_event" (instance hrpsys_ros_bridge_jvrc::StringRequestRequest :init :data "finish quadruped")))))
             ((equal res-value "release inner force")
              (when use-robot-interface-flag
                (let ((gt (mg . current-gait-type)))
                  (setq fsm-res (ros::service-call "/call_gait_state_event"
                                                   (instance hrpsys_ros_bridge_jvrc::StringRequestRequest :init :data (if (eq gt 'biped) "biped walk" "quadruped walk"))))
                  (when (send fsm-res :result)
                    (send (mg . q-util) :release-inner-force (mg . current-gait-type))
                    (ros::service-call "/call_gait_state_event"
                                       (instance hrpsys_ros_bridge_jvrc::StringRequestRequest :init :data (if (eq gt 'biped) "biped stop" "quadruped stop")))))))
             ))
     ))
  )

;; main
(progn
  (ros::rate 10)
  (let ((service-list (list "/SequencePlayerServiceROSBridge/setJointAngles"
                            "/SequencePlayerServiceROSBridge/waitInterpolation"
                            "/call_gait_state_event"
                            "/rviz_menu_call")))
    (dolist (s service-list)
      (ros::wait-for-service s)))
  (instance robot-menu :init)
  (while t
    (ros::sleep)
    (ros::spin-once)
    )
  )
