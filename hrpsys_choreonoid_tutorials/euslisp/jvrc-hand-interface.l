(ros::load-ros-manifest "control_msgs")

(defclass jvrc-hand-controller
  :super propertied-object
  :slots (laction raction)
  )
(defmethod jvrc-hand-controller
  (:init ()
   (ros::roseus (format nil "roseus_jvrc_hand_interface"))
   (setq laction (instance ros::simple-action-client :init
                           "/lhand_controller/follow_joint_trajectory_action"
                           control_msgs::FollowJointTrajectoryAction))
   (send laction :put :fingers (list "LARM_F_JOINT0" "LARM_F_JOINT1"))
   (send laction :wait-for-server 5.0)
   (setq raction (instance ros::simple-action-client :init
                           "/rhand_controller/follow_joint_trajectory_action"
                           control_msgs::FollowJointTrajectoryAction))
   (send raction :put :fingers (list "RARM_F_JOINT0" "RARM_F_JOINT1"))
   (send raction :wait-for-server 5.0)
   )
  (:send-goal (action angle0 angle1 tm)
   (let ((goal (send action :make-goal-instance)))
     (send goal :header :stamp (ros::time-now))
     (send goal :header :seq 1)
     (send goal :goal :trajectory :header :stamp (ros::time-now))
     (send goal :goal :trajectory :header :seq 1)
     (send goal :goal :trajectory :joint_names (send action :get :fingers))
     (send goal :goal :trajectory :points (list (instance trajectory_msgs::JointTrajectoryPoint :init
                                                          :positions (float-vector (deg2rad angle0) (deg2rad angle1))
                                                          :time_from_start (ros::Time tm))))
     (ros::spin-once)
     (send action :send-goal goal)
     ))
  (:move-gripper
   (arm pos &key (effort 30) (wait t))
   (let* (goal
          (clients
           (case arm
             (:rarm (list raction))
             (:larm (list laction))
             (:arms (list raction laction))
             (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm)))))
     (dolist (client clients)
       (send self :send-goal client
             (- (+ (* (- 80 -15) pos) -15))
             (+ (* (- 80 -15) pos) -15) (/ 30.0 effort)))
     (if wait (send-all clients :wait-for-result))
     ))
  (:start-grasp
   (&optional (arm :arms) &key ((:gain g) 30) (wait t))
   (send self :move-gripper arm 1.0 :effort g :wait wait)
   )
  (:stop-grasp
   (&optional (arm :arms) &key (wait t) ((:gain g) 30))
   (send self :move-gripper arm 0.0 :effort g :wait wait)
   )
  ;;
  (:hand-reset
   (&rest args) (warn ":hand-reset called with ~A~%" args))
  (:hand-stop
   (&rest args) (warn ":hand-stop called with ~A~%" args))
  (:hand-resume
   (&rest args) (warn ":hand-resume called with ~A~%" args))
  (:get-joint-angle
   (&rest args) (warn ":get-joint-angle called with ~A~%" args))
  (:get-joint-velocity
   (&rest args) (warn ":get-joint-velocity called with ~A~%" args))
  (:get-joint-effort
   (&rest args) (warn ":get-joint-effort called with ~A~%" args))
  )
