(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon_jvrc "package://hrpsys_ros_bridge_jvrc/models/jaxon_jvrc.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-utils.l"))
  (require :jaxon_jvrc-utils "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-utils.l"))

(defclass jaxon_jvrc-interface
  :super rtm-ros-robot-interface
  :slots (hand-enable))
(defmethod jaxon_jvrc-interface
  (:init (&rest args)
         (prog1
             (send-super* :init :robot jaxon_jvrc-robot args)
           (setq hand-enable (send self :set-hand-controller))
           (when (and (ros::get-param "/use_sim_time")
                      (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_jvrc/euslisp/jvrc-hand-interface.l")))
             (load "package://hrpsys_ros_bridge_jvrc/euslisp/jvrc-hand-interface.l")
             (send self :put :hand-controller (instance jvrc-hand-controller :init))
             (setq hand-enable t))
           ))
  (:set-hand-controller () nil)
  (:move-gripper
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :move-gripper args)
     (warn ";; can not use hand~%")))
  (:start-grasp
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :start-grasp args)
     (warn ";; can not use hand~%")))
  (:stop-grasp
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :stop-grasp args)
     (warn ";; can not use hand~%")))
  (:hand-reset
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :reset args)
     (warn ";; can not use hand~%")))
  ;; (:hand-open
  ;;  (&rest args)
  ;;  (send* (send self :get :hand-controller) :open args))
  ;; (:hand-close
  ;;  (&rest args)
  ;;  (send* (send self :get :hand-controller) :close args))
  (:hand-stop
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :stop args)
     (warn ";; can not use hand~%")))
  (:hand-resume
   (&rest args)
   (if hand-enable
       (send* (send self :get :hand-controller) :resume args)
     (warn ";; can not use hand~%")))
  )

(defun jaxon_jvrc-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon_jvrc-interface :init args)))
  (if (not (boundp '*jaxon_jvrc*))
      (setq *jaxon_jvrc* (instance jaxon_jvrc-robot :init))))
