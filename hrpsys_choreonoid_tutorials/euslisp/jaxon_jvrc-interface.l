(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon_jvrc "package://hrpsys_choreonoid_tutorials/models/jaxon_jvrc.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-utils.l"))
  (require :jaxon_jvrc-utils "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-utils.l"))
(require :jaxon-common-interface "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-common-interface.l")

(defclass jaxon_jvrc-interface
  :super jaxon-common-interface
  :slots ())
(defmethod jaxon_jvrc-interface
  (:init (&rest args)
         (prog1
             (send-super* :init :robot jaxon_jvrc-robot args)
           (setq hand-enable (send self :set-hand-controller))
           (when (and (ros::get-param "/use_sim_time")
                      (probe-file (ros::resolve-ros-path "package://hrpsys_choreonoid_tutorials/euslisp/jvrc-hand-interface.l")))
             (load "package://hrpsys_choreonoid_tutorials/euslisp/jvrc-hand-interface.l")
             (send self :put :hand-controller (instance jvrc-hand-controller :init))
             (setq hand-enable t))
           ))
  (:set-hand-controller () nil)
  )

(defun jaxon_jvrc-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon_jvrc-interface :init args)))
  (if (not (boundp '*jaxon_jvrc*))
      (setq *jaxon_jvrc* (instance jaxon_jvrc-robot :init))))
