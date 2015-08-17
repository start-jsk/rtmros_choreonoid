(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :jaxon_jvrc "package://hrpsys_ros_bridge_jvrc/models/jaxon_jvrc.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-utils.l"))
  (require :jaxon_jvrc-utils "package://hrpsys_ros_bridge_jvrc/euslisp/jaxon_jvrc-utils.l"))

(defclass jaxon_jvrc-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod jaxon_jvrc-interface
  (:init (&rest args)
         (send-super* :init :robot jaxon_jvrc-robot args)))

(defun jaxon_jvrc-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* jaxon_jvrc-interface :init args)))
  (if (not (boundp '*jaxon_jvrc*))
      (setq *jaxon_jvrc* (instance jaxon_jvrc-robot :init))))
