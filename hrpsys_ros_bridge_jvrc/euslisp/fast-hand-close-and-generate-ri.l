#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_jvrc/euslisp/jvrc-hand-interface.l")
(send (instance jvrc-hand-controller :init) :start-grasp :arms)

(load "package://hrpsys_ros_bridge_jvrc/euslisp/4legs-walking.l")
(setup-robot :robot 'jaxon_jvrc-init)

(warn "(send *ri* :autobalancerservice_releaseEmergencyStop)~%")

;; for hcf

#|

rtconf 10.1.4.10:2809/abc.rtc set debugLevel 3

ipython -i `rospack find hrpsys_tools`/scripts/hrpsys_tools_config.py -- --use-unstable-rtc --host localhost --port 2809

C-d -> %paste

|#
