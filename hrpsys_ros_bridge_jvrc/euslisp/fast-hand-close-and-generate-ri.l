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

#|
hcf.startAutoBalancer()
hcf.ic_svc.startImpedanceController("larm")
hcf.ic_svc.startImpedanceController("rarm")
hcf.startStabilizer()
# abc param
ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
ggp.default_step_time = 1.0
ggp.stride_parameter = [0.15, 0.05, 30.0, 0.05]
ggp.optional_go_pos_finalize_footstep_num = 0
hcf.abc_svc.setGaitGeneratorParam(ggp)
# st param
stp = hcf.st_svc.getParameter()
stp.emergency_check_mode = OpenHRP.StabilizerService.CP
hcf.st_svc.setParameter(stp)
# ic param
ric = hcf.ic_svc.getImpedanceControllerParam("rarm")[1]
ric.K_p = 1000
hcf.ic_svc.setImpedanceControllerParam("rarm", ric)
lic = hcf.ic_svc.getImpedanceControllerParam("larm")[1]
lic.K_p = 1000
hcf.ic_svc.setImpedanceControllerParam("larm", lic)

mypose = [0.003047, -0.002276, -1.574, 2.73648, -1.01477, -0.010465, 0.001111, -0.00147, -1.58267, 2.72659, -0.996271, -0.010734, 0.009035, 0.610865, 0.00042, -3.286455e-05, 0.000964, 1.36277, -0.086354, -0.895883, -1.47528, -2.08261, -3.10269, -0.016172, -0.823221, -1.36346, -0.039878, 0.902656, 1.50219, -2.06699, 1.53389, -0.161394, -1.35985, 0, 0, 0, 0, 0]
hcf.seq_svc.setJointAngles(mypose, 5)
hcf.seq_svc.waitInterpolation()

# massugu
hcf.abc_svc.goPos(1.5 * 1 + 0.3, 0, 0)
hcf.abc_svc.waitFootSteps()
hcf.abc_svc.goPos(0, 0, 90)
hcf.abc_svc.waitFootSteps()
# hidari
hcf.abc_svc.goPos(1.5 * 3 + 0.3, 0, 0)
hcf.abc_svc.waitFootSteps()
hcf.abc_svc.goPos(0, 0, -90)
hcf.abc_svc.waitFootSteps()
# massugu
hcf.abc_svc.goPos(1.5 * 4 + 0.3, 0, 0)
hcf.abc_svc.waitFootSteps()
hcf.abc_svc.goPos(0, 0, -90)
hcf.abc_svc.waitFootSteps()
# migi
hcf.abc_svc.goPos(1.5 * 1 + 0.1, 0, 0)
hcf.abc_svc.waitFootSteps()
hcf.abc_svc.goPos(0, 0, 90)
hcf.abc_svc.waitFootSteps()
# massugu
hcf.abc_svc.goPos(1.5 * 1 + 0.1, 0, 0)
hcf.abc_svc.waitFootSteps()
hcf.abc_svc.goPos(0, 0, -90)
hcf.abc_svc.waitFootSteps()
# migi
hcf.abc_svc.goPos(1.5 * 2 + 0.1, 0, 0)
hcf.abc_svc.waitFootSteps()
hcf.abc_svc.goPos(0, 0, 90)
hcf.abc_svc.waitFootSteps()
# massugu
hcf.abc_svc.goPos(1.5 * 2 + 0.3, 0, 0)
hcf.abc_svc.waitFootSteps()
|#
