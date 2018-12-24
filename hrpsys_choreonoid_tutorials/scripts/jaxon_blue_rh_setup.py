#!/usr/bin/env python

import sys
from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import ChoreonoidHrpsysConfigurator
from hrpsys_ros_bridge_tutorials.jaxon_blue_hrpsys_config import JAXON_BLUEHrpsysConfigurator

class JAXON_BLUECnoidHrpsysConfigurator(ChoreonoidHrpsysConfigurator, JAXON_BLUEHrpsysConfigurator):
    def __init__(self):
        JAXON_BLUEHrpsysConfigurator.__init__(self)

    def getRTCList (self):
        ##return self.getRTCListUnstable()
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['es', "EmergencyStopper"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['rfu', "ReferenceForceUpdater"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    def startABSTIMP (self):
        ### not used on hrpsys
        # self.setServoErrorLimit("motor_joint",   sys.float_info.max)
        # self.setServoErrorLimit("RARM_F_JOINT0", sys.float_info.max)
        # self.setServoErrorLimit("RARM_F_JOINT1", sys.float_info.max)
        # self.setServoErrorLimit("LARM_F_JOINT0", sys.float_info.max)
        # self.setServoErrorLimit("LARM_F_JOINT1", sys.float_info.max)
        ###
        self.startAutoBalancer()
        # Suppress limit over message and behave like real robot that always angle-vector is in seq.
        self.setResetPose(1.0)
        self.ic_svc.startImpedanceController("larm")
        self.ic_svc.startImpedanceController("rarm")
        self.startStabilizer()

if __name__ == '__main__':
    hcf = JAXON_BLUECnoidHrpsysConfigurator()
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    else :
        hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
