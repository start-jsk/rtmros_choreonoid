#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import *

class TABLIS_HrpsysConfigurator(ChoreonoidHrpsysConfigurator):
    def getRTCList(self):
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['kf', "KalmanFilter"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            # ['octd', "ObjectContactTurnaroundDetector"],
            # ['es', "EmergencyStopper"],
            # ['rfu', "ReferenceForceUpdater"],
            # ['ic', "ImpedanceController"],
            # ['abc', "AutoBalancer"],
            # ['st', "Stabilizer"],
            # ['co', "CollisionDetector"],
            # ['hes', "EmergencyStopper"],
            # ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]
    def startABSTIMP (self):
        #self.startAutoBalancer()
        #self.setResetPose()
        #self.startStabilizer()
        pass

if __name__ == '__main__':
    hcf = TABLIS_HrpsysConfigurator("TABLIS")
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    else :
        hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
