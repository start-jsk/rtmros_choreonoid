#!/usr/bin/env python

from hrpsys_ros_bridge_tutorials.urata_hrpsys_config import *

class ChoreonoidHrpsysConfiguratorOrg(URATAHrpsysConfigurator):
    """
    Subclass to specify Choreonoid-dependent code.
    Please inherit this class and hrpsys configurator for each robot to specify
    robot-dependent class for Choreonoid.
    """

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
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]
    def setupLogger(self, maxlen=15000):
        HrpsysConfigurator.setupLogger(self, maxlen)
        # self.connectLoggerPort(self.rh, 'WAIST') # For debug
        for pn in filter (lambda x : re.match("Trans_", x), self.rh.ports.keys()):
            self.connectLoggerPort(self.rh, pn)

    def connectConstraintForceLoggerPorts(self):
        for pn in filter (lambda x : re.match("T_", x), self.rh.ports.keys()):
            self.connectLoggerPort(self.rh, pn)
        for pn in filter (lambda x : re.match("F_", x), self.rh.ports.keys()):
            self.connectLoggerPort(self.rh, pn)

    def init (self, robotname="Robot", url="", connect_constraint_force_logger_ports = False):
        URATAHrpsysConfigurator.init(self, robotname, url)
        if connect_constraint_force_logger_ports:
            self.connectConstraintForceLoggerPorts()

    def parse_arg_for_connect_ports (self, arg_list):
        # Check flag for connect ports
        # Return arg list without connect port flag and connect port flag
        if "--connect-constraint-force-logger-ports" in arg_list:
            tmpidx = arg_list.index("--connect-constraint-force-logger-ports")
            return [arg_list[:tmpidx]+arg_list[tmpidx+1:],True]
        else:
            return [arg_list, False]

class ChoreonoidHrpsysConfigurator(ChoreonoidHrpsysConfiguratorOrg):
    """
    Subclass to specify Choreonoid-dependent code with RobotHardware.
    Please inherit this class and hrpsys configurator for each robot to specify
    robot-dependent class for Choreonoid.
    """

    def waitForRobotHardware(self, robotname="Robot"):
        '''!@brief
        Wait for RobotHardware is exists and activated.

        @param robotname str: name of RobotHardware component.
        '''
        self.rh = None
        timeout_count = 0
        # wait for simulator or RobotHardware setup which sometime takes a long time
        while self.rh == None and timeout_count < 10:  # <- time out limit
            if timeout_count > 0: # do not sleep initial loop
                time.sleep(1);
            self.rh = rtm.findRTC("RobotHardware_choreonoid0")
            print(self.configurator_name + "wait for %s : %s ( timeout %d < 10)" % ( robotname, self.rh, timeout_count))
            if self.rh and self.rh.isActive() == None:  # just in case rh is not ready...
                self.rh = None
            timeout_count += 1

        if not self.rh:
            print(self.configurator_name + "Could not find RobotHardware_choreonoid0")
            if self.ms:
                print(self.configurator_name + "Candidates are .... " + str([x.name()  for x in self.ms.get_components()]))
            print(self.configurator_name + "Exitting.... " + robotname)
            exit(1)

        print(self.configurator_name + "findComps -> RobotHardware_choreonoid0 : %s isActive? = %s " % (self.rh,  self.rh.isActive()))

        # wait for simulator or RobotHardware setup which sometime takes a long time
        self.rh_choreonoid = None
        while self.rh_choreonoid == None and timeout_count < 10:  # <- time out limit
            if timeout_count > 0: # do not sleep initial loop
                time.sleep(1);
            self.rh_choreonoid = rtm.findRTC(robotname)
            print(self.configurator_name + "wait for %s : %s ( timeout %d < 10)" % ( robotname, self.rh_choreonoid, timeout_count))
            if self.rh_choreonoid and self.rh_choreonoid.isActive() == None:  # just in case rh is not ready...
                self.rh_choreonoid = None
            timeout_count += 1

        if not self.rh_choreonoid:
            print(self.configurator_name + "Could not find " + robotname)
            if self.ms:
                print(self.configurator_name + "Candidates are .... " + str([x.name()  for x in self.ms.get_components()]))
            print(self.configurator_name + "Exitting.... " + robotname)
            exit(1)

        print(self.configurator_name + "findComps -> %s : %s isActive? = %s " % (robotname, self.rh_choreonoid,  self.rh_choreonoid.isActive()))

    def activateComps(self):
        stash_rh = self.rh
        self.rh = self.rh_choreonoid
        HrpsysConfigurator.activateComps(self)
        self.rh = stash_rh
