#!/usr/bin/env python

from hrpsys_ros_bridge_tutorials.hrp2_hrpsys_config import *

class JSKHRP2ChoreonoidHrpsysConfigurator(JSKHRP2HrpsysConfigurator):
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

    def startABSTIMP (self):
        ### not used on hrpsys
        if self.ROBOT_NAME == "HRP2JSKNT" or self.ROBOT_NAME == "HRP2JSKNTS":
            self.el_svc.setServoErrorLimit("RARM_JOINT7", sys.float_info.max)
            self.el_svc.setServoErrorLimit("LARM_JOINT7", sys.float_info.max)
            self.rh_svc.servo("RARM_JOINT7",OpenHRP.RobotHardwareService.SWITCH_OFF)
            self.rh_svc.servo("LARM_JOINT7",OpenHRP.RobotHardwareService.SWITCH_OFF)
            self.rh_svc.setServoGainPercentage("RLEG_JOINT6", 30.0)
            self.rh_svc.setServoGainPercentage("LLEG_JOINT6", 30.0)
        ###
        self.startAutoBalancer()
        self.ic_svc.startImpedanceController("larm")
        self.ic_svc.startImpedanceController("rarm")
        self.startStabilizer()

if __name__ == '__main__':
    if os.environ["CHOREONOID_ROBOT"] == "HRP2JSKNTS":
        robot_name = "HRP2JSKNTS"
    elif os.environ["CHOREONOID_ROBOT"] == "HRP2JSKNT":
        robot_name = "HRP2JSKNT"
    elif os.environ["CHOREONOID_ROBOT"] == "HRP2JSK":
        robot_name = "HRP2JSK"
    else:
        robot_name = ""
    hcf = JSKHRP2ChoreonoidHrpsysConfigurator(robot_name)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
        hcf.startABSTIMP()
    else :
        hcf.init()
