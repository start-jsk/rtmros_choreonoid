#!/usr/bin/env python

from hrpsys_ros_bridge_tutorials.hrp2_hrpsys_config import *

class JSKHRP2ChoreonoidHrpsysConfigurator(JSKHRP2HrpsysConfigurator):
    def getRTCList (self):
        rtclist = JSKHRP2HrpsysConfigurator.getRTCList(self)
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            if ['hc', "HRP3HandController"] in rtclist:
                rtclist.remove(['hc', "HRP3HandController"]) #HRP3HandController is created by choreonoid like RobotHardware
        return rtclist

    def getRTCInstanceList(self, verbose=True):
        ret = JSKHRP2HrpsysConfigurator.getRTCInstanceList(self, verbose)
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            ret.append(self.hc)
        return ret

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

    def waitForHRP3HandController(self, robotname="Robot"):
        '''!@brief
        Wait for HRP3HandController is exists and activated.

        @param robotname str: name of RobotHardware component.
        '''
        self.hc = None
        timeout_count = 0
        # wait for simulator or HRP3HANDController setup which sometime takes a long time
        while self.hc == None and timeout_count < 10:  # <- time out limit
            if timeout_count > 0: # do not sleep initial loop
                time.sleep(1);
            self.hc = rtm.findRTC("HRP3HandController_choreonoid0")
            print(self.configurator_name + "wait for %s : %s ( timeout %d < 10)" % ( "HRP3Hand", self.hc, timeout_count))
            if self.hc and self.hc.isActive() == None:  # just in case hc is not ready...
                self.hc = None
            timeout_count += 1

        if not self.hc:
            print(self.configurator_name + "Could not find HRP3HandController_choreonoid0")
            if self.ms:
                print(self.configurator_name + "Candidates are .... " + str([x.name()  for x in self.ms.get_components()]))
            print(self.configurator_name + "Exitting.... " + robotname)
            exit(1)

        print(self.configurator_name + "findComps -> HRP3HandController_choreonoid0 : %s isActive? = %s " % (self.hc,  self.hc.isActive()))

    def waitForRTCManagerAndRobotHardware(self, robotname="Robot", managerhost=nshost):
        '''!@brief
        Wait for both RTC Manager (waitForRTCManager()) and RobotHardware (waitForRobotHardware()) and HRP3HandController (waitForHRP3HandController)
        @param managerhost str: name of host computer that manager is running
        @param robotname str: name of RobotHardware component.
        '''
        self.waitForRTCManager(managerhost)
        self.waitForRobotHardware(robotname)
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            self.waitForHRP3HandController(robotname)
        self.checkSimulationMode()

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
