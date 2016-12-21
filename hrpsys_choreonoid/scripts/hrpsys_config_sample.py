#!/usr/bin/env python
###
### Please stop nameserver 'sudo service omniorb4-nameserver stop'
###

from hrpsys.hrpsys_config import *
import OpenHRP

### settings
nameservicehost = 'localhost'
nameserviceport = 15005
robotname = 'JAXON_RED(Robot)0' ## name of BodyRTC
###

import sys

setattr(sys, 'argv', [''])

rtm.nshost = nameservicehost
rtm.nsport = nameserviceport

hcf = HrpsysConfigurator()
hcf.getRTCList = hcf.getRTCListUnstable

hcf.waitForRTCManager(nameservicehost)
hcf.waitForRobotHardware(robotname)
hcf.findComps(max_timeout_count=0, verbose=False) ## search all components from getRTCList
##
## [hcf.abc, hcf.abc_svc, hcf.abc_version] = hcf.findComp('AutoBalancer', 'abc') ## or single component search

### debug message
rtc = hcf.getRTCInstanceList(verbose=False)
print('\033[33;1mStarting hrpsys python interface for %s\033[0m'%(rtc[0].name()))
print('\033[32;1m Installed rtc are ...\033[0m'),
for r in rtc[1:]:
    print('\033[32;1m %s \033[0m'%(r.name())),
print('\033[32;1m\033[0m')
print("\033[32;1m Use 'hcf' instance to access to the hrpsys controller\033[0m")

### white code here

#if hcf.abc_svc:
#    hcf.abc_svc.goPos(2, 0, 0)
