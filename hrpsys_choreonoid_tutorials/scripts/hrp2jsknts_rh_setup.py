#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.hrp2_choreonoid_hrpsys_config import *

if __name__ == '__main__':
    hcf = JSKHRP2ChoreonoidHrpsysConfigurator("HRP2JSKNTS")
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
        hcf.startABSTIMP()
    else :
        hcf.init()
