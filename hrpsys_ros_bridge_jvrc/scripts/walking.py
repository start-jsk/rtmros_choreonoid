#!/usr/bin/env python
import roslib; roslib.load_manifest("hrpsys")
import OpenRTM_aist.RTM_IDL # for catkin
import sys

from hrpsys import rtm
from hrpsys.hrpsys_config import *
import OpenHRP

def s():
    hcf.startStabilizer()
    hcf.startAutoBalancer()
    ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp.default_step_time = 0.6
    ggp.stride_parameter = [0.25, 0.1, 15.0, 0.1]
    ggp.optional_go_pos_finalize_footstep_num = 0
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    # my_pose=[2.243121364372378e-08, -1.677387289795795e-06, -47.4265569017645, 94.85305345382321, -47.42648705431388, 1.7710867212618633e-06, -2.2431213643723734e-08, 1.6773872897957934e-06, -47.42655690176444, 94.85305345382321, -47.42648705431388, -1.7710867212618696e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 39.99999999999998, -19.99999999999999, -4.999999999999997, -79.99999999999996, 0.0, 0.0, -19.99999999999999, 0.0, 39.99999999999998, 19.99999999999999, 4.999999999999997, -79.99999999999996, 0.0, 0.0, -19.99999999999999, 0.013164060391472788, 0.026339217559469965, 0.012809998735311013, 0.025632659621207473, 7701.64124886024]
    # hcf.seq_svc.setJointAngles(my_pose, 3)
    # hcf.waitInterpolation()

def d():
    hcf.startStabilizer()
    hcf.startAutoBalancer()
    ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp.default_step_time = 1.0
    ggp.stride_parameter = [0.15, 0.05, 10.0, 0.05]
    ggp.optional_go_pos_finalize_footstep_num = 0
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    # my_pose=[0.005758720663153675, -0.17758120104529337, -14.963027553687986, 39.202856573205956, -23.963956596380577, 0.17785865643794385, -0.0007829792552356656, -0.17916383528538288, -14.94666867854323, 39.2198422547489, -23.96483615392008, 0.1790635235710215, -0.0006556704280599674, -0.043486876531697305, -7.892070044007178e-05, 7.038159392312187e-05, 0.011113521670449137, -0.002385773037734423, 30.007973710014266, -19.97299885449637, -4.984098187334756, -79.96460662911088, 0.0009470183365338822, 0.003918283807496354, -19.992440294067375, 0.002390424151659463, 30.0079744759056, 19.973225040157768, 4.984228452333181, -79.96453229924315, -0.0009363907210502285, -0.003888559832133108, -19.992425152445765, 0.0046110216103121895, 0.009238847421223549, 0.004649646102526224, 0.009317503733382642, 358.1557805867397]
    # hcf.seq_svc.setJointAngles(my_pose, 3)
    # hcf.waitInterpolation()

    # st param
def doa():
    d()
    g(0, 0, 0)
    g(1, 0, 0)
    g(1, 0, 0)
    g(1, 0.1, 0)
    g(1, 0, 0)
    g(0.4, 0, 0)
    g(0, 0, -90)
    g(-0.1, 0, 0)
def g(x,y,r):
    hcf.abc_svc.goPos(x, y, r)
    hcf.abc_svc.waitFootSteps()
def h():
    print "funcs, d()=default_mode, s()=speedup_mode, g(x, y, r)=go_pos"

def m():
    print "funcs, d()=default_mode, s()=speedup_mode, g(x, y, r)=go_pos"
def dor():
    doa()
# copy from hrpsys/lib/python2.7/dist-packages/hrpsys_config.py
import argparse, code
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="hrpsys command line interpreters, try `./hrpsys_tools_config.py--host xxxx --port xxxx  -i` or `ipython ./hrpsys_tools_config.py -- --host xxxx --port xxxx` for robot debugging, we recommend to use ipython")
    parser.add_argument('--host', help='corba name server hostname')
    parser.add_argument('--port', help='corba name server port number')
    parser.add_argument('-i', help='interactive mode',  action='store_true')
    parser.add_argument('-c', help='execute command',  nargs='*')
    parser.add_argument('--use-unstable-rtc', help='use unstable rtc', action='store_true')
    args, unknown = parser.parse_known_args()

    if args.i: # interactive
        sys.argv.remove('-i')
    if args.c:
        sys.argv.remove('-c')
        [sys.argv.remove(a) for a in args.c] # remove command from sys.argv
    if args.host:
        rtm.nshost = args.host; sys.argv = [sys.argv[0]] + sys.argv[3:]
    if args.port:
        rtm.nsport = args.port; sys.argv = [sys.argv[0]] + sys.argv[3:]

    # ipython ./hrpsys_tools_config.py -i -- --port 2809
    hcf = HrpsysConfigurator()
    if args.use_unstable_rtc: # use Unstable RTC
        hcf.getRTCList = hcf.getRTCListUnstable; sys.argv = [sys.argv[0]] + sys.argv[2:]
    if args.i or '__IPYTHON__' in vars(__builtins__):
        hcf.waitForModelLoader()
        if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
            hcf.waitForRTCManagerAndRoboHardware(robotname=sys.argv[1])
            sys.argv = [sys.argv[0]] + sys.argv[2:]
        hcf.findComps()
        print >> sys.stderr, "[hrpsys.py] #\n[hrpsys.py] # use `hcf` as robot interface, for example hcf.getJointAngles()\n[hrpsys.py] #"
        while args.c != None:
            print >> sys.stderr, ">>", args.c[0]
            exec(args.c[0])
            args.c.pop(0)
        hcf.startStabilizer()
        hcf.startAutoBalancer()
        if not (args.i and '__IPYTHON__' in vars(__builtins__)):
            h()
            code.interact(local=locals()) #drop in shell if invoke from python, or ipython without -i option
    elif len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
