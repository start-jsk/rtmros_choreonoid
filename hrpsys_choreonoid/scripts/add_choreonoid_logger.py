#!/usr/bin/env python

from hrpsys.hrpsys_config import *
import OpenHRP

import argparse, code
if __name__ == '__main__':
    if hasattr(sys, 'argv'):
        ### called from command line
        parser = argparse.ArgumentParser(description="hrpsys command line interpreters, try `hrpsyspy -- --host xxxx --port xxxx `")
        parser.add_argument('--host', help='corba name server hostname', default='localhost')
        parser.add_argument('--port', help='corba name server port number', default=15005)
        parser.add_argument('--robot', help='robot modlule name (RobotHardware0) for real robot)', default='RobotHardware0')
        parser.add_argument('--sync_rtc', help='rtc synchronized with choreonoid_logger', default='RobotHardware_choreonoid0')
        parser.add_argument('--save', help='saveLog', default=None)
        parser.add_argument('--clear', help='clearLog', action='store_true', default=False)
        parser.add_argument('--maxlength', help='maxlength', default='1000000')
        parser.add_argument('--loglist', help='log list like [[\'rh\', \'rfsensor\'], [\'rh\', \'lfsensor\']]', default=None)

        args, unknown = parser.parse_known_args()
    else:
        ### called from choreonoid
        args = argparse.Namespace()
        args.host = 'localhost'
        args.port = 15005
        args.robot = 'JAXON_RED'
        args.sync_rtc = 'PDcontroller0'
        args.save = None
        args.clear = None
        args.loglist = None
        args.maxlength = '1000'


    if args.host:
        rtm.nshost = args.host
    if args.port:
        rtm.nsport = args.port
    rh_name = args.robot

    hcf = HrpsysConfigurator()

    # estimate robot hardware name
    hcf.waitForRTCManager(args.host)
    if args.save:
        log, log_svc, ver = hcf.findComp('DataLogger', 'log_choreonoid')
        if log_svc:
            log_svc.save(args.save)
        exit(-1)

    if args.clear:
        log, log_svc, ver = hcf.findComp('DataLogger', 'log_choreonoid')
        if log_svc:
            log_svc.maxLength(int(args.maxlength))
            log_svc.clear()
        exit(-1)

    if rh_name in [x.name() for x in hcf.ms.get_components()]:
        pass
    else:
        for x in hcf.ms.get_components():
            try: # see findService() in rtm.py
                for pp in x.ref.get_component_profile().port_profiles:
                    ifs = pp.interfaces
                    for aif in ifs:
                        if aif.instance_name == "service0" and aif.type_name == "RobotHardwareService" and aif.polarity == PROVIDED:
                            con_prof = RTC.ConnectorProfile("noname", "", [pp.port_ref], [])
                            ret, con_prof = pp.port_ref.connect(con_prof)
                            ior = any.from_any(con_prof.properties[0].value)
                            svc = rtm.orb.string_to_object(ior)
                            if len(svc.getStatus().angle) > 0 :
                                rh_name = x.name()
            except:
                pass

    hcf.waitForRobotHardware(rh_name)
    hcf.checkSimulationMode()
    hcf.findComps(max_timeout_count=0, verbose=False)

    if args.loglist:
        log_tmp = hcf.createComp('DataLogger', 'log_choreonoid')
        ## swap choreonoid logger
        if log_tmp:
            hcf.log = log_tmp[0]
            hcf.log_svc = log_tmp[1]

            exec('log_list = ' + args.loglist)
            for ll in log_list:
                exestr = "hcf.connectLoggerPort(hcf." + ll[0] + ",'" + ll[1] + "')"
                exec(exestr)
        exit(-1)

    if args.sync_rtc:
        sync_rtc = rtm.findRTC(args.sync_rtc)
    if sync_rtc:
        log_tmp = hcf.createComp('DataLogger', 'log_choreonoid')
        ## swap choreonoid logger
        if log_tmp:
            hcf.log = log_tmp[0]
            hcf.log_svc = log_tmp[1]

            ### join EC of sync_rtc, simulation should be stopped
            hcf.log.ec.stop()
            ec = sync_rtc.ec
            ec.add_component(hcf.log.ref)
            hcf.log.ec = ec
            hcf.log.start()
