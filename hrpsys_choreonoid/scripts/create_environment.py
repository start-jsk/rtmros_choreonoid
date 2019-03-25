from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.OpenRTMPlugin import *
from cnoid.PythonSimScriptPlugin import *
import cnoid.Body

import math
import os
import sys
import yaml
import time

import re
#import subprocess ## subprocess is recommended but choreonoid crashed when calling rospack find
import commands

try:
    objs_yaml = os.environ['CHOREONOID_SIMULATION_SETTING']
except:
    print >> sys.stderr, "environment variable 'CHOREONOID_SIMULATION_SETTING' is not found"
    raise

###
### sample yaml
#obj1:
#  name: 'MOGE_DOORWALL'
#  type: 'fixed'
#  file: '/home/leus/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/jvrc_models/models/door_wallMain.wrl'
#  translation: [0, 0, 4]
#  rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
#obj2:
#  name: 'HOGE_FLOOR'
#  file: '/home/leus/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/jvrc_models/models/visible_floor.wrl'
#  translation: [0, 0, -0.1]
#  rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
#robot:
#  name: ''
#  type: 'robot'
#  file: ''
#  translation: []
#  rotation: [[], [], []]
#  joint_angles: []
#  body_rtc:
#    name:
#    module:
#    config:
#    rate:
#  vision:
#    body:
#    namelist:
#sci:
#  name: ''
#  type: 'script'
#  file: ''

try:
    f = open(objs_yaml, 'r')
    dict_objs = yaml.load(f)
    f.close()
except:
    print >> sys.stderr, "can not read %s"%(objs_yaml)
    raise

___sci_path = os.path.abspath(os.path.dirname(__file__))

def parse_filename(filestr):
    re_find = re.compile(r'\$\(find ([^ ]+)\)')
    ret = re_find.search(filestr)
    if ret != None:
        pkgname = ret.group(1)
        packagepath = commands.getoutput('rospack find %s'%(pkgname))
        #packagepath = subprocess.check_output(['rospack', 'find', pkgname])
        filestr = filestr[:ret.start(0)] + packagepath + filestr[ret.end(0):]

    if filestr[0] != '/' and filestr[0] != '$':
        filestr = "%s/%s"%(___sci_path, filestr)

    if not os.path.exists(filestr):
        print >> sys.stderr, "file: %s not found"%(filestr)
        raise

    return filestr

def addObjectItem(world, obj_conf, filename, objname):
    global itemTreeView
    robotItem = BodyItem()
    robotItem.load(filename)
    robotItem.setName(objname)
    robot = robotItem.body()

    if 'static' in obj_info:
        static = obj_info['static']
        if static:
            robot.rootLink().setJointType(cnoid.Body.Link.JointType.FIXED_JOINT)
            robot.updateLinkTree()
        else:
            robot.rootLink().setJointType(cnoid.Body.Link.JointType.FREE_JOINT)
            robot.updateLinkTree()

    if 'translation' in obj_conf:
        trans = obj_conf['translation']
        robot.rootLink().setTranslation(trans);

    if 'rotation' in obj_conf:
        rot = obj_conf['rotation']
        robot.rootLink().setRotation(rot);

    for i in range(robot.numJoints()):
        robot.joint(i).q = 0

    robot.calcForwardKinematics()
    robotItem.storeInitialState()

    world.insertChildItem(robotItem, world.childItem())
    itemTreeView.checkItem(robotItem)


def addRobotItem(world, obj_conf, filename, objname):
    global itemTreeView
    robotItem = BodyItem()
    robotItem.load(filename)
    robot = robotItem.body()

    body_rtc_conf = None
    vision_conf = None
    if 'body_rtc' in obj_conf:
        body_rtc_conf = obj_conf['body_rtc']
    if 'vision' in obj_conf:
        vision_conf = obj_conf['vision']

    if 'translation' in obj_conf:
        trans = obj_conf['translation']
        robot.rootLink().setTranslation(trans);

    if 'rotation' in obj_conf:
        rot = obj_conf['rotation']
        robot.rootLink().setRotation(rot);

    q = [0] * robot.numJoints()
    if 'joint_angles' in obj_conf:
        q = obj_conf['joint_angles']

    for i in range(robot.numJoints()):
        robot.joint(i).q = q[i];

    robot.calcForwardKinematics()
    robotItem.storeInitialState()
    if body_rtc_conf and 'name' in body_rtc_conf:
        robotItem.setName(body_rtc_conf['name']) ## add name for BodyRTC name
    else:
        robotItem.setName(objname)
    world.insertChildItem(robotItem, world.childItem())
    itemTreeView.checkItem(robotItem)

    if body_rtc_conf:
        bodyRTC = BodyRTCItem()
        if 'module' in body_rtc_conf:
            bodyRTC.setControllerModule(parse_filename(body_rtc_conf['module']))
        if 'config' in body_rtc_conf:
            bodyRTC.setConfigMode(BodyRTCItem.ConfigMode.FILE)
            bodyRTC.setConfigFile(parse_filename(body_rtc_conf['config']))
            bodyRTC.setAutoConnectionMode(False)
        if 'rate' in body_rtc_conf:
            bodyRTC.setPeriodicRate(body_rtc_conf['rate'])

        #add bodyRTC
        robotItem.addChildItem(bodyRTC)
        #check
        ##itemTreeView.checkItem(bodyRTC)

        if 'name' in body_rtc_conf:
            robotItem.setName(objname) ## rename

    simulator = world.findItem("AISTSimulator")
    if simulator == None:
        simulator = AISTSimulatorItem()
        world.addChildItem(simulator)
        itemTreeView.checkItem(simulator)

    if vision_conf:
        vsim = GLVisionSimulatorItem()
        vsim.setTargetBodies(objname)
        if 'namelist' in vision_conf:
            vsim.setTargetSensors(vision_conf['namelist'])
        vsim.setMaxFrameRate(1000)
        vsim.setMaxLatency(0)
        vsim.setVisionDataRecordingEnabled(False)
        ## vsim.setThreadEnabled(True)
        vsim.setDedicatedSensorThreadsEnabled(True)
        vsim.setBestEffortMode(True)
        vsim.setRangeSensorPrecisionRatio(2.0)
        vsim.setAllSceneObjectsEnabled(False)
        vsim.setHeadLightEnabled(True)
        vsim.setAdditionalLightsEnabled(True)
        #add
        simulator.addChildItem(vsim)

def addScriptItem(world, obj_conf, filename):
    global itemTreeView
    script = PythonSimScriptItem()
    script.load(filename)
    script.setExecutionTiming(SimulationScriptItem.ExecutionTiming.AFTER_INITIALIZATION)
    script.setBackgroundMode(False)
    world.addChildItem(script)
    itemTreeView.checkItem(script)

##
## main
##

itemTreeView = ItemTreeView.instance()
rootItem = RootItem.instance()

world = rootItem.findItem("World")
if world == None:
    world = WorldItem()
    rootItem.addChildItem(world)

itemTreeView.checkItem(world)

robotname = None
for obj_name in dict_objs:
    obj_conf = dict_objs[obj_name]
    if isinstance(obj_conf, dict):
        obj_type = 'object'
        filename = None
        objname = None
        if 'type' in obj_conf:
            obj_type = obj_conf['type']

        if 'file' in obj_conf:
            filename = parse_filename(obj_conf['file'])

        if 'name' in obj_conf:
            objname = obj_conf['name']
        else:
            objname = obj_name

        if obj_type == 'object':
            addObjectItem(world, obj_conf, filename, objname)
        elif obj_type == 'robot':
            addRobotItem(world, obj_conf, filename, objname)
            robotname = objname
        elif obj_type == 'script':
            addScriptItem(world, obj_conf, filename)

if 'start_simulation' in dict_objs:
    if dict_objs['start_simulation']:
        sim = RootItem.instance().findItem('AISTSimulator')
        sim.setRealtimeSyncMode(False)
        itemTreeView.selectItem(sim)
        ## itemTreeView.sigSelectionChanged()
        if sim:
            sim.startSimulation(True)
