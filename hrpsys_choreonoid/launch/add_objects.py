from __future__ import print_function ### for python2 compatible with python3

from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.OpenRTMPlugin import *
from cnoid.PythonSimScriptPlugin import *
import cnoid.Body

import math
import os
import sys
import yaml

import re
import subprocess ## subprocess is recommended but choreonoid crashed when calling rospack find
#import commands

try:
    rname = os.environ['ROBOT']
except:
    rname = None
    print("environment variable 'ROBOT' is not found", file=sys.stderr)

try:
    objs_yaml = os.environ['EXTRA_CHOREONOID_OBJS']
except:
    print("environment variable 'EXTRA_CHOREONOID_OBJS' is not found",  file=sys.stderr)
    raise
### sample yaml
#obj1:
#  name: 'MOGE_DOORWALL'
#  file: '/home/leus/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/jvrc_models/models/door_wallMain.wrl'
#  translation: [0, 0, 4]
#  rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
#obj2:
#  name: 'HOGE_FLOOR'
#  file: '/home/leus/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/jvrc_models/models/visible_floor.wrl'
#  translation: [0, 0, -0.1]
#  rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
#  static: true

def parse_filename(filestr):
    re_find = re.compile(r'\$\(find ([^ ]+)\)')
    ret = re_find.search(filestr)
    if ret != None:
        pkgname = ret.group(1)
        #packagepath = commands.getoutput('rospack find %s'%(pkgname))
        packagepath = subprocess.check_output(['rospack', 'find', pkgname])
        packagepath = packagepath.rstrip('\n')
        filestr = filestr[:ret.start(0)] + packagepath + filestr[ret.end(0):]

    return filestr

try:
    f = open(objs_yaml, 'r')
    dict_objs = yaml.load(f)
    f.close()
except:
    print("can not read %s"%(objs_yaml), file=sys.stderr)
    raise

sci_path = os.path.abspath(os.path.dirname(__file__))

if callable(ItemTreeView.instance):
    itemTreeView = ItemTreeView.instance()
else:
    itemTreeView = ItemTreeView.instance

if callable(RootItem.instance):
    rootItem = RootItem.instance()
else:
    rootItem = RootItem.instance

world = rootItem.findItem("World")
if world:
    for obj_name in dict_objs:
        obj_info = dict_objs[obj_name]
        if 'file' in obj_info:
            filename = parse_filename(obj_info['file'])
        else:
            continue

        if 'name' in obj_info:
            objname = obj_info['name']
        else:
            objname = obj_name

        if filename[0] != '/' and filename[0] != '$':
            filename = "%s/%s"%(sci_path, filename)

        if not os.path.exists(filename):
            print("file: %s not found"%(filename), file=sys.stderr)
            raise

        robotItem = BodyItem()
        robotItem.load(filename)
        robotItem.setName(objname)
        if callable(robotItem.body):
            robot = robotItem.body()
        else:
            robot = robotItem.body

        if callable(robot.rootLink):
            robot_rootLink = robot.rootLink()
        else:
            robot_rootLink = robot.rootLink

        if 'static' in obj_info:
            static = obj_info['static']
            if static:
                robot_rootLink.setJointType(cnoid.Body.Link.JointType.FIXED_JOINT)
                robot.updateLinkTree()
            else:
                robot_rootLink.setJointType(cnoid.Body.Link.JointType.FREE_JOINT)
                robot.updateLinkTree()

        if 'static_joint' in obj_info:
            static_joint = obj_info['static_joint']
            if static_joint:
                if callable(robot.numAllJoints): # include virtual joints
                    for i in range(robot.numAllJoints()):
                        robot.joint(i).setJointType(cnoid.Body.Link.JointType.FIXED_JOINT)
                else:
                    for i in range(robot.numAllJoints):
                        robot.joint(i).setJointType(cnoid.Body.Link.JointType.FIXED_JOINT)
                robot.updateLinkTree()

        if 'static_joints' in obj_info:
            static_joints = obj_info['static_joints']
            for j in static_joints:
                if robot.link(j):
                    robot.link(j).setJointType(cnoid.Body.Link.JointType.FIXED_JOINT)
                else:
                    print("joint: %s not found"%(j), file=sys.stderr)
            robot.updateLinkTree()

        if 'translation' in obj_info:
            trans = obj_info['translation']
            robot_rootLink.setTranslation(trans);

        if 'rotation' in obj_info:
            rot = obj_info['rotation']
            robot_rootLink.setRotation(rot);

        if callable(robot.numJoints):
            for i in range(robot.numJoints()):
                robot.joint(i).q = 0
        else:
            for i in range(robot.numJoints):
                robot.joint(i).q = 0

        robot.calcForwardKinematics()
        robotItem.storeInitialState()
        if callable(world.childItem):
            world.insertChildItem(robotItem, world.childItem())
        else:
            world.insertChildItem(robotItem, world.childItem)
        itemTreeView.checkItem(robotItem)
