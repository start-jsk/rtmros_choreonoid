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
#import subprocess ## subprocess is recommended but choreonoid crashed when calling rospack find
import commands

try:
    rname = os.environ['ROBOT']
except:
    rname = None
    print >> sys.stderr, "environment variable 'ROBOT' is not found"

try:
    objs_yaml = os.environ['EXTRA_CHOREONOID_OBJS']
except:
    print >> sys.stderr, "environment variable 'EXTRA_CHOREONOID_OBJS' is not found"
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
        packagepath = commands.getoutput('rospack find %s'%(pkgname))
        #packagepath = subprocess.check_output(['rospack', 'find', pkgname])
        filestr = filestr[:ret.start(0)] + packagepath + filestr[ret.end(0):]

    return filestr

try:
    f = open(objs_yaml, 'r')
    dict_objs = yaml.load(f)
    f.close()
except:
    print >> sys.stderr, "can not read %s"%(objs_yaml)
    raise

sci_path = os.path.abspath(os.path.dirname(__file__))

itemTreeView = ItemTreeView.instance()
rootItem = RootItem.instance()

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
            print >> sys.stderr, "file: %s not found"%(filename)
            raise

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

        if 'translation' in obj_info:
            trans = obj_info['translation']
            robot.rootLink().setTranslation(trans);

        if 'rotation' in obj_info:
            rot = obj_info['rotation']
            robot.rootLink().setRotation(rot);

        for i in range(robot.numJoints()):
            robot.joint(i).q = 0

        robot.calcForwardKinematics()
        robotItem.storeInitialState()
        world.insertChildItem(robotItem, world.childItem())
        itemTreeView.checkItem(robotItem)
