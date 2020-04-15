from __future__ import print_function ### for python2 compatible with python3

import cnoid.Base
import cnoid.BodyPlugin
import cnoid.Util
#from cnoid.OpenRTMPlugin import *
#from cnoid.PythonSimScriptPlugin import *
import numpy, time, math

import sys
setattr(sys, 'argv', [''])

import rospy
from roseus.srv import *

thisSimulatorItem = None

def roscallback (req):
    ret = 'not called'
    if (len(req.str)) > 0:
        print('service called: %s'%(req.str))
        ret = eval(req.str)
        print('service result: %s'%(ret))
    return StringStringResponse(ret)

def ros_service_init ():
    rospy.init_node('choreonoid_ros')
    s = rospy.Service('/choreonoid_service', StringString, roscallback)
    ## rospy.spin()

def addExternalForce(robotname = "JAXON_RED", linkname = "WAIST", pos = [0,0,1.0], force = [100,0,0], tm = 0.2):
    #addExternalForce("JAXON_RED", "WAIST", [0, 0, 1.0], [100, 0, 0], 0.2)
    global thisSimulatorItem

    if callable(cnoid.Base.RootItem.instance):
        robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    else:
        robotItem = cnoid.Base.RootItem.instance.findItem(robotname)
    if robotItem == None:
        return '(:fail "invalid robotname %s")'%(robotname)

    if thisSimulatorItem == None:
        thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
        if thisSimulatorItem == None:
            return '(:fail "invalid simulatorItem")'

    if callable(robotItem.body):
        pushingLink = robotItem.body().link(linkname)
    else:
        pushingLink = robotItem.body.link(linkname)
    if pushingLink:
        thisSimulatorItem.setExternalForce(robotItem, pushingLink, pos, force, tm)
        return '(:success)'

    return '(:fail "invalid link: %s")'%(linkname)

def resetPosition(robotname = "JAXON_RED", pos = [0,0,1.0], rpy = [0,0,0], sleep = 0.2):
    #resetPosition("JAXON_RED", [0, 0, 1.0], [0, 0, 0])
    global thisSimulatorItem
    if callable(cnoid.Base.RootItem.instance):
        robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    else:
        robotItem = cnoid.Base.RootItem.instance.findItem(robotname)
    if robotItem == None:
        return '(:fail "invalid robotname %s")'%(robotname)

    if thisSimulatorItem == None:
        thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
        if thisSimulatorItem == None:
            return '(:fail "invalid simulatorItem")'

    mat = cnoid.Util.rotFromRpy(rpy)

    trs = numpy.array([[mat[0][0], mat[0][1], mat[0][2], pos[0]],
                       [mat[1][0], mat[1][1], mat[1][2], pos[1]],
                       [mat[2][0], mat[2][1], mat[2][2], pos[2]]])

    thisSimulatorItem.setForcedPosition(robotItem, trs)
    ##
    time.sleep(sleep) ##
    thisSimulatorItem.clearForcedPositions()

    return '(:success)'

def addObject(objname, filename, translation = None, rotation = None):
    global thisSimulatorItem

    if callable(cnoid.Base.ItemTreeView.instance):
        itemTreeView = cnoid.Base.ItemTreeView.instance()
    else:
        itemTreeView = cnoid.Base.ItemTreeView.instance
    if callable(cnoid.Base.RootItem.instance):
        rootItem = cnoid.Base.RootItem.instance()
    else:
        rootItem = cnoid.Base.RootItem.instance
    world = rootItem.findItem("World")
    if world == None:
        return '(:fail "world not found")'

    robotItem = cnoid.BodyPlugin.BodyItem()
    robotItem.load(filename)
    robotItem.setName(objname)

    if callable(robotItem.body):
        robot = robotItem.body()
    else:
        robot = robotItem.body
    if robot == None:
        return '(:fail "file not found")'

    if callable(robot.rootLink):
        robot_rootLink = robot.rootLink()
    else:
        robot_rootLink = robot.rootLink
    if translation:
        robot_rootLink.setTranslation(translation);
    if rotation:
        robot_rootLink.setRotation(rotation);

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

    return '(:success)'

def callSimulation(robotname = "JAXON_RED", call = 'pauseSimulation'):
    global thisSimulatorItem

    if callable(cnoid.Base.RootItem.instance):
        robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    else:
        robotItem = cnoid.Base.RootItem.instance.findItem(robotname)
    if robotItem == None:
        return '(:fail "invalid robotname %s")'%(robotname)

    if thisSimulatorItem == None:
        thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
        if thisSimulatorItem == None:
            return '(:fail "invalid simulatorItem")'

    ret = eval('thisSimulatorItem.%s()'%(call))

    return '(:success "%s")'%(ret)

def getCoordinate(robotname = 'JAXON_RED', linkname = 'WAIST'):
    if callable(cnoid.Base.RootItem.instance):
        robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    else:
        robotItem = cnoid.Base.RootItem.instance.findItem(robotname)
    if robotItem == None:
        return '(:fail "invalid robotname %s")'%(robotname)

    if callable(robotItem.body):
        iLink = robotItem.body().link(linkname)
    else:
        iLink = robotItem.body.link(linkname)

    if iLink == None:
        return '(:fail "invalid linkname %s")'%(linkname)
    if callable(iLink.position):
        coords = iLink.position()
    else:
        coords = iLink.position
    pos = coords[0:3, 3]
    rot = coords[0:3, 0:3]

    return '(:success (make-coords :pos #f(%f %f %f) :rot #2f((%f %f %f) (%f %f %f) (%f %f %f))))'%(pos[0]*1000,pos[1]*1000,pos[2]*1000,rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2])

ros_service_init()
