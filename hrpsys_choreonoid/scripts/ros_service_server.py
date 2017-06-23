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
        print 'service called: %s'%(req.str)
        ret = eval(req.str)
        print 'service result: %s'%(ret)
    return StringStringResponse(ret)

def ros_service_init ():
    rospy.init_node('choreonoid_ros')
    s = rospy.Service('/choreonoid_service', StringString, roscallback)
    ## rospy.spin()

def addExternalForce(robotname = "JAXON_RED", linkname = "WAIST", pos = [0,0,1.0], force = [100,0,0], tm = 0.2):
    #addExternalForce("JAXON_RED", "WAIST", [0, 0, 1.0], [100, 0, 0], 0.2)
    global thisSimulatorItem

    robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    if robotItem == None:
        return 'invalid robotname %s'%(robotname)

    if thisSimulatorItem == None:
        thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
        if thisSimulatorItem == None:
            return 'invalid simulatorItem'

    pushingLink = robotItem.body().link(linkname)
    if pushingLink:
        thisSimulatorItem.setExternalForce(robotItem, pushingLink, pos, force, tm)
        return 'success'

    return 'invalid link: %s'%(linkname)

def resetPosition(robotname = "JAXON_RED", pos = [0,0,1.0], rpy = [0,0,0], sleep = 0.2):
    #resetPosition("JAXON_RED", [0, 0, 1.0], [0, 0, 0])
    global thisSimulatorItem

    robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    if robotItem == None:
        return 'invalid robotname %s'%(robotname)

    if thisSimulatorItem == None:
        thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
        if thisSimulatorItem == None:
            return 'invalid simulatorItem'

    mat = cnoid.Util.rotFromRpy(rpy)

    trs = numpy.array([[mat[0][0], mat[0][1], mat[0][2], pos[0]],
                       [mat[1][0], mat[1][1], mat[1][2], pos[1]],
                       [mat[2][0], mat[2][1], mat[2][2], pos[2]]])

    thisSimulatorItem.setForcedPosition(robotItem, trs)
    ##
    time.sleep(sleep) ##
    thisSimulatorItem.clearForcedPositions()

    return 'success'

def addObject(objname, filename, translation = None, rotation = None):
    global thisSimulatorItem

    itemTreeView = cnoid.Base.ItemTreeView.instance()
    rootItem = cnoid.Base.RootItem.instance()

    world = rootItem.findItem("World")
    if world == None:
        return 'world not found'

    robotItem = cnoid.BodyPlugin.BodyItem()
    robotItem.load(filename)
    robotItem.setName(objname)

    robot = robotItem.body()
    if robot == None:
        return 'file not found'

    if translation:
        robot.rootLink().setTranslation(translation);
    if rotation:
        robot.rootLink().setRotation(rotation);

    for i in range(robot.numJoints()):
        robot.joint(i).q = 0

    robot.calcForwardKinematics()
    robotItem.storeInitialState()
    world.insertChildItem(robotItem, world.childItem())
    itemTreeView.checkItem(robotItem)

    return 'success'

def callSimulation(robotname = "JAXON_RED", call = 'pauseSimulation'):
    global thisSimulatorItem

    robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    if robotItem == None:
        return 'invalid robotname %s'%(robotname)

    if thisSimulatorItem == None:
        thisSimulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
        if thisSimulatorItem == None:
            return 'invalid simulatorItem'

    ret = eval('thisSimulatorItem.%s()'%(call))

    return '(:return %s)'%(ret)

ros_service_init()
