import cnoid.Base
import cnoid.BodyPlugin
import cnoid.Util
import numpy, time, math

import sys
setattr(sys, 'argv', [''])

import rospy
from roseus.srv import *

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
    robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    if robotItem == None:
        return 'invalid robotname %s'%(robotname)

    simulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
    if simulatorItem == None:
        return 'invalid simulatorItem'

    pushingLink = robotItem.body().link(linkname)
    if pushingLink:
        simulatorItem.setExternalForce(robotItem, pushingLink, pos, force, tm)
        return 'success'
    return 'invalid link: %s'%(linkname)

def resetPosition(robotname = "JAXON_RED", pos = [0,0,1.0], rpy = [0,0,0], sleep = 0.2):
    #resetPosition("JAXON_RED", [0, 0, 1.0], [0, 0, 0])
    robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    if robotItem == None:
        return 'invalid robotname %s'%(robotname)

    simulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
    if simulatorItem == None:
        return 'invalid simulatorItem'

    mat = cnoid.Util.rotFromRpy(rpy)

    trs = numpy.array([[mat[0][0], mat[0][1], mat[0][2], pos[0]],
                       [mat[1][0], mat[1][1], mat[1][2], pos[1]],
                       [mat[2][0], mat[2][1], mat[2][2], pos[2]]])

    simulatorItem.setForcedPosition(robotItem, trs)
    ##
    time.sleep(sleep) ##
    simulatorItem.clearForcedPositions()

    return 'success'

ros_service_init()
