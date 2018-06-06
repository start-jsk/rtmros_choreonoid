import cnoid.Base
import cnoid.BodyPlugin
import cnoid.Util
import numpy, time, math

def resetPosition():
    robotname = "JAXON_RED"
    pos = [0, 0, 1.00]
    rpy = [0, 0, 0]

    robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    simulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)

    mat = cnoid.Util.rotFromRpy(rpy)

    trs = numpy.array([[mat[0][0], mat[0][1], mat[0][2], pos[0]],
                       [mat[1][0], mat[1][1], mat[1][2], pos[1]],
                       [mat[2][0], mat[2][1], mat[2][2], pos[2]]])

    simulatorItem.setForcedPosition(robotItem, trs)
    time.sleep(0.2) ##
    simulatorItem.clearForcedPositions()

resetPosition()
