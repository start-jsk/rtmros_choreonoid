import cnoid.Base
import cnoid.BodyPlugin

def addExternalForce():
    robotname = "JAXON_RED"
    linkname = "WAIST"
    pos   = [0, 0, 0.2] ## link local position
    force = [100, 0, 0] ## [ N ]
    tm = 1 ## [ sec ]

    robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    simulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
    pushingLink = robotItem.body().link(linkname)
    if pushingLink:
        simulatorItem.setExternalForce(robotItem, pushingLink, pos, force, tm)

addExternalForce()
