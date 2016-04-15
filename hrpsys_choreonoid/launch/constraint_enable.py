from cnoid.Base import *
from cnoid.BodyPlugin import *

#sr1 = Item.find("SR1").body()
#floorLink = Item.find("Floor").body().rootLink()
simulator = Item.find("AISTSimulator")

simulator.setConstraintForceOutputEnabled(True)
