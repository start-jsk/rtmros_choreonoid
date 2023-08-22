#ifndef CLOCKPUBLISHER_ITEM_H
#define CLOCKPUBLISHER_ITEM_H

#include <cnoid/Item>
#include <ros/ros.h>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/ConnectionSet>

namespace cnoid {

  class ClockPublisherItem : public Item
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    ClockPublisherItem();

  protected:
    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();

    ros::Publisher clockPublisher_;
    SimulatorItem* currentSimulatorItem_;
    ScopedConnectionSet currentSimulatorItemConnections_;
  };

  typedef ref_ptr<ClockPublisherItem> ClockPublisherItemPtr;
}

#endif
