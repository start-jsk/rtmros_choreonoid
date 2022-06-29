#ifndef CLOCKSHM_ITEM_H
#define CLOCKSHM_ITEM_H

#include <cnoid/Item>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/ConnectionSet>
#include <sys/time.h>

namespace cnoid {

  class ClockShmItem : public Item
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    ClockShmItem();

  protected:
    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();

    SimulatorItem* currentSimulatorItem_;
    ScopedConnectionSet currentSimulatorItemConnections_;
    struct timeval* c_shm;
  };

  typedef ref_ptr<ClockShmItem> ClockShmItemPtr;
}

#endif
