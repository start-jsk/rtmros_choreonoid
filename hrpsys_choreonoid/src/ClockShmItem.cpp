#include "ClockShmItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <sys/shm.h>

namespace cnoid {

  void ClockShmItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<ClockShmItem>("ClockShmItem");
  }

  ClockShmItem::ClockShmItem(){
    SimulationBar::instance()->sigSimulationAboutToStart().connect([&](SimulatorItem* simulatorItem){onSimulationAboutToStart(simulatorItem);});

    int shm_id = shmget(969, sizeof(struct timeval), 0666|IPC_CREAT);
    if(shm_id == -1) {
      MessageView::instance()->cout() << "\e[0;31m" << "[ClockShmItem] shmget failed"  << "\e[0m" << std::endl;
      return;
    }
    this->c_shm = (struct timeval *)shmat(shm_id, (void *)0, 0);
    if(this->c_shm == (void*)-1) {
      MessageView::instance()->cout() << "\e[0;31m" << "[ClockShmItem] shmat failed"  << "\e[0m" << std::endl;
    }
  }

  void ClockShmItem::onSimulationAboutToStart(SimulatorItem* simulatorItem)
  {
    this->currentSimulatorItem_ = simulatorItem;
    this->currentSimulatorItemConnections_.add(
        simulatorItem->sigSimulationStarted().connect(
            [&](){ onSimulationStarted(); }));
  }

  void ClockShmItem::onSimulationStarted()
  {
    this->currentSimulatorItem_->addPreDynamicsFunction([&](){ onSimulationStep(); });
  }

  void ClockShmItem::onSimulationStep()
  {
    if(!this->c_shm) return;

    double timestep = this->currentSimulatorItem_->worldTimeStep();
    int frame = this->currentSimulatorItem_->simulationFrame();

    unsigned long timestep_usec = timestep * 1000000;
    unsigned long long time_usec = frame * timestep_usec;

    c_shm->tv_sec = time_usec / 1000000;
    c_shm->tv_usec = time_usec - c_shm->tv_sec * 1000000;
  }
}

