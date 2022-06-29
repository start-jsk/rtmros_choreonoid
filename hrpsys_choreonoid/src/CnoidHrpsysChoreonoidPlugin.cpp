#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include "ClockPublisherItem.h"
#include "ClockShmItem.h"
#include "DepthCameraPublisherItem.h"
#include "OdometryPublisherItem.h"

using namespace cnoid;

class HrpsysChoreonoidPlugin : public Plugin
{
public:

    HrpsysChoreonoidPlugin() : Plugin("HrpsysChoreonoid")
    {
      require("Body");
    }

    virtual bool initialize() override
    {
      ClockPublisherItem::initializeClass(this);
      ClockShmItem::initializeClass(this);
      DepthCameraPublisherItem::initializeClass(this);
      OdometryPublisherItem::initializeClass(this);
      return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(HrpsysChoreonoidPlugin)
