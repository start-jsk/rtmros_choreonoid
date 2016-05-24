#ifndef ROBOT_HARDWARE_C_H
#define ROBOT_HARDWARE_C_H

#include <RobotHardware.h>

// Module specification
// <rtc-template block="module_spec">
static const char* robothardware_spec[] =
  {
    "implementation_id", "RobotHardware",
    "type_name",         "RobotHardware",
    "description",       "RobotHardware",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.isDemoMode", "0",
    "conf.default.fzLimitRatio", "2.0",
    "conf.default.servoErrorLimit", ",",
    "conf.default.jointAccelerationLimit", "0",

    ""
  };
// </rtc-template>

extern Time iob_time;

class RobotHardware_choreonoid : public RobotHardware
{
public:
  RobotHardware_choreonoid(RTC::Manager* manager);

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  virtual inline void getTimeNow(Time &tm) {
      tm.sec  = iob_time.sec;
      tm.nsec = iob_time.nsec;
  };
};

extern "C"
{
  void RobotHardware_choreonoidInit(RTC::Manager* manager);
};

#endif // ROBOT_HARDWARE_C_H
