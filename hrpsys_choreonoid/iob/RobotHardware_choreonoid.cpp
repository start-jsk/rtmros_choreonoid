#include "RobotHardware_choreonoid.h"

#include <hrpUtil/EigenTypes.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
// Module specification
// <rtc-template block="module_spec">
static const char* robothardware_choreonoid_spec[] =
  {
    "implementation_id", "RobotHardware_choreonoid",
    "type_name",         "RobotHardware_choreonoid",
    "description",       "RobotHardware_choreonoid",
    "version",           "0.0.1",
    "vendor",            "JSK (hrpsys_choreonoid)",
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

extern RobotHardware_choreonoid *self_ptr;
extern void iob_update();
extern void iob_finish();
extern void iob_set_torque_limit(std::vector<double> &vec);

RobotHardware_choreonoid::RobotHardware_choreonoid(RTC::Manager* manager)
  : RobotHardware(manager)
{
}

RTC::ReturnCode_t RobotHardware_choreonoid::onInitialize()
{
  std::cerr << "[rh choreonoid initialize]" << std::endl;
  self_ptr = this;
  RTC::ReturnCode_t ret = RobotHardware::onInitialize();
  std::vector<double> vec;
  {
    robot *m_robot = this->robot_ptr();
    vec.resize(m_robot->numJoints());
    for(int i = 0; i < vec.size(); i++) {
      vec[i] =
        m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
    }
  }
  iob_set_torque_limit(vec);
  std::cerr << "[rh choreonoid initialize iob_set_torque_limit]" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RobotHardware_choreonoid::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[rh choreonoid activate]" << std::endl;
  iob_update();
  std::cerr << "[rh choreonoid activate iob_update]" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RobotHardware_choreonoid::onExecute(RTC::UniqueId ec_id)
{
  //std::cerr << "[rh choreonoid exe]" << std::endl;
  iob_update();
  //std::cerr << "[rh choreonoid exe / updated]" << std::endl;
  RTC::ReturnCode_t ret = RobotHardware::onExecute(ec_id);
  //std::cerr << "[rh choreonoid exe] ret: " << ret << " / " << RTC::RTC_OK << std::endl;
  iob_finish();
  return ret;
}

extern "C"
{

  void RobotHardware_choreonoidInit(RTC::Manager* manager)
  {
    RTC::Properties profile(robothardware_choreonoid_spec);
    manager->registerFactory(profile,
                             RTC::Create<RobotHardware_choreonoid>,
                             RTC::Delete<RobotHardware_choreonoid>);
  }

};



