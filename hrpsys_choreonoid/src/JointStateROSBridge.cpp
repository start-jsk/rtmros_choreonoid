// -*- C++ -*-
/*!
 * @file  JointStateROSBridge.cpp * @brief openhrp image - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "JointStateROSBridge.h"
#include <algorithm>

// Module specification
// <rtc-template block="module_spec">
static const char* jointstaterosbridge_spec[] =
  {
    "implementation_id", "JointStateROSBridge",
    "type_name",         "JointStateROSBridge",
    "description",       "rtm range data - ros bridge",
    "version",           "1.0",
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

JointStateROSBridge::JointStateROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleIn("qRef", m_angle),
    prev_stamp(ros::Time(0)),
    pub_cycle(0)
{
}

JointStateROSBridge::~JointStateROSBridge()
{
}


RTC::ReturnCode_t JointStateROSBridge::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_angleIn);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  jstate_pub = node.advertise<sensor_msgs::JointState>("joint_states", 10);

  if(ros::param::has("~names")) {
    //std::vector<std::string> name_list;
    ros::param::get("~names", joint_name_list);

    ROS_INFO_STREAM("[JointStateROSBridge] @Initilize joint_name_list :");
    for(int i = 0; i < joint_name_list.size(); i++) {
      ROS_INFO_STREAM(i << " : " << joint_name_list[i]);
    }
  }

  if(ros::param::has("~rate")) {
    double rate;
    ros::param::get("~rate", rate);
    if(rate != 0) {
      pub_cycle = 1/rate;
    }
  }
  // initialize
  ROS_INFO_STREAM("[JointStateROSBridge] @Initilize name : " << getInstanceName());

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t JointStateROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t JointStateROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t JointStateROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t JointStateROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t JointStateROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t JointStateROSBridge::onExecute(RTC::UniqueId ec_id)
{
  //capture_time = ros::Time::now();

  if (m_angleIn.isNew()){ // m_range

    m_angleIn.read();
    //
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time(m_angle.tm.sec, m_angle.tm.nsec);

    if ((pub_cycle != 0) && (prev_stamp.toSec() != 0 ) && ((js.header.stamp - prev_stamp).toSec() < pub_cycle)) {
      return RTC::RTC_OK;
    }
    double tm = 0.0;
    if (prev_angle.data.length() == m_angle.data.length()) {
      tm = (m_angle.tm.sec - prev_angle.tm.sec) +
        0.000000001 * (m_angle.tm.nsec - prev_angle.tm.nsec);
    }
    for(int i = 0; i < std::min((long)m_angle.data.length(),
                                (long)joint_name_list.size()); i++) {
       js.name.push_back(joint_name_list[i]);
       js.position.push_back(m_angle.data[i]);
       if (tm != 0.0) {
         js.velocity.push_back((m_angle.data[i] - prev_angle.data[i])/tm);
       }
       //js.effort.push_back();
    }
    jstate_pub.publish(js);

    prev_angle = m_angle;
    prev_stamp = js.header.stamp;
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t JointStateROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t JointStateROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t JointStateROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t JointStateROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t JointStateROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void JointStateROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(jointstaterosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<JointStateROSBridge>,
                             RTC::Delete<JointStateROSBridge>);
  }
  
};
