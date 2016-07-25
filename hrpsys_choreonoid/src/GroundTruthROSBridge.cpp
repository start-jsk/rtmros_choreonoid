// -*- C++ -*-
/*!
 * @file  GroundTruthROSBridge.cpp * @brief openhrp image - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "GroundTruthROSBridge.h"
#include <hrpUtil/EigenTypes.h>
#include <hrpUtil/Eigen3d.h>

// Module specification
// <rtc-template block="module_spec">
static const char* imagesensorrosbridge_spec[] =
  {
    "implementation_id", "GroundTruthROSBridge",
    "type_name",         "GroundTruthROSBridge",
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

GroundTruthROSBridge::GroundTruthROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_baseTformIn("baseTform", m_baseTform),
    prev_stamp(ros::Time(0)),
    is_initialized(false),
    pub_cycle(0)
{
}

GroundTruthROSBridge::~GroundTruthROSBridge()
{
}


RTC::ReturnCode_t GroundTruthROSBridge::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("baseTform", m_baseTformIn);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  ground_truth_odom_pub = node.advertise<nav_msgs::Odometry>("ground_truth_odom", 1);

  if(ros::param::has("~rate")) {
    double rate;
    ros::param::get("~rate", rate);
    if(rate != 0) {
      pub_cycle = 1/rate;
    }
  }
  // initialize
  ROS_INFO_STREAM("[GroundTruthROSBridge] @Initilize name : " << getInstanceName());

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t GroundTruthROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t GroundTruthROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t GroundTruthROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t GroundTruthROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t GroundTruthROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t GroundTruthROSBridge::onExecute(RTC::UniqueId ec_id)
{
  //capture_time = ros::Time::now();

  if (m_baseTformIn.isNew()) {
    m_baseTformIn.read();
    
    tf::Transform current_base;
    convertBaseTformToTfTransform(current_base);
    
    ros::Time current_stamp = ros::Time(m_baseTform.tm.sec, m_baseTform.tm.nsec);

    if ((pub_cycle != 0) && ((current_stamp - prev_stamp).toSec() < pub_cycle)) {
      return RTC::RTC_OK;
    }
    
    nav_msgs::Odometry ground_truth_odom;
    ground_truth_odom.header.stamp = current_stamp;
    tf::Vector3 current_origin = current_base.getOrigin();
    ground_truth_odom.pose.pose.position.x = current_origin[0];
    ground_truth_odom.pose.pose.position.y = current_origin[1];
    ground_truth_odom.pose.pose.position.z = current_origin[2];
    tf::quaternionTFToMsg(current_base.getRotation(), ground_truth_odom.pose.pose.orientation);

    ground_truth_odom_pub.publish(ground_truth_odom);

    prev_base = current_base;
    prev_stamp = current_stamp;
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GroundTruthROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t GroundTruthROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t GroundTruthROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t GroundTruthROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t GroundTruthROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void GroundTruthROSBridge::convertBaseTformToTfTransform(tf::Transform& base)
{
  // baseTform: x, y, z, R[0][0], R[0][1], ... , R[2][2]
  double *data = m_baseTform.data.get_buffer();
  tf::Vector3 base_origin(data[0], data[1], data[2]);
  base.setOrigin(base_origin);
  hrp::Matrix33 hrpsys_R;
  hrp::getMatrix33FromRowMajorArray(hrpsys_R, data, 3);
  hrp::Vector3 hrpsys_rpy = hrp::rpyFromRot(hrpsys_R);
  tf::Quaternion base_q = tf::createQuaternionFromRPY(hrpsys_rpy(0), hrpsys_rpy(1), hrpsys_rpy(2));
  base.setRotation(base_q);
  return;
}


extern "C"
{
 
  void GroundTruthROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(imagesensorrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<GroundTruthROSBridge>,
                             RTC::Delete<GroundTruthROSBridge>);
  }
  
};
