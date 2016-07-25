// -*- C++ -*-
/*!
 * @file  GroundTruthROSBridge.cpp * @brief openhrp image - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "GroundTruthROSBridge.h"
#include <hrpUtil/EigenTypes.h>
#include <hrpUtil/Eigen3d.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

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
    ros::Time current_stamp = ros::Time(m_baseTform.tm.sec, m_baseTform.tm.nsec);
    convertBaseTformToTfTransform(current_base);
    double dt = (current_stamp - prev_stamp).toSec();

    // skip for publish rate
    if (pub_cycle != 0 && dt < pub_cycle) {
      return RTC::RTC_OK;
    }

    if (is_initialized == false) {
      prev_base = current_base;
      is_initialized = true;
    }

    // calculate velocity
    tf::Vector3 linear_twist, angular_twist;
    calculateTwist(current_base, prev_base, linear_twist, angular_twist, dt);
    geometry_msgs::Twist current_twist;
    tf::vector3TFToMsg(linear_twist, current_twist.linear);
    tf::vector3TFToMsg(angular_twist, current_twist.angular);

    // register msg
    nav_msgs::Odometry ground_truth_odom;
    ground_truth_odom.header.stamp = current_stamp;
    tf::Vector3 current_origin = current_base.getOrigin();
    ground_truth_odom.pose.pose.position.x = current_origin[0];
    ground_truth_odom.pose.pose.position.y = current_origin[1];
    ground_truth_odom.pose.pose.position.z = current_origin[2];
    tf::quaternionTFToMsg(current_base.getRotation(), ground_truth_odom.pose.pose.orientation);
    ground_truth_odom.twist.twist = current_twist;

    ground_truth_odom_pub.publish(ground_truth_odom);

    // preserve values for velocity calculation in next step
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

void GroundTruthROSBridge::convertBaseTformToTfTransform(tf::Transform& result_base)
{
  // baseTform: x, y, z, R[0][0], R[0][1], ... , R[2][2]
  double *data = m_baseTform.data.get_buffer();
  tf::Vector3 base_origin(data[0], data[1], data[2]);
  result_base.setOrigin(base_origin);
  hrp::Matrix33 hrpsys_R;
  hrp::getMatrix33FromRowMajorArray(hrpsys_R, data, 3);
  hrp::Vector3 hrpsys_rpy = hrp::rpyFromRot(hrpsys_R);
  tf::Quaternion base_q = tf::createQuaternionFromRPY(hrpsys_rpy(0), hrpsys_rpy(1), hrpsys_rpy(2));
  result_base.setRotation(base_q);
  return;
}

void GroundTruthROSBridge::calculateTwist(const tf::Transform& _current_base, const tf::Transform& _prev_base, tf::Vector3& _linear_twist, tf::Vector3& _angular_twist, double _dt)
{
  // current rotation matrix
  tf::Matrix3x3 current_basis = _current_base.getBasis();
  
  // linear twist
  tf::Vector3 current_origin = _current_base.getOrigin();
  tf::Vector3 prev_origin = _prev_base.getOrigin();
  _linear_twist = current_basis.transpose() * ((current_origin - prev_origin) / _dt);

  // angular twist
  // R = exp(omega_w*dt) * prev_R
  // omega_w is described in global coordinates in relationships of twist transformation.
  // it is easier to calculate omega using hrp functions than tf functions
  tf::Matrix3x3 prev_basis = _prev_base.getBasis();
  double current_rpy[3], prev_rpy[3];
  current_basis.getRPY(current_rpy[0], current_rpy[1], current_rpy[2]);
  prev_basis.getRPY(prev_rpy[0], prev_rpy[1], prev_rpy[2]);
  hrp::Matrix33 current_hrpR = hrp::rotFromRpy(current_rpy[0], current_rpy[1], current_rpy[2]);
  hrp::Matrix33 prev_hrpR = hrp::rotFromRpy(prev_rpy[0], prev_rpy[1], prev_rpy[2]);
  hrp::Vector3 hrp_omega = current_hrpR.transpose() * hrp::omegaFromRot(current_hrpR * prev_hrpR.transpose()) / _dt;
  _angular_twist.setX(hrp_omega[0]);
  _angular_twist.setY(hrp_omega[1]);
  _angular_twist.setZ(hrp_omega[2]);
  
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
