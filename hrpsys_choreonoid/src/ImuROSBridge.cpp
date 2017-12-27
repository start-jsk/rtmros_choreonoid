// -*- C++ -*-
/*!
 * @file  ImuROSBridge.cpp * @brief openhrp image - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "ImuROSBridge.h"
#include <hrpUtil/EigenTypes.h>
#include <hrpUtil/Eigen3d.h>

#include "sensor_msgs/Imu.h"
#include "rosgraph_msgs/Clock.h" // for clock

// Module specification
// <rtc-template block="module_spec">
static const char* transformrosbridge_spec[] =
  {
    "implementation_id", "ImuROSBridge",
    "type_name",         "ImuROSBridge",
    "description",       "convert transform from OpenRTM to ROS",
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

ImuROSBridge::ImuROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_gsensor_in("gsensorIn", m_gsensor),
    m_gyrometer_in("gyrometerIn", m_gyrometer),
    nh(),
    prev_stamp_(ros::Time(0)),
    pub_cycle_(0),
    frame_id_("imu"),
    publish_clock_(false)
{
}

ImuROSBridge::~ImuROSBridge()
{
}


RTC::ReturnCode_t ImuROSBridge::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("gsensorIn", m_gsensor_in);
  addInPort("gyrometerIn", m_gyrometer_in);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);

  if(ros::param::has("~rate")) {
    double rate;
    ros::param::get("~rate", rate);
    if(rate != 0) {
      pub_cycle_ = 1/rate;
    }
  }

  if(ros::param::has("~frame_id")) {
    ros::param::get("~frame_id", frame_id_);
  }
  if(ros::param::has("~publish_clock")) {
    ros::param::get("~publish_clock", publish_clock_);
  }

  if (publish_clock_) {
    clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
  }
  // initialize
  ROS_INFO_STREAM("[ImuROSBridge] @Initilize name : " << getInstanceName());

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t ImuROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImuROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImuROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImuROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImuROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ImuROSBridge::onExecute(RTC::UniqueId ec_id)
{
  //capture_time = ros::Time::now();
  if (m_gsensor_in.isNew() && m_gyrometer_in.isNew()) {

    m_gsensor_in.read();
    m_gyrometer_in.read();

    ros::Time current_stamp;
    if ((m_gsensor.tm.sec != m_gyrometer.tm.sec) ||
        (m_gsensor.tm.nsec != m_gyrometer.tm.nsec)) {
      // error ??
      //std::cerr << "error" << std::endl;
      //std::cerr << "accel: " << m_gsensor.tm.sec << "/" << m_gsensor.tm.nsec << std::endl;
      //std::cerr << "gyro:  " << m_gyrometer.tm.sec << "/" << m_gyrometer.tm.nsec << std::endl;
      ros::Time gs = ros::Time(m_gsensor.tm.sec, m_gsensor.tm.nsec);
      ros::Time gy = ros::Time(m_gyrometer.tm.sec, m_gyrometer.tm.nsec);
      if (gy > gs) {
        current_stamp = gy;
      } else {
        current_stamp = gs;
      }
    } else {
      current_stamp = ros::Time(m_gsensor.tm.sec, m_gsensor.tm.nsec);
    }

    if (publish_clock_) {
      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = current_stamp;
      clock_pub.publish(clock_msg);
    }

    sensor_msgs::Imu imu;
    imu.header.frame_id = frame_id_;
    imu.header.stamp = current_stamp;

    imu.angular_velocity.x = m_gyrometer.data.avx;
    imu.angular_velocity.y = m_gyrometer.data.avy;
    imu.angular_velocity.z = m_gyrometer.data.avz;

    imu.linear_acceleration.x = m_gsensor.data.ax;
    imu.linear_acceleration.y = m_gsensor.data.ay;
    imu.linear_acceleration.z = m_gsensor.data.az;

    for(int i = 0; i < 9; i++) {
      imu.orientation_covariance[i] = 0;
      imu.angular_velocity_covariance[i] = 0;
      imu.linear_acceleration_covariance[i] = 0;
    }
    //imu.orientation_covariance[0]         = 2.89e-08;
    imu.orientation_covariance[0]         = 7e-5; // sigma = 0.5 deg
    imu.orientation_covariance[4]         = 7e-5;
    imu.orientation_covariance[8]         = 7e-5;
    //imu.angular_velocity_covariance[0]    = 0.000144;
    imu.angular_velocity_covariance[0]    = 1e-4; // sigma = 0.01
    imu.angular_velocity_covariance[4]    = 1e-4;
    imu.angular_velocity_covariance[8]    = 1e-4;
    //imu.linear_acceleration_covariance[0] = 0.0096;
    imu.linear_acceleration_covariance[0] = 4e-4;
    imu.linear_acceleration_covariance[4] = 4e-4;
    imu.linear_acceleration_covariance[8] = 4e-4;
    imu_pub.publish(imu);

    prev_stamp_ = current_stamp;
  }

  //
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ImuROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImuROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImuROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImuROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImuROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C"
{
  void ImuROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(transformrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<ImuROSBridge>,
                             RTC::Delete<ImuROSBridge>);
  }
};
