// -*- C++ -*-
/*!
 * @file  TransformROSBridge.cpp * @brief openhrp image - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "TransformROSBridge.h"
#include <hrpUtil/EigenTypes.h>
#include <hrpUtil/Eigen3d.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

// Module specification
// <rtc-template block="module_spec">
static const char* transformrosbridge_spec[] =
  {
    "implementation_id", "TransformROSBridge",
    "type_name",         "TransformROSBridge",
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

TransformROSBridge::TransformROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_TformIn("TformIn", m_Tform),
    nh(),
    publish_odom_(true),
    prev_stamp_(ros::Time(0)),
    is_initialized_(false),
    initial_relative_(true),
    pub_cycle_(0),
    publish_tf_(false),
    invert_tf_(false),
    tf_parent_frame_("world"), tf_frame_("self")
{
}

TransformROSBridge::~TransformROSBridge()
{
}


RTC::ReturnCode_t TransformROSBridge::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("TformIn", m_TformIn);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

  if(ros::param::has("~rate")) {
    double rate;
    ros::param::get("~rate", rate);
    if(rate != 0) {
      pub_cycle_ = 1/rate;
    }
  }
  if(ros::param::has("~initial_relative")) {
    ros::param::get("~initial_relative", initial_relative_);
  }
  if(ros::param::has("~publish_odom")) {
    ros::param::get("~publish_odom", publish_odom_);
  }
  if(ros::param::has("~publish_tf")) {
    ros::param::get("~publish_tf", publish_tf_);
  }
  if(ros::param::has("~invert_tf")) {
    ros::param::get("~invert_tf", invert_tf_);
  }
  if(ros::param::has("~tf_frame")) {
    ros::param::get("~tf_frame", tf_frame_);
  }
  if(ros::param::has("~tf_parent_frame")) {
    ros::param::get("~tf_parent_frame", tf_parent_frame_);
  }

  init_trans_.setOrigin(tf::Vector3(0, 0, 0));
  init_trans_.setRotation(tf::Quaternion(0, 0, 0, 1));
  prev_trans_.setOrigin(tf::Vector3(0, 0, 0));
  prev_trans_.setRotation(tf::Quaternion(0, 0, 0, 1));

  // initialize
  ROS_INFO_STREAM("[TransformROSBridge] @Initilize name : " << getInstanceName());

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t TransformROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TransformROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TransformROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TransformROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TransformROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TransformROSBridge::onExecute(RTC::UniqueId ec_id)
{
  //capture_time = ros::Time::now();
  if (m_TformIn.isNew()) {
    m_TformIn.read();

    tf::Transform current_trans;
    ros::Time current_stamp = ros::Time(m_Tform.tm.sec, m_Tform.tm.nsec);
    convertTformToTfTransform(current_trans);
    double dt = (current_stamp - prev_stamp_).toSec();

    // skip for publish rate
    if (pub_cycle_ != 0 && dt < pub_cycle_) {
      return RTC::RTC_OK;
    }

    // transform current_trans into initial trans relative coords
    if (initial_relative_)
    {
      if (is_initialized_ == false) {
        tf::Vector3 init_origin = current_trans.getOrigin();
        tf::Quaternion init_rotation = current_trans.getRotation();
        init_origin.setZ(0.0); // z origin should be on the ground
        init_trans_.setOrigin(init_origin);
        init_trans_.setRotation(init_rotation);
        current_trans = init_trans_.inverse() * current_trans;
        prev_trans_ = current_trans; // prev_base should be initilaized, too
        is_initialized_ = true;
      } else {
        current_trans = init_trans_.inverse() * current_trans;
      }
    } else {
      // do nothing
    }

    if(publish_tf_) {
      if (invert_tf_) {
        tf::Transform inv_trans = current_trans.inverse();
        br.sendTransform(tf::StampedTransform(inv_trans, current_stamp,
                                              tf_frame_, tf_parent_frame_));
      } else {
        br.sendTransform(tf::StampedTransform(current_trans, current_stamp,
                                              tf_parent_frame_, tf_frame_));
      }
    }

    if (publish_odom_) {
      // calculate velocity
      tf::Vector3 linear_twist, angular_twist;
      calculateTwist(current_trans, prev_trans_, linear_twist, angular_twist, dt);
      geometry_msgs::Twist current_twist;
      tf::vector3TFToMsg(linear_twist, current_twist.linear);
      tf::vector3TFToMsg(angular_twist, current_twist.angular);

      // register msg
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = current_stamp;
      tf::Vector3 current_origin = current_trans.getOrigin();
      odom_msg.pose.pose.position.x = current_origin[0];
      odom_msg.pose.pose.position.y = current_origin[1];
      odom_msg.pose.pose.position.z = current_origin[2];
      tf::quaternionTFToMsg(current_trans.getRotation(), odom_msg.pose.pose.orientation);
      odom_msg.twist.twist = current_twist;
      odom_pub.publish(odom_msg);
    }
    // preserve values for velocity calculation in next step
    prev_trans_ = current_trans;
    prev_stamp_ = current_stamp;
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TransformROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TransformROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TransformROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TransformROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TransformROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



void TransformROSBridge::convertTformToTfTransform(tf::Transform& result_trans)
{
  // Tform: x, y, z, R[0][0], R[0][1], ... , R[2][2]
  double *data = m_Tform.data.get_buffer();
  tf::Vector3 base_origin(data[0], data[1], data[2]);
  result_trans.setOrigin(base_origin);
  hrp::Matrix33 hrpsys_R;
  hrp::getMatrix33FromRowMajorArray(hrpsys_R, data, 3);
  hrp::Vector3 hrpsys_rpy = hrp::rpyFromRot(hrpsys_R);
  tf::Quaternion base_q = tf::createQuaternionFromRPY(hrpsys_rpy(0), hrpsys_rpy(1), hrpsys_rpy(2));
  result_trans.setRotation(base_q);
  return;
}

void TransformROSBridge::calculateTwist(const tf::Transform& _current_trans, const tf::Transform& _prev_trans, tf::Vector3& _linear_twist, tf::Vector3& _angular_twist, double _dt)
{
  // current rotation matrix
  tf::Matrix3x3 current_basis = _current_trans.getBasis();
  
  // linear twist
  tf::Vector3 current_origin = _current_trans.getOrigin();
  tf::Vector3 prev_origin = _prev_trans.getOrigin();
  _linear_twist = current_basis.transpose() * ((current_origin - prev_origin) / _dt);

  // angular twist
  // R = exp(omega_w*dt) * prev_R
  // omega_w is described in global coordinates in relationships of twist transformation.
  // it is easier to calculate omega using hrp functions than tf functions
  tf::Matrix3x3 prev_basis = _prev_trans.getBasis();
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
 
  void TransformROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(transformrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<TransformROSBridge>,
                             RTC::Delete<TransformROSBridge>);
  }
  
};
