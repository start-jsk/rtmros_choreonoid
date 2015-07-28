#include <string>
#include <vector>
#include <iostream>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosgraph_msgs/Clock.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "hrpsys_gazebo_msgs/JointCommand.h"
#include "hrpsys_gazebo_msgs/RobotState.h"

#include "PubQueue.h"

// choreonoid
#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>

namespace cnoid
{
  typedef hrpsys_gazebo_msgs::JointCommand JointCommand;
  typedef hrpsys_gazebo_msgs::RobotState RobotState;

  class IOBSimpleController : public cnoid::SimpleController
  {
  public:
    IOBSimpleController();
    virtual ~IOBSimpleController();
    virtual bool initialize();
    virtual bool control();

  private:
    void UpdateStates();
    void RosQueueThread();
    void GetRobotStates(const ros::Time &_curTime);
    void SetJointCommand(const JointCommand::ConstPtr &_msg);
    void SetJointCommand_impl(const JointCommand &_msg);
    void LoadPIDGainsFromParameter();
    void ZeroJointCommand();
    void UpdatePIDControl(double _dt);
    void PublishJointState();
    bool RosParameterInitializaion();

#if 0
    struct force_sensor_info {
      physics::JointPtr joint;
      std::string frame_id;
      PosePtr pose;
    };

    struct imu_sensor_info {
      physics::LinkPtr link;
      ImuSensorPtr sensor;
      std::string sensor_name;
      std::string frame_id;
    };
#endif
    typedef boost::shared_ptr< cnoid::Link > LinkPtr;
    BodyPtr body;
    std::vector<std::string> jointNames;
    std::vector<LinkPtr> joints;

    boost::shared_ptr<ros::NodeHandle> rosNode;
    ros::CallbackQueue rosQueue;

    RobotState robotState;
    ros::Publisher pubRobotState;
    PubQueue<RobotState>::Ptr pubRobotStateQueue;

    JointCommand jointCommand;
    ros::Subscriber subIOBCommand;

    boost::thread callbackQueeuThread_msg;
    ros::Time lastControllerUpdateTime;
    long time_counter;

    bool publish_joint_state;
    int  publish_joint_state_step;
    int  publish_joint_state_counter;
    ros::Publisher pubJointState;
    PubQueue<sensor_msgs::JointState>::Ptr pubJointStateQueue;

    ros::Publisher pubClock;
    PubQueue<rosgraph_msgs::Clock>::Ptr pubClockQueue;

    //std::vector<double> lastJointCFMDamping;
    //std::vector<double> jointDampingModel;
    //std::vector<double> jointDampingMax;
    //std::vector<double> jointDampingMin;

    //typedef std::map< std::string, struct force_sensor_info > forceSensorMap;
    //typedef std::map< std::string, struct imu_sensor_info > imuSensorMap;
    std::vector<std::string> forceSensorNames;
    std::vector<std::string> imuSensorNames;
    //forceSensorMap forceSensors;
    DeviceList<ForceSensor> forceSensors;
    //imuSensorMap imuSensors;
    DeviceList<RateGyroSensor> rateSensors;
    DeviceList<AccelSensor> accelSensors;

    DeviceList<Camera> cameras;
    DeviceList<RangeSensor> rangeSensors;
    DeviceList<RangeCamera> rcameras;

    std::vector<double> initial_angle;
    std::vector<double> effortLimit;

    class ErrorTerms {
      /// error term contributions to final control output
      double q_p;
      double d_q_p_dt;
      double k_i_q_i;  // integral term weighted by k_i
      double qd_p;
      friend class IOBSimpleController;
    };
    std::vector<ErrorTerms> errorTerms;

    //
    PubMultiQueue pmq;
    boost::mutex mutex;
    //
    std::string robot_name;
    std::string controller_name;
    bool use_loose_synchronized;
    double iob_period;

    static inline int xmlrpc_value_as_int(XmlRpc::XmlRpcValue &v) {
      if((v.getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
         (v.getType() == XmlRpc::XmlRpcValue::TypeInt)) {
        if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double d = v;
          return (int)d;
        } else {
          int i = v;
          return i;
        }
      }
      // not number
      return 0;
    }
    static inline double xmlrpc_value_as_double(XmlRpc::XmlRpcValue &v) {
      if((v.getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
         (v.getType() == XmlRpc::XmlRpcValue::TypeInt)) {
        if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double d = v;
          return d;
        } else {
          int i = v;
          return (double)i;
        }
      }
      // not number
      return 0.0;
    }
    // force sensor averaging
    int force_sensor_average_window_size;
    int force_sensor_average_cnt;
    std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<geometry_msgs::WrenchStamped> > > > forceValQueueMap;
    // effort averaging
    int effort_average_cnt;
    int effort_average_window_size;
    std::vector< boost::shared_ptr<std::vector<double> > > effortValQueue;
    // stepping data publish cycle
    int publish_count;
    int publish_step;
  };
}
