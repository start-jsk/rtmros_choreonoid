/**
   Sample controller
*/

#include <algorithm> // min/max

#include "IOBSimpleController.h"

using namespace std;
using namespace cnoid;

cnoid::IOBSimpleController::IOBSimpleController() : time_counter(0),
                                                    publish_joint_state(false),
                                                    publish_joint_state_step(0),
                                                    publish_joint_state_counter(0),
                                                    use_loose_synchronized(false),
                                                    iob_period(0.005),
                                                    force_sensor_average_window_size(6),
                                                    force_sensor_average_cnt(0),
                                                    effort_average_window_size(6),
                                                    effort_average_cnt(0),
                                                    publish_step(2),
                                                    publish_count(0)
{
  std::cerr << "IOBSimpleContoller created" << std::endl;

  if (!rosNode) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
    rosNode.reset(new ros::NodeHandle(""));
  }
}

cnoid::IOBSimpleController::~IOBSimpleController() {
  {
    boost::mutex::scoped_lock lock(mutex);
    time_counter = 0;
    std::cerr << "IOBSimpleContoller destructed" << std::endl;
  }
}

bool cnoid::IOBSimpleController::initialize() {
  time_counter = 0;
  std::cerr << "IOBSimpleContoller initializing" << std::endl;

  body = ioBody();

  ///////
  std::cerr << "numJoints: " << body->numJoints() << std::endl;
  std::cerr << "numLinks: " << body->numLinks() << std::endl;
  std::cerr << "numDevies: " << body->numDevices() << std::endl;

  //ros::isInitialized()
  if(!RosParameterInitializaion()) {
    return false;
  }
  return true;
}

bool cnoid::IOBSimpleController::control() {
  //std::cerr << "control" << std::endl;
  time_counter++;
  //std::cerr << "tm: " << time_counter << std::endl;

  rosgraph_msgs::Clock tm;
  tm.clock = ros::Time((double(time_counter)/1000));
  pubClockQueue->push(tm, pubClock);

  UpdateStates();

  return true;
}

bool cnoid::IOBSimpleController::RosParameterInitializaion() {
  controller_name = "hrpsys_gazebo_configuration";
  robot_name = body->name();     // name enable to change in choreonoid
  //robot_name = body->modelName();// model name from wrl
  ROS_INFO("robot_name: %s", robot_name.c_str());

  controller_name = robot_name + "/" + controller_name;

  // creating joints from ros param
  if (rosNode->hasParam(controller_name)) {
    { // iob rate
      std::string pname = controller_name + "/iob_rate";
      if (rosNode->hasParam(pname)) {
        double rate;
        rosNode->getParam(pname, rate);
        ROS_INFO("iob rate %f", rate);
        iob_period = 1.0 / rate;
      }
    }
    { // force sensor averaging
      std::string pname = controller_name + "/force_sensor_average_window_size";
      if (rosNode->hasParam(pname)) {
        int asize;
        rosNode->getParam(pname, asize);
        force_sensor_average_window_size = asize;
        ROS_INFO("force_sensor_average_window_size %d", asize);
      }
    }
    { // read publish_joint_state from rosparam
      std::string pname = controller_name + "/publish_joint_state";
      if (rosNode->hasParam(pname)) {
        std::string topic;
        if (rosNode->hasParam(pname + "/topic")) {
          rosNode->getParam(pname + "/topic", topic);
        } else {
          topic = robot_name + "/joint_state";
        }
        //
        publish_joint_state_counter = 0;
        publish_joint_state_step = 1;
        if (rosNode->hasParam(pname + "/step")) {
          int stp;
          rosNode->getParam(pname + "/step", stp);
          publish_joint_state_step = stp;
        }
        //
        pubJointStateQueue = pmq.addPub<sensor_msgs::JointState>();
        pubJointState
          = rosNode->advertise<sensor_msgs::JointState>(topic, 100, true);
        ROS_INFO("publish joint state");
        publish_joint_state = true;
      }
    }
    //
    XmlRpc::XmlRpcValue param_val;
    rosNode->getParam(controller_name, param_val);
    if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeStruct) {
      std::string rname = param_val["robotname"];
      XmlRpc::XmlRpcValue joint_lst = param_val["joints"];
      XmlRpc::XmlRpcValue init_ang = param_val["initial_angle"];
      XmlRpc::XmlRpcValue fsensors = param_val["force_torque_sensors"];
      XmlRpc::XmlRpcValue fsensors_config = param_val["force_torque_sensors_config"];
      XmlRpc::XmlRpcValue imusensors = param_val["imu_sensors"];
      XmlRpc::XmlRpcValue imusensors_config = param_val["imu_sensors_config"];

      if (rname != robot_name) {
        ROS_WARN("mismatch robotnames: %s (ros parameter) != %s (gazebo element)",
                 rname.c_str(), robot_name.c_str());
      } else {
        ROS_INFO("robotname: %s", rname.c_str());
      }
      // joint name
      if (joint_lst.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int s = 0; s < joint_lst.size(); s++) {
          std::string n = joint_lst[s];
          ROS_INFO("add joint: %s", n.c_str());
          jointNames.push_back(n);
        }
      } else {
        ROS_WARN("Controlled Joints: no setting exists");
      }
      if (init_ang.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        initial_angle.resize(init_ang.size());
        ROS_INFO("read initial %d", init_ang.size());
        for(int s = 0; s < init_ang.size(); s++) {
          double ang = init_ang[s];
          initial_angle[s] = ang;
        }
      } else {
        ROS_WARN("Type mismatch: initial_angle");
      }

      // Force sensor setting
      if (fsensors.getType() == XmlRpc::XmlRpcValue::TypeArray &&
          fsensors_config.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for(int s = 0; s < fsensors.size(); s++) {
          forceSensorNames.push_back(fsensors[s]);
        }
        for(XmlRpc::XmlRpcValue::iterator f = fsensors_config.begin();
            f != fsensors_config.end(); f++) {
          std::string sensor_name = f->first;
          if (f->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            struct force_sensor_info fsi;
            std::string fi = f->second["frame_id"];
            fsi.frame_id = fi;
            DeviceList<ForceSensor> fs_lst = body->devices();
            for(DeviceList<ForceSensor> ::iterator it = fs_lst.begin();
                it != fs_lst.end();  it++) {
              // std::cerr << "name: " << (*it)->name() << std::endl;
              if((*it)->name() == sensor_name) {
                fsi.fsensor = *it;
                break;
              }
            }
            if(!!fsi.fsensor){
              forceSensors[sensor_name] = fsi;
            } else {
              ROS_ERROR("Force-Torque sensor: %s not found", sensor_name.c_str());
              continue;
            }
          } else {
            ROS_ERROR("Force-Torque sensor: %s has invalid configuration", sensor_name.c_str());
          }
          // setup force sensor publishers
          boost::shared_ptr<std::vector<boost::shared_ptr<geometry_msgs::WrenchStamped> > > forceValQueue(new std::vector<boost::shared_ptr<geometry_msgs::WrenchStamped> >);
          // forceValQueue->resize(force_sensor_average_window_size);
          for ( int i=0; i<force_sensor_average_window_size; i++ ){
            boost::shared_ptr<geometry_msgs::WrenchStamped> fbuf(new geometry_msgs::WrenchStamped);
            forceValQueue->push_back(fbuf);
          }
          forceValQueueMap[sensor_name] = forceValQueue;
        }
      } else {
        ROS_WARN("Force-Torque sensor: no setting exists");
      } // Force sensor setting

#if 0
      // IMU sensor setting
      if (imusensors.getType() == XmlRpc::XmlRpcValue::TypeArray &&
          imusensors_config.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for(int s = 0; s < imusensors.size(); s++) {
          imuSensorNames.push_back(imusensors[s]);
        }
        for(XmlRpc::XmlRpcValue::iterator im = imusensors_config.begin(); im != imusensors_config.end(); im++) {
          std::string sensor_name = im->first;
          if (im->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            std::string sn = im->second["ros_name"];
            std::string ln = im->second["link_name"];
            std::string fi = im->second["frame_id"];
            ROS_INFO("imu: %s, %s, %s, %s", sensor_name.c_str(), sn.c_str(),
                     ln.c_str(), fi.c_str());

            struct imu_sensor_info msi;
            msi.sensor_name = sn;
            msi.frame_id = fi;
            msi.link = model->GetLink(ln);

            if (!msi.link)  {
              gzerr << ln << " not found\n";
            } else {
              // Get imu sensors
              msi.sensor = boost::dynamic_pointer_cast<sensors::ImuSensor>
                (sensors::SensorManager::Instance()->GetSensor
                 (world->GetName() + "::" + msi.link->GetScopedName() + "::" + msi.sensor_name));

              if (!msi.sensor)  {
                gzerr << sensor_name << "("                             \
                      << (world->GetName() + "::" + msi.link->GetScopedName() + "::" + msi.sensor_name) \
                      <<" not found\n" << "\n";
              }
              imuSensors[sensor_name] = msi;
            }
          } else {
            ROS_ERROR("IMU sensor: %s has invalid configuration", sensor_name.c_str());
          }
        }
      } else {
        ROS_WARN("IMU sensor: no setting exists");
      } // IMU sensor setting
#endif
    } else {
      ROS_WARN_STREAM("param: " << controller_name << ", configuration is not an array.");
    }
  } else {
    ROS_ERROR_STREAM("controller: " << controller_name << " has no parameter.");
  }

  // get pointers to joints from gazebo
  joints.resize(0);
  ROS_INFO("joints size = %ld", jointNames.size());
  for (unsigned int i = 0; i < jointNames.size(); ++i) {
    LinkPtr p(body->link(jointNames[i]));
    if (!p)  {
      ROS_ERROR("%s robot expected joint[%s] not present, plugin not loaded",
                robot_name.c_str(), jointNames[i].c_str());
      return false;
    }
    joints.push_back(p);
  }

  // get effort limits from gazebo
  effortLimit.resize(jointNames.size());
  for (unsigned i = 0; i < effortLimit.size(); ++i) {
    effortLimit[i] = 400; // there is no torque limit in choreonoid ???
    // ROS_DEBUG("effort_limit: %s %f", joints[i]->GetName().c_str(), joints[i]->GetEffortLimit(0));
  }

  {
    // initialize PID states: error terms
    errorTerms.resize(joints.size());
    for (unsigned i = 0; i < errorTerms.size(); ++i) {
      errorTerms[i].q_p = 0;
      errorTerms[i].d_q_p_dt = 0;
      errorTerms[i].k_i_q_i = 0;
      errorTerms[i].qd_p = 0;
    }
  }

  {
    // We are not sending names due to the fact that there is an enum
    // joint indices in ...
    robotState.position.resize(joints.size());
    robotState.velocity.resize(joints.size());
    robotState.effort.resize(joints.size());
    // effort average
    effortValQueue.resize(0);
    for(int i = 0; i < effort_average_window_size; i++) {
      boost::shared_ptr<std::vector<double> > vbuf(new std::vector<double> (joints.size()));
      effortValQueue.push_back(vbuf);
    }
    // for reference
    robotState.ref_position.resize(joints.size());
    robotState.ref_velocity.resize(joints.size());
    // for effort feedback
    robotState.kp_position.resize(joints.size());
    robotState.ki_position.resize(joints.size());
    robotState.kd_position.resize(joints.size());
    robotState.kp_velocity.resize(joints.size());
    robotState.i_effort_min.resize(joints.size());
    robotState.i_effort_max.resize(joints.size());
  }

  {
    jointCommand.position.resize(joints.size());
    jointCommand.velocity.resize(joints.size());
    jointCommand.effort.resize(joints.size());
    jointCommand.kp_position.resize(joints.size());
    jointCommand.ki_position.resize(joints.size());
    jointCommand.kd_position.resize(joints.size());
    jointCommand.kp_velocity.resize(joints.size());
    jointCommand.i_effort_min.resize(joints.size());
    jointCommand.i_effort_max.resize(joints.size());

    ZeroJointCommand();
  }

  // publish multi queue
  pmq.startServiceThread();

  // pull down controller parameters
  LoadPIDGainsFromParameter();

  // ROS clock
  pubClockQueue = pmq.addPub<rosgraph_msgs::Clock>();
  pubClock = rosNode->advertise<rosgraph_msgs::Clock>("/clock", 100, true);

  // ROS Controller API
  pubRobotStateQueue = pmq.addPub<RobotState>();
  pubRobotState = rosNode->advertise<RobotState>(robot_name + "/robot_state", 100, true);

  // ros topic subscribtions
  ros::SubscribeOptions IOBCommandSo =
    ros::SubscribeOptions::create<JointCommand>(robot_name + "/joint_command", 100,
                                                boost::bind(&cnoid::IOBSimpleController::SetJointCommand, this, _1),
                                                ros::VoidPtr(), &rosQueue);
  // Enable TCP_NODELAY because TCP causes bursty communication with high jitter,
  IOBCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
  subIOBCommand = rosNode->subscribe(IOBCommandSo);

  callbackQueeuThread_msg =
    boost::thread(boost::bind(&cnoid::IOBSimpleController::RosQueueThread, this));

  return true;
}

///////
void cnoid::IOBSimpleController::ZeroJointCommand() {
  for (unsigned i = 0; i < jointNames.size(); ++i) {
    jointCommand.position[i] = 0;
    jointCommand.velocity[i] = 0;
    jointCommand.effort[i] = 0;
    // store these directly on altasState, more efficient for pub later
    robotState.kp_position[i] = 0;
    robotState.ki_position[i] = 0;
    robotState.kd_position[i] = 0;
    robotState.kp_velocity[i] = 0;
    robotState.i_effort_min[i] = 0;
    robotState.i_effort_max[i] = 0;
  }
  //ROS_INFO("init ang %d %d", jointCommand.position.size(), initial_angle.size());
  if (jointCommand.position.size() == initial_angle.size()) {
    for (unsigned i = 0; i < jointNames.size(); ++i) {
      jointCommand.position[i] = initial_angle[i];
    }
  }
  jointCommand.desired_controller_period_ms = 0;
}

void cnoid::IOBSimpleController::LoadPIDGainsFromParameter() {
  // pull down controller parameters
  std::string namestr(controller_name);

  for (unsigned int i = 0; i < joints.size(); ++i) {
    std::string joint_ns(namestr);
    joint_ns += ("/gains/" + joints[i]->name() + "/");

    // this is so ugly
    double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0, vp_val = 0;
    std::string p_str = std::string(joint_ns)+"p";
    std::string i_str = std::string(joint_ns)+"i";
    std::string d_str = std::string(joint_ns)+"d";
    std::string i_clamp_str = std::string(joint_ns)+"i_clamp";
    std::string vp_str = std::string(joint_ns)+"vp";

    if (!rosNode->getParam(p_str, p_val)) {
      ROS_WARN("IOBPlugin: couldn't find a P param for %s", joint_ns.c_str());
    }
    if (!rosNode->getParam(i_str, i_val)) {
      ROS_WARN("IOBPlugin: couldn't find a I param for %s", joint_ns.c_str());
    }
    if (!rosNode->getParam(d_str, d_val)) {
      ROS_WARN("IOBPlugin: couldn't find a D param for %s", joint_ns.c_str());
    }
    if (!rosNode->getParam(i_clamp_str, i_clamp_val)) {
      ROS_WARN("IOBPlugin: couldn't find a I_CLAMP param for %s", joint_ns.c_str());
    }
    if (!rosNode->getParam(vp_str, vp_val)) {
      ROS_WARN("IOBPlugin: couldn't find a VP param for %s", joint_ns.c_str());
    }

    // store these directly on altasState, more efficient for pub later
    robotState.kp_position[i]  =  p_val;
    robotState.ki_position[i]  =  i_val;
    robotState.kd_position[i]  =  d_val;
    robotState.i_effort_min[i] = -i_clamp_val;
    robotState.i_effort_max[i] =  i_clamp_val;
    robotState.kp_velocity[i]  =  vp_val;
  }
}

void cnoid::IOBSimpleController::SetJointCommand(const JointCommand::ConstPtr &_msg) {
  SetJointCommand_impl(*_msg);
}

void cnoid::IOBSimpleController::SetJointCommand_impl(const JointCommand &_msg) {
  // Update Joint Command
  boost::mutex::scoped_lock lock(mutex);

  jointCommand.header.stamp = _msg.header.stamp;

  // for jointCommand, only position, velocity and efforts are used.
  if (_msg.position.size() == jointCommand.position.size())
    std::copy(_msg.position.begin(), _msg.position.end(), jointCommand.position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements position[%ld] than expected[%ld]",
      _msg.position.size(), jointCommand.position.size());

  if (_msg.velocity.size() == jointCommand.velocity.size())
    std::copy(_msg.velocity.begin(), _msg.velocity.end(), jointCommand.velocity.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements velocity[%ld] than expected[%ld]",
      _msg.velocity.size(), jointCommand.velocity.size());

  if (_msg.effort.size() == jointCommand.effort.size())
    std::copy(_msg.effort.begin(), _msg.effort.end(), jointCommand.effort.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements effort[%ld] than expected[%ld]",
      _msg.effort.size(), jointCommand.effort.size());

  // the rest are stored in robotState for publication
  if (_msg.kp_position.size() == robotState.kp_position.size())
    std::copy(_msg.kp_position.begin(), _msg.kp_position.end(), robotState.kp_position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
              " elements kp_position[%ld] than expected[%ld]",
              _msg.kp_position.size(), robotState.kp_position.size());

  if (_msg.ki_position.size() == robotState.ki_position.size())
    std::copy(_msg.ki_position.begin(), _msg.ki_position.end(), robotState.ki_position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
              " elements ki_position[%ld] than expected[%ld]",
              _msg.ki_position.size(), robotState.ki_position.size());

  if (_msg.kd_position.size() == robotState.kd_position.size())
    std::copy(_msg.kd_position.begin(), _msg.kd_position.end(), robotState.kd_position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
              " elements kd_position[%ld] than expected[%ld]",
              _msg.kd_position.size(), robotState.kd_position.size());

  if (_msg.kp_velocity.size() == robotState.kp_velocity.size())
    std::copy(_msg.kp_velocity.begin(), _msg.kp_velocity.end(), robotState.kp_velocity.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
              " elements kp_velocity[%ld] than expected[%ld]",
              _msg.kp_velocity.size(), robotState.kp_velocity.size());

  if (_msg.i_effort_min.size() == robotState.i_effort_min.size())
    std::copy(_msg.i_effort_min.begin(), _msg.i_effort_min.end(), robotState.i_effort_min.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
              " elements i_effort_min[%ld] than expected[%ld]",
              _msg.i_effort_min.size(), robotState.i_effort_min.size());

  if (_msg.i_effort_max.size() == robotState.i_effort_max.size())
    std::copy(_msg.i_effort_max.begin(), _msg.i_effort_max.end(), robotState.i_effort_max.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
              " elements i_effort_max[%ld] than expected[%ld]",
              _msg.i_effort_max.size(), robotState.i_effort_max.size());

  jointCommand.desired_controller_period_ms =
    _msg.desired_controller_period_ms;
}

void cnoid::IOBSimpleController::PublishJointState() {
  publish_joint_state_counter++;
  if(publish_joint_state_step > publish_joint_state_counter) {
    return;
  }
  if(jointNames.size() != joints.size()) {
    ROS_ERROR("joint length miss match %ld != %ld", jointNames.size(), joints.size());
    return;
  }
  if(jointNames.size() == 0) {
    ROS_ERROR("joint length is zero");
    return;
  }
  if(!pubJointStateQueue || !pubJointState) {
    ROS_ERROR("no publisher %d %d", !pubJointStateQueue, !pubJointState);
    return;
  }
  // publish joint_state
  sensor_msgs::JointState jstate;
  jstate.header.stamp = robotState.header.stamp;
  jstate.name.resize(jointNames.size());
  jstate.position.resize(joints.size());
  for (unsigned int i = 0; i < joints.size(); ++i) {
    jstate.name[i] = jointNames[i];
    jstate.position[i] = joints[i]->q();
  }
  pubJointStateQueue->push(jstate, pubJointState);
  publish_joint_state_counter = 0;
}

void cnoid::IOBSimpleController::UpdateStates() {
  //ROS_DEBUG("update");
  ros::Time curTime = ros::Time((double)time_counter/1000);

  if (curTime > lastControllerUpdateTime) {
    // gather robot state data
    GetRobotStates(curTime);

    publish_count++;
    if(publish_count % publish_step == 0) {
      // publish robot states
      pubRobotStateQueue->push(robotState, pubRobotState);
      if (publish_joint_state) PublishJointState();
    }

    if (use_loose_synchronized) { // loose synchronization
      ros::Time rnow = curTime;
      int counter = 0;
      //if ((rnow - jointCommand.header.stamp).toSec() > 0.002) {
      //  ROS_WARN("%f update fail %f", rnow.toSec(), jointCommand.header.stamp.toSec());
      //}
      while ((rnow - jointCommand.header.stamp).toSec() > iob_period) {
        ros::WallDuration(0, 200000).sleep(); // 0.2 ms
        if(counter++ > 100) {
          //std::cerr << "time out" << std::endl;
          break;
        }
      }
      //if(counter > 0) {
      //  ROS_WARN("%f recover %f", rnow.toSec(), jointCommand.header.stamp.toSec());
      //}
    }

    {
      boost::mutex::scoped_lock lock(mutex);
      UpdatePIDControl((curTime - lastControllerUpdateTime).toSec());
    }
    lastControllerUpdateTime = curTime;
  }
}

void cnoid::IOBSimpleController::GetRobotStates(const ros::Time &_curTime){
  // populate robotState from robot
  robotState.header.stamp = _curTime;

  boost::shared_ptr<std::vector<double > > vbuf = effortValQueue.at(effort_average_cnt);
  // joint states
  for (unsigned int i = 0; i < joints.size(); ++i) {
    robotState.position[i] = joints[i]->q();
    robotState.velocity[i] = joints[i]->dq();
    robotState.effort[i] = 0.0;
    vbuf->at(i) = joints[i]->u();
  }
  for (int j = 0; j < effortValQueue.size(); j++) {
    boost::shared_ptr<std::vector<double > > vbuf = effortValQueue.at(j);
    for (int i = 0; i < joints.size(); i++) {
      robotState.effort[i] += vbuf->at(i);
    }
  }
  if (effortValQueue.size() > 0 ){
    for (int i = 0; i < joints.size(); i++) {
      robotState.effort[i] *= 1.0/effortValQueue.size();
    }
  } else {
    ROS_WARN("invalid force val queue size 0");
  }
  effort_average_cnt = (effort_average_cnt+1) % effort_average_window_size;

  // enqueue force sensor values
  robotState.sensors.resize(forceSensorNames.size());
  for (unsigned int i = 0; i < forceSensorNames.size(); i++) {
    forceSensorMap::iterator it = forceSensors.find(forceSensorNames[i]);
    boost::shared_ptr<std::vector<boost::shared_ptr<geometry_msgs::WrenchStamped> > > forceValQueue = forceValQueueMap.find(forceSensorNames[i])->second;
    boost::shared_ptr<geometry_msgs::WrenchStamped> forceVal = forceValQueue->at(force_sensor_average_cnt);
    if(it != forceSensors.end()) {
      robotState.sensors[i].name = forceSensorNames[i];
      robotState.sensors[i].frame_id = it->second.frame_id;
      cnoid::Vector6 wrench = it->second.fsensor->F();
      forceVal->wrench.force.x = wrench[0];
      forceVal->wrench.force.y = wrench[1];
      forceVal->wrench.force.z = wrench[2];
      forceVal->wrench.torque.x = wrench[3];
      forceVal->wrench.torque.y = wrench[4];
      forceVal->wrench.torque.z = wrench[5];
    } else {
      ROS_ERROR("force sensor error");
    }
    robotState.sensors[i].force.x = 0;
    robotState.sensors[i].force.y = 0;
    robotState.sensors[i].force.z = 0;
    robotState.sensors[i].torque.x = 0;
    robotState.sensors[i].torque.y = 0;
    robotState.sensors[i].torque.z = 0;
    for ( int j=0; j<forceValQueue->size() ; j++ ){
      boost::shared_ptr<geometry_msgs::WrenchStamped> forceValBuf = forceValQueue->at(j);
      robotState.sensors[i].force.x += forceValBuf->wrench.force.x;
      robotState.sensors[i].force.y += forceValBuf->wrench.force.y;
      robotState.sensors[i].force.z += forceValBuf->wrench.force.z;
      robotState.sensors[i].torque.x += forceValBuf->wrench.torque.x;
      robotState.sensors[i].torque.y += forceValBuf->wrench.torque.y;
      robotState.sensors[i].torque.z += forceValBuf->wrench.torque.z;
    }
    if ( forceValQueue->size() > 0 ){
      robotState.sensors[i].force.x *= 1.0/forceValQueue->size();
      robotState.sensors[i].force.y *= 1.0/forceValQueue->size();
      robotState.sensors[i].force.z *= 1.0/forceValQueue->size();
      robotState.sensors[i].torque.x *= 1.0/forceValQueue->size();
      robotState.sensors[i].torque.y *= 1.0/forceValQueue->size();
      robotState.sensors[i].torque.z *= 1.0/forceValQueue->size();
    } else {
      ROS_WARN("invalid force val queue size 0");
    }
  }
  force_sensor_average_cnt = (force_sensor_average_cnt+1) % force_sensor_average_window_size;

#if 0
  // imu sensors
  robotState.Imus.resize(imuSensorNames.size());
  for (unsigned int i = 0; i < imuSensorNames.size(); i++) {
    imuSensorMap::iterator it = imuSensors.find(imuSensorNames[i]);
    ImuSensorPtr sp = it->second.sensor;
    if(!!sp) {
      robotState.Imus[i].name = imuSensorNames[i];
      robotState.Imus[i].frame_id = it->second.frame_id;
      math::Vector3 wLocal = sp->GetAngularVelocity();
      math::Vector3 accel = sp->GetLinearAcceleration();
      math::Quaternion imuRot = sp->GetOrientation();
      robotState.Imus[i].angular_velocity.x = wLocal.x;
      robotState.Imus[i].angular_velocity.y = wLocal.y;
      robotState.Imus[i].angular_velocity.z = wLocal.z;
      robotState.Imus[i].linear_acceleration.x = accel.x;
      robotState.Imus[i].linear_acceleration.y = accel.y;
      robotState.Imus[i].linear_acceleration.z = accel.z;
      robotState.Imus[i].orientation.x = imuRot.x;
      robotState.Imus[i].orientation.y = imuRot.y;
      robotState.Imus[i].orientation.z = imuRot.z;
      robotState.Imus[i].orientation.w = imuRot.w;
    }
  }
#endif
  {
    boost::mutex::scoped_lock lock(mutex);
    for (unsigned int i = 0; i < joints.size(); ++i) {
      robotState.ref_position[i] = jointCommand.position[i];
      robotState.ref_velocity[i] = jointCommand.velocity[i];
#if 0 //DEBUG
      ROS_INFO("%d %f %f %f %f %f %f",
               i, robotState.position[i], robotState.velocity[i],
               joints[i]->q(), joints[i]->dq(),
               robotState.ref_position[i], robotState.ref_velocity[i]);
#endif
    }
  }
}

inline double myclamp(double v, double min, double max) {
  return (double) std::max(std::min(v, max), min);
}

void cnoid::IOBSimpleController::UpdatePIDControl(double _dt) {

  /// update pid with feedforward force
  for (unsigned int i = 0; i < joints.size(); ++i) {
    // truncate joint position within range of motion
    double positionTarget = myclamp(jointCommand.position[i],
                                    joints[i]->q_lower(), joints[i]->q_upper());

    //double q_p = positionTarget - robotState.position[i];
    double q_p = positionTarget - joints[i]->q();

    if (std::fabs(_dt) > 1e-6)
      errorTerms[i].d_q_p_dt = (q_p - errorTerms[i].q_p) / _dt;

    errorTerms[i].q_p = q_p;

    errorTerms[i].qd_p =
      jointCommand.velocity[i] - joints[i]->dq();
      //jointCommand.velocity[i] - robotState.velocity[i];

    errorTerms[i].k_i_q_i = myclamp(errorTerms[i].k_i_q_i +
                                    _dt *
                                    static_cast<double>(robotState.ki_position[i])
                                    * errorTerms[i].q_p,
                                    static_cast<double>(robotState.i_effort_min[i]),
                                    static_cast<double>(robotState.i_effort_max[i]));

    // use gain params to compute force cmd
    double forceUnclamped =
      static_cast<double>(robotState.kp_position[i]) * errorTerms[i].q_p +
      errorTerms[i].k_i_q_i +
      static_cast<double>(robotState.kd_position[i]) * errorTerms[i].d_q_p_dt +
      static_cast<double>(robotState.kp_velocity[i]) * errorTerms[i].qd_p +
      jointCommand.effort[i];

    // keep unclamped force for integral tie-back calculation
    double forceClamped = myclamp(forceUnclamped, -effortLimit[i], effortLimit[i]);

    // integral tie-back during control saturation if using integral gain
    if ((std::fabs(forceClamped - forceUnclamped) > 1e-6) &&
        (std::fabs(static_cast<double>(robotState.ki_position[i])) > 1e-6)) {
      // lock integral term to provide continuous control as system moves
      // out of staturation
      errorTerms[i].k_i_q_i = myclamp(errorTerms[i].k_i_q_i + (forceClamped - forceUnclamped),
                                      static_cast<double>(robotState.i_effort_min[i]),
                                      static_cast<double>(robotState.i_effort_max[i]));
    }
    // clamp force after integral tie-back
    forceClamped = myclamp(forceUnclamped, -effortLimit[i], effortLimit[i]);

    // apply force to joint
    joints[i]->u() = forceClamped;
#if 0 //DEBUG
    ROS_INFO("%d u: %f %f %f %f / %f %f %f ",i, forceClamped, errorTerms[i].q_p, errorTerms[i].d_q_p_dt, errorTerms[i].qd_p,
             positionTarget, joints[i]->q(), joints[i]->dq());
#endif
    // fill in jointState efforts
    //robotState.effort[i] = forceClamped;
  }
}

void cnoid::IOBSimpleController::RosQueueThread() {
  static const double timeout = 0.01;

  while (rosNode->ok()) {
    ros::spinOnce();
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(IOBSimpleController)
