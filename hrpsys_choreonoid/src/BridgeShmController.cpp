#include <cnoid/SimpleController>
#include <cnoid/RateGyroSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/ForceSensor>
#include <vector>
#include <iostream>
#include <fstream>

#include "../controller/system_shm.c"
#include "../controller/myshm.h"
#include "../controller/servo_shm.h"

using namespace cnoid;

class BridgeShmController : public SimpleController
{
  SimpleControllerIO* io;
  BodyPtr robot;
  double dt;

  std::vector<double> hardware_pgain;
  std::vector<double> hardware_dgain;
  std::vector<double> hardware_tqpgain;
  std::vector<double> hardware_tqdgain;

  struct servo_shm *s_shm;
  unsigned long long frame_counter=0;

  void readGainFile(const std::string& pdGainsSimFileName) {
    hardware_pgain.resize(robot->numJoints(),0);
    hardware_dgain.resize(robot->numJoints(),0);
    hardware_tqpgain.resize(robot->numJoints(),0);
    hardware_tqdgain.resize(robot->numJoints(),0);

    std::ifstream gain;
    gain.open(pdGainsSimFileName.c_str());

    if (gain.is_open()) {
      io->os() << "[BridgeShmController] Gain file [" << pdGainsSimFileName << "] opened" << std::endl;
      double tmp;
      int i = 0;
      for (; i < robot->numJoints(); i++) {

      retry:
        {
          std::string str;
          if (std::getline(gain, str)) {
            if (str.empty())   goto retry;
            if (str[0] == '#') goto retry;

            std::istringstream sstrm(str);
            sstrm >> tmp;
            hardware_pgain[i] = tmp;
            if(sstrm.eof()) goto next;
            sstrm >> tmp;
            hardware_dgain[i] = tmp;
            if(sstrm.eof()) goto next;
            sstrm >> tmp;
            hardware_tqpgain[i] = tmp;
            if(sstrm.eof()) goto next;
            sstrm >> tmp;
            hardware_tqdgain[i] = tmp;
          } else {
            i--;
            break;
          }
        }

      next:
        io->os() << "joint: " << i << ", P: " << hardware_pgain[i] << ", D: " << hardware_dgain[i] << ", tqP: " << hardware_tqpgain[i] << ", tqD: " << hardware_tqdgain[i] << std::endl;
      }
      gain.close();
      if (i != robot->numJoints()) {
        io->os() << "\e[0;31m[BridgeShmController] Gain file [" << pdGainsSimFileName << "] does not contain gains for all joints\e[0m" << std::endl;
      }
    } else {
      io->os() << "\e[0;31m[BridgeShmController] Gain file [" << pdGainsSimFileName << "] not opened\e[0m" << std::endl;
    }
  }

  void initialize_shm(int shm_key, bool servoOff = false)
  {
    io->os() << "[BridgeShmController] set_shared_memory " << shm_key << std::endl;
    s_shm = (struct servo_shm *)set_shared_memory(shm_key, sizeof(struct servo_shm));
    if (s_shm == NULL) {
      io->os() << "[BridgeShmController] set_shared_memory failed" << std::endl;
      exit(1);
    }

    // initialize shm
    s_shm->set_ref_vel = 0;
    s_shm->calib_mode = 0;
    s_shm->disable_alert_on_servo_on = 0;

    for (int i = 0; i < robot->numJoints(); i++) {
      s_shm->ref_angle[i] = robot->joint(i)->q();
      s_shm->ref_vel[i] = 0.0;
      s_shm->ref_torque[i] = 0.0;
      s_shm->pgain[i] = 1.0;
      s_shm->dgain[i] = 1.0;
      s_shm->torque_pgain[i] = 0.0;
      s_shm->torque_dgain[i] = 0.0;
      for (int j = 0; j < (int)(sizeof(s_shm->subgain[0]) / sizeof(s_shm->subgain[0][0])); j++){
        s_shm->subgain[i][j] = 0.0;
      }
      s_shm->motor_num[i] = 1;
      s_shm->controlmode[i] = SERVOMODE_POSITION;
      s_shm->servo_on[i] = 0;
      s_shm->servo_off[i] = 0;
      s_shm->torque0[i] = 0;
      s_shm->joint_enable[i] = 1;
      s_shm->joint_offset[i] = 0;
      if (servoOff) {
        s_shm->is_servo_on[i] = 0;
        s_shm->servo_state[0][i] = 0x0800;
        s_shm->loopback[i] = 1;
      } else {
        s_shm->is_servo_on[i] = 1;
        s_shm->servo_state[0][i] = 0x0000;
        s_shm->loopback[i] = 0;
      }
    }
  }

  void write_shm() {
    s_shm->frame = (int)frame_counter; frame_counter++;
    s_shm->received_packet_cnt = 0;
    s_shm->jitter = 0.0;

    for (int i = 0; i < robot->numJoints(); i++){
      s_shm->cur_angle[i] = robot->joint(i)->q();
      s_shm->abs_angle[i] = robot->joint(i)->q();
      s_shm->cur_vel[i] = robot->joint(i)->dq();
      s_shm->cur_torque[i] = robot->joint(i)->u();
      s_shm->motor_current[0][i] = robot->joint(i)->u();
      s_shm->motor_temp[0][i] = 0.0;
      s_shm->motor_outer_temp[0][i] = 0.0;
      s_shm->motor_output[0][i] = 0.0;
      s_shm->board_vin[0][i] = 0.0;
      s_shm->board_vdd[0][i] = 0.0;
      s_shm->comm_normal[0][i] = 1;
      s_shm->h817_rx_error0[0][i] = 0.0;
      s_shm->h817_rx_error1[0][i] = 0.0;
      s_shm->hole_status[0][i] = 0x11;
      s_shm->torque_coef_current[i] = 0.0;
      s_shm->torque_coef_inertia[i] = 0.0;
      s_shm->torque_coef_coulombfric[i] = 0.0;
      s_shm->torque_coef_viscousfric[i] = 0.0;
    }
    {
      const DeviceList<RateGyroSensor>& rateGyroSensors = robot->devices();
      for(int i=0; i < rateGyroSensors.size() && i < MAX_IMU_NUM/* defined in servo_shm.h */; i++){
        for (int j=0; j<3; j++) s_shm->body_omega[rateGyroSensors[i]->id()][j] = rateGyroSensors[i]->w()[j];
      }
    }
    {
      const DeviceList<AccelerationSensor>& accelerationSensors = robot->devices();
      for(int i=0; i < accelerationSensors.size() && i < MAX_IMU_NUM/* defined in servo_shm.h */; i++){
        for (int j=0; j<3; j++) s_shm->body_acc[accelerationSensors[i]->id()][j] = accelerationSensors[i]->dv()[j];
      }
    }
    {
      const DeviceList<ForceSensor>& forceSensors = robot->devices();
      for(int i=0; i < forceSensors.size() && i < MAX_FSENSOR_NUM/* defined in servo_shm.h */ ; i++){
        for (int j=0; j<6; j++) s_shm->reaction_force[forceSensors[i]->id()][j] = forceSensors[i]->F()[j];
      }
    }
  }

  void read_shm_and_control() {
    for (int i = 0; i < robot->numJoints(); i++){
      Link* joint = robot->joint(i);

      if (s_shm->servo_off[i]){
        //do_power_off
        s_shm->servo_state[0][i] = 0x0800;
        s_shm->is_servo_on[i] = 0;
        s_shm->servo_off[i] = 0;
      }
      else if (s_shm->servo_on[i]) {
        if (s_shm->joint_enable[i]) {
          //do_power_on
          s_shm->servo_state[0][i] = 0x0000;
          s_shm->is_servo_on[i] = 1;
        }
        s_shm->servo_on[i] = 0;
      }

      if ( s_shm->is_servo_on[i] == 1 && s_shm->loopback[i] == 0 ) {
        double qref = (s_shm->loopback[i] == 1) ? s_shm->cur_angle[i] : s_shm->ref_angle[i];
        double dqref = s_shm->ref_vel[i];
        double qact = joint->q();
        double dqact = joint->dq();
        float limited_pgain = (s_shm->pgain[i] < 0.0) ? 0.0 : s_shm->pgain[i];
        float limited_dgain = (s_shm->dgain[i] < 0.0) ? 0.0 : s_shm->dgain[i];
        float limited_torque_pgain = (s_shm->torque_pgain[i] < 0.0) ? 0.0 : s_shm->torque_pgain[i];
        float limited_torque_dgain = (s_shm->torque_dgain[i] < 0.0) ? 0.0 : s_shm->torque_dgain[i];
        double u=0;
        switch(s_shm->controlmode[i]){
        case SERVOMODE_POSITION_TORQUE:
        case SERVOMODE_POSITION_FFTORQUE:
          u = (qref - qact) * limited_pgain * hardware_pgain[i] + (dqref - dqact) * limited_dgain * hardware_dgain[i] + s_shm->ref_torque[i] * limited_torque_pgain * hardware_tqpgain[i];
          break;
        case SERVOMODE_POSITION:
          u = (qref - qact) * limited_pgain * hardware_pgain[i] + (dqref - dqact) * limited_dgain * hardware_dgain[i];
          break;
        default:
          break;
        }
        joint->u() = u;
      } else {
        joint->u() = 0;
      }
    }
  }

public:

  virtual bool initialize(SimpleControllerIO* io_) override
  {
    io = io_;
    robot = io->body();
    dt = io->timeStep();

    const DeviceList<RateGyroSensor>& rateGyroSensors = robot->devices();
    for(int i=0; i < rateGyroSensors.size(); i++){
      io->enableInput(rateGyroSensors[i]);
    }
    const DeviceList<AccelerationSensor>& accelerationSensors = robot->devices();
    for(int i=0; i < accelerationSensors.size(); i++){
      io->enableInput(accelerationSensors[i]);
    }
    const DeviceList<ForceSensor>& forceSensors = robot->devices();
    for(int i=0; i < forceSensors.size(); i++){
      io->enableInput(forceSensors[i]);
    }

    std::string pdgainsSimFileName;
    int shm_key = 5555;
    bool servoOff = false;

    std::vector<std::string> options = io->options();
    for(size_t i=0;i<options.size();i++){
      std::string option = "pdGainsSimFileName:";
      if (options[i].size() >= option.size() &&
          std::equal(std::begin(option), std::end(option), std::begin(options[i]))) {
        pdgainsSimFileName = options[i].substr(option.size());
        continue;
      }
      option = "shm_key:";
      if (options[i].size() >= option.size() &&
          std::equal(std::begin(option), std::end(option), std::begin(options[i]))) {
        shm_key = std::stoi(options[i].substr(option.size()));
        continue;
      }
      option = "servoOff:";
      if (options[i].size() >= option.size() &&
          std::equal(std::begin(option), std::end(option), std::begin(options[i]))) {
        std::string value = options[i].substr(option.size());
        if (value == "True" ||
            value == "true" ||
            value == "TRUE" ||
            value == "1") {
          servoOff = true;
        }
        continue;
      }
    }

    readGainFile(pdgainsSimFileName);

    for(int i=0; i < robot->numJoints(); ++i){
      Link* joint = robot->joint(i);
      joint->setActuationMode(Link::JOINT_TORQUE);
      io->enableOutput(joint);
      io->enableInput(joint, JOINT_DISPLACEMENT | JOINT_VELOCITY);
    }

    this->initialize_shm(shm_key, servoOff);
    return true;
  }

  virtual bool control() override
  {
    read_shm_and_control();
    write_shm();
    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(BridgeShmController)
