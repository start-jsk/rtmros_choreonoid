#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "iob.h"

#include <stdio.h>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/CorbaNaming.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include "RobotHardware_choreonoid.h"

static std::vector<double> command;
static std::vector<double> act_angle;
static std::vector<double> com_torque;
static std::vector<double> act_vel;
static std::vector<double> act_torque;
static std::vector<std::vector<double> > forces;
static std::vector<std::vector<double> > gyros;
static std::vector<std::vector<double> > accelerometers;
static std::vector<std::vector<double> > attitude_sensors;
static std::vector<std::vector<double> > force_offset;
static std::vector<std::vector<double> > gyro_offset;
static std::vector<std::vector<double> > accel_offset;
static std::vector<int> power;
static std::vector<int> servo;
static bool isLocked = false;
static int frame = 0;
static timespec g_ts;
static long g_period_ns=5000000;

Time iob_time;

#define FORCE_AVERAGE 8
static int force_counter;
static std::vector< std::vector<std::vector<double> > > force_queue;

#define CHECK_JOINT_ID(id) if ((id) < 0 || (id) >= number_of_joints()) return E_ID
#define CHECK_FORCE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_force_sensors()) return E_ID
#define CHECK_ACCELEROMETER_ID(id) if ((id) < 0 || (id) >= number_of_accelerometers()) return E_ID
#define CHECK_GYRO_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_gyro_sensors()) return E_ID
#define CHECK_ATTITUDE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_attitude_sensors()) return E_ID

using namespace RTC;

RobotHardware_choreonoid *self_ptr;

//* *//
TimedDoubleSeq m_angleIn;
InPort<TimedDoubleSeq> *ip_angleIn;
TimedDoubleSeq m_torqueOut;
OutPort<TimedDoubleSeq> *op_torqueOut;

//* *//
TimedDoubleSeq m_qvel_sim;
TimedDoubleSeq m_torque_sim;
TimedDoubleSeq m_rfsensor_sim;
TimedDoubleSeq m_lfsensor_sim;
TimedDoubleSeq m_rhsensor_sim;
TimedDoubleSeq m_lhsensor_sim;
TimedAcceleration3D m_gsensor_sim;
TimedAngularVelocity3D m_gyrometer_sim;

InPort<TimedDoubleSeq> *ip_qvel_sim;
InPort<TimedDoubleSeq> *ip_torque_sim;
InPort<TimedDoubleSeq> *ip_rfsensor_sim;
InPort<TimedDoubleSeq> *ip_lfsensor_sim;
InPort<TimedDoubleSeq> *ip_rhsensor_sim;
InPort<TimedDoubleSeq> *ip_lhsensor_sim;
InPort<TimedAcceleration3D> *ip_gsensor_sim;
InPort<TimedAngularVelocity3D> *ip_gyrometer_sim;

static void readGainFile();

double dt;
std::ifstream gain;
std::string gain_fname;
std::vector<double> qold, qold_ref, Pgain, Dgain;
std::vector<double> Pgain_orig, Dgain_orig;
size_t dof, loop;
unsigned int m_debugLevel;

#if (defined __APPLE__)
typedef int clockid_t;
#define CLOCK_MONOTONIC 0
#include <mach/mach_time.h>  
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (clk_id != CLOCK_MONOTONIC) return -1;

    uint64_t clk;
    clk = mach_absolute_time();  

    static mach_timebase_info_data_t info = {0,0};  
    if (info.denom == 0) mach_timebase_info(&info);  
  
    uint64_t elapsednano = clk * (info.numer / info.denom);  
  
    tp->tv_sec = elapsednano * 1e-9;  
    tp->tv_nsec = elapsednano - (tp->tv_sec * 1e9);  
    return 0;
}

#define TIMER_ABSTIME 0
int clock_nanosleep(clockid_t clk_id, int flags, struct timespec *tp,
    struct timespec *remain)
{
    if (clk_id != CLOCK_MONOTONIC || flags != TIMER_ABSTIME) return -1;

    static mach_timebase_info_data_t info = {0,0};  
    if (info.denom == 0) mach_timebase_info(&info);  
  
    uint64_t clk = (tp->tv_sec*1e9 + tp->tv_nsec)/(info.numer/info.denom);
    
    mach_wait_until(clk);
    return 0;
}
#endif


int number_of_joints()
{
    return (int)command.size();
}

int number_of_force_sensors()
{
    return (int)forces.size();
}

int number_of_gyro_sensors()
{
    return (int)gyros.size();
}

int number_of_accelerometers()
{
    return (int)accelerometers.size();
}

int number_of_attitude_sensors()
{
    return (int)attitude_sensors.size();
}

int set_number_of_joints(int num)
{
    fprintf(stderr, "set_number_of_joints\n");
    command.resize(num);
    act_angle.resize(num);
    com_torque.resize(num);
    act_torque.resize(num);
    act_vel.resize(num);
    power.resize(num);
    servo.resize(num);
    for (int i=0; i<num; i++){
        command[i] = power[i] = servo[i] = 0;
    }

    return TRUE;
}

int set_number_of_force_sensors(int num)
{
    forces.resize(num);
    force_offset.resize(num);
    force_queue.resize(FORCE_AVERAGE);
    force_counter = 0;
    for (int cnt = 0; cnt < FORCE_AVERAGE; cnt++) {
      force_queue[cnt].resize(num);
      for (int n = 0; n < num; n++) {
        force_queue[cnt][n].resize(6);
        for(int i = 0; i < 6; i++) {
          force_queue[cnt][n][i] = 0;
        }
      }
    }
    for (unsigned int i=0; i<forces.size();i++){
        forces[i].resize(6);
        force_offset[i].resize(6);
        for (int j=0; j<6; j++){
            forces[i][j] = 0;
            force_offset[i][j] = 0;
        }
    }
    return TRUE;
}

int set_number_of_gyro_sensors(int num)
{
    gyros.resize(num);
    gyro_offset.resize(num);
    for (unsigned int i=0; i<gyros.size();i++){
        gyros[i].resize(3);
        gyro_offset[i].resize(3);
        for (int j=0; j<3; j++){
            gyros[i][j] = 0.0;
            gyro_offset[i][j] = 0.0;
        }
    }
    return TRUE;
}

int set_number_of_accelerometers(int num)
{
    accelerometers.resize(num);
    accel_offset.resize(num);
    for (unsigned int i=0; i<accelerometers.size();i++){
        accelerometers[i].resize(3);
        accel_offset[i].resize(3);
        for (int j=0; j<3; j++){
            accelerometers[i][j] = j == 2 ? 9.81 : 0.0;
            accel_offset[i][j] = 0;
        }
    }
    return TRUE;
}

int set_number_of_attitude_sensors(int num)
{
    attitude_sensors.resize(num);
    for (unsigned int i=0; i<attitude_sensors.size();i++){
        attitude_sensors[i].resize(3);
        for (int j=0; j<3; j++){
            attitude_sensors[i][j] = 0.0;
        }
    }
    return TRUE;
}

int read_power_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    *s = power[id];
    return TRUE;
}

int write_power_command(int id, int com)
{
    CHECK_JOINT_ID(id);
    power[id] = com;
    return TRUE;
}

int read_power_command(int id, int *com)
{
    CHECK_JOINT_ID(id);
    *com = power[id];
    return TRUE;
}

int read_servo_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    *s = servo[id];
    return TRUE;
}

int read_servo_alarm(int id, int *a)
{
    CHECK_JOINT_ID(id);
    *a = 0;
    return TRUE;
}
    
int read_control_mode(int id, joint_control_mode *s)
{
    CHECK_JOINT_ID(id);
    *s = JCM_POSITION;
    return TRUE;
}

int write_control_mode(int id, joint_control_mode s)
{
    CHECK_JOINT_ID(id);
    return TRUE;
}

int read_actual_angle(int id, double *angle)
{
    CHECK_JOINT_ID(id);
    *angle = act_angle[id];
    return TRUE;
}

int read_actual_angles(double *angles)
{
    //fprintf(stderr, "read\n");
    for (int i=0; i<number_of_joints(); i++){
      angles[i] = act_angle[i];
    }
    return TRUE;
}

int read_actual_torques(double *torques)
{
    for (int i=0; i<number_of_joints(); i++){
      torques[i] = act_torque[i];
    }
    return TRUE;
}

int read_command_torque(int id, double *torque)
{
    CHECK_JOINT_ID(id);
    *torque = com_torque[id];
    return TRUE;
}

int write_command_torque(int id, double torque)
{
    CHECK_JOINT_ID(id);
    com_torque[id] = torque;
    return TRUE;
}

int read_command_torques(double *torques)
{
    for (int i=0; i<number_of_joints(); i++){
      torques[i] = com_torque[i];
    }
    return TRUE;
}

int write_command_torques(const double *torques)
{
    for (int i=0; i<number_of_joints(); i++){
      com_torque[i] = torques[i];
    }
    return TRUE;
}

int read_command_angle(int id, double *angle)
{
    CHECK_JOINT_ID(id);
    *angle = command[id];
    return TRUE;
}

int write_command_angle(int id, double angle)
{
    CHECK_JOINT_ID(id);
    command[id] = angle;
    return TRUE;
}

int read_command_angles(double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        angles[i] = command[i];
    }
    return TRUE;
}

int write_command_angles(const double *angles)
{
    //fprintf(stderr, "write angles\n");
    for (int i=0; i<number_of_joints(); i++){
        command[i] = angles[i];
    }
    return TRUE;
}

int read_pgain(int id, double *gain)
{
    CHECK_JOINT_ID(id);
    if (id == 9)
    std::cerr << "read_pgain: [" << id << "] " << Pgain[id] << std::endl;
    *gain = Pgain[id];
    return TRUE;
}

int write_pgain(int id, double gain)
{
    CHECK_JOINT_ID(id);
    if (id == 9)
    std::cerr << "write_pgain: [" << id << "] " << gain << std::endl;
    Pgain[id] = gain;
    return TRUE;
}

int read_dgain(int id, double *gain)
{
    CHECK_JOINT_ID(id);
    if (id == 9)
    std::cerr << "read_dgain: [" << id << "] " << Dgain[id] << std::endl;
    *gain = Dgain[id];
    return TRUE;
}

int write_dgain(int id, double gain)
{
    CHECK_JOINT_ID(id);
    if (id == 9)
    std::cerr << "write_dgain: [" << id << "] " << gain << std::endl;
    Dgain[id] = gain;
    return TRUE;
}

int read_force_sensor(int id, double *fs)
{
    CHECK_FORCE_SENSOR_ID(id);
    for (int i=0; i<6; i++){
      fs[i] = forces[id][i] + force_offset[id][i]; // 2 = initial offset
    }
    return TRUE;
}

int read_gyro_sensor(int id, double *rates)
{
    CHECK_GYRO_SENSOR_ID(id);
    for(int i = 0; i < 3; i++) {
      rates[i] = gyros[id][i];
    }
    return TRUE;
}

int read_accelerometer(int id, double *accels)
{
    CHECK_ACCELEROMETER_ID(id);
    for(int i = 0; i < 3; i++) {
      accels[i] = accelerometers[id][i];
    }
    return TRUE;
}

int read_touch_sensors(unsigned short *onoff)
{
    return FALSE;
}

int read_attitude_sensor(int id, double *att)
{
    return FALSE;
}

int read_current(int id, double *mcurrent)
{
    return FALSE;
}

int read_current_limit(int id, double *v)
{
    return FALSE;
}

int read_currents(double *currents)
{
    return FALSE;
}

int read_gauges(double *gauges)
{
    return FALSE;
}

int read_actual_velocity(int id, double *vel)
{
    *vel = act_vel[id];
    return TRUE;
}

int read_command_velocity(int id, double *vel)
{
    return FALSE;
}

int write_command_velocity(int id, double vel)
{
    return FALSE;
}

int read_actual_velocities(double *vels)
{
    for (int i = 0; i < number_of_joints(); i++) {
      vels[i] = act_vel[i];
    }
    return TRUE;
}

int read_command_velocities(double *vels)
{
    return FALSE;
}

int write_command_velocities(const double *vels)
{
    return FALSE;
}

int read_temperature(int id, double *v)
{
    return FALSE;
}

int write_servo(int id, int com)
{
    servo[id] = com;
    return TRUE;
}

int write_dio(unsigned short buf)
{
    return FALSE;
}

int open_iob(void)
{
    std::cerr << "choreonoid IOB will open" << std::endl;
    for (int i=0; i<number_of_joints(); i++){
        command[i] = 0.0;
        power[i] = OFF;
        servo[i] = OFF;
    }
    clock_gettime(CLOCK_MONOTONIC, &g_ts);

    self_ptr->bindParameter("pdgains_sim.file_name", gain_fname, "");
    self_ptr->bindParameter("debugLevel", m_debugLevel, "0");
    //
    dt = 0.001; // fixed number
    //* for PD controller *//
    ip_angleIn    = new InPort<TimedDoubleSeq> ("angleIn", m_angleIn);
    op_torqueOut  = new OutPort<TimedDoubleSeq> ("torqueOut", m_torqueOut);

    self_ptr->addInPort("angleIn", *ip_angleIn);
    self_ptr->addOutPort("torqueOut", *op_torqueOut);

    //* pass through port *//
    ip_qvel_sim     = new InPort<TimedDoubleSeq> ("qvel_sim", m_qvel_sim);
    ip_torque_sim   = new InPort<TimedDoubleSeq> ("torque_sim", m_torque_sim);
    ip_rfsensor_sim = new InPort<TimedDoubleSeq> ("rfsensor_sim", m_rfsensor_sim);
    ip_lfsensor_sim = new InPort<TimedDoubleSeq> ("lfsensor_sim", m_lfsensor_sim);
    ip_rhsensor_sim = new InPort<TimedDoubleSeq> ("rhsensor_sim", m_rhsensor_sim);
    ip_lhsensor_sim = new InPort<TimedDoubleSeq> ("lhsensor_sim", m_lhsensor_sim);
    ip_gsensor_sim   = new InPort<TimedAcceleration3D> ("gsensor_sim", m_gsensor_sim);
    ip_gyrometer_sim = new InPort<TimedAngularVelocity3D> ("gyrometer_sim", m_gyrometer_sim);

    self_ptr->addInPort("qvel_sim", *ip_qvel_sim);
    self_ptr->addInPort("torque_sim", *ip_torque_sim);
    self_ptr->addInPort("rfsensor_sim", *ip_rfsensor_sim);
    self_ptr->addInPort("lfsensor_sim", *ip_lfsensor_sim);
    self_ptr->addInPort("rhsensor_sim", *ip_rhsensor_sim);
    self_ptr->addInPort("lhsensor_sim", *ip_lhsensor_sim);
    self_ptr->addInPort("gsensor_sim", *ip_gsensor_sim);
    self_ptr->addInPort("gyrometer_sim", *ip_gyrometer_sim);

    std::cerr << "choreonoid IOB is opened" << std::endl;
    return TRUE;
}
void iob_update(void)
{
    if(ip_angleIn->isNew()) {
      ip_angleIn->read();
      if (dof == 0) {
        dof = m_angleIn.data.length();
        m_torqueOut.data.length(dof);
        readGainFile();
      }
      for(int i = 0; i < m_angleIn.data.length(); i++) {
        act_angle[i] = m_angleIn.data[i];
      }
      iob_time.sec = m_angleIn.tm.sec;
      iob_time.nsec = m_angleIn.tm.nsec;
    }
    if(ip_qvel_sim->isNew()) {
      ip_qvel_sim->read();
      for(int i = 0; i < m_qvel_sim.data.length(); i++) {
        act_vel[i] = m_qvel_sim.data[i];
      }
    }
    if(ip_torque_sim->isNew()) {
      ip_torque_sim->read();
      for(int i = 0; i < m_torque_sim.data.length(); i++) {
        act_torque[i] = m_torque_sim.data[i];
      }
    }
    //* *//
    if(ip_rfsensor_sim->isNew()) {
      ip_rfsensor_sim->read();
      for(int i = 0; i < 6; i++) {
        //forces[0][i] = m_rfsensor_sim.data[i];
        force_queue[force_counter][0][i] = m_rfsensor_sim.data[i];
      }
    }
    if(ip_lfsensor_sim->isNew()) {
      ip_lfsensor_sim->read();
      for(int i = 0; i < 6; i++) {
        //forces[1][i] = m_lfsensor_sim.data[i];
        force_queue[force_counter][1][i] = m_lfsensor_sim.data[i];
      }
    }
    if(ip_rhsensor_sim->isNew()) {
      ip_rhsensor_sim->read();
      for(int i = 0; i < 6; i++) {
        //forces[2][i] = m_rhsensor_sim.data[i];
        force_queue[force_counter][2][i] = m_rhsensor_sim.data[i];
      }
    }
    if(ip_lhsensor_sim->isNew()) {
      ip_lhsensor_sim->read();
      for(int i = 0; i < 6; i++) {
        //forces[3][i] = m_lhsensor_sim.data[i];
        force_queue[force_counter][3][i] = m_lhsensor_sim.data[i];
      }
    }
    force_counter++;
    if (force_counter >= FORCE_AVERAGE) {
      force_counter = 0;
    }
    for(int n = 0; n < number_of_force_sensors(); n++) {
      for(int i = 0; i < 6; i++) {
        forces[n][i] = 0.0;
      }
      for(int cnt = 0; cnt < FORCE_AVERAGE; cnt++) {
        for(int i = 0; i < 6; i++) {
          forces[n][i] += force_queue[cnt][n][i];
        }
      }
      for(int i = 0; i < 6; i++) {
        forces[n][i] /= FORCE_AVERAGE;
      }
    }

    //* *//
    if(ip_gsensor_sim->isNew()) {
      ip_gsensor_sim->read();
      accelerometers[0][0] = m_gsensor_sim.data.ax;
      accelerometers[0][1] = m_gsensor_sim.data.ay;
      accelerometers[0][2] = m_gsensor_sim.data.az;
    }
    if(ip_gyrometer_sim->isNew()) {
      ip_gyrometer_sim->read();
      gyros[0][0] = m_gyrometer_sim.data.avx;
      gyros[0][1] = m_gyrometer_sim.data.avy;
      gyros[0][2] = m_gyrometer_sim.data.avz;
    }

    //std::cerr << "tm: " << m_angleIn.tm.sec << " / " << m_angleIn.tm.nsec << std::endl; 
}
void iob_finish(void)
{
    //* *//
    for(int i=0; i<dof; i++){
      double q = act_angle[i]; // current angle
      double q_ref = command[i];
      double dq = (q - qold[i]) / dt;
      double dq_ref = (q_ref - qold_ref[i]) / dt;
      qold[i] = q;
      qold_ref[i] = q_ref;
      double tq = -(q - q_ref) * Pgain[i] - (dq - dq_ref) * Dgain[i];
      //double tlimit = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
      double tlimit;
      if (i < 15) {
        // torso/leg
        tlimit = 1200;
      } else if (i < 33) {
        // arm/head
        tlimit = 600;
      } else {
        // hand
        tlimit = 140;
      }
      m_torqueOut.data[i] = std::max(std::min(tq, tlimit), -tlimit);
#if 1
      if (i == (dof - 1)) { // for range joint, fixed rotational rate of 2.0 [rad/sec]
        static double dq_old = 0.0;
        double ddq = (dq - dq_old)/dt;
        tq = -(dq - 1.0) * 100 - 0.2 * ddq;
        double tlimit = 200;
        m_torqueOut.data[i] = std::max(std::min(tq, tlimit), -tlimit);
        //std::cerr << "tau: " << m_torque.data[i] << ", q: " << q << ", dq: " << dq << ", dq_old: " << dq_old << ", ddq: " << ddq << std::endl;
        dq_old = dq;
      }
#endif
#if 0
      if (loop % 100 == 0 && m_debugLevel == 1) {
        std::cerr << "[" << m_profile.instance_name << "] joint = "
                  << i << ", tq = " << m_torque.data[i] << ", q,qref = (" << q << ", " << q_ref << "), dq,dqref = (" << dq << ", " << dq_ref << "), pd = (" << Pgain[i] << ", " << Dgain[i] << "), tlimit = " << tlimit << std::endl;
      }
#endif
    }
    m_torqueOut.tm = m_angleIn.tm;
    op_torqueOut->write();
    //* *//
}
static void readGainFile()
{
    if (gain_fname == "") {
        RTC::Properties& prop = self_ptr->getProperties();
        coil::stringTo(gain_fname, prop["pdgains_sim_file_name"].c_str());
    }
    // initialize length of vectors
    m_torqueOut.data.length(dof);
    m_angleIn.data.length(dof);

    qold.resize(dof);
    qold_ref.resize(dof);
    Pgain.resize(dof);
    Dgain.resize(dof);
    gain.open(gain_fname.c_str());
    if (gain.is_open()){
      double tmp;
      for (int i=0; i<dof; i++){
          if (gain >> tmp) {
              Pgain[i] = tmp;
          } else {
            //std::cerr << "[" << m_profile.instance_name << "] Gain file [" << gain_fname << "] is too short" << std::endl;
          }
          if (gain >> tmp) {
              Dgain[i] = tmp;
          } else {
            //std::cerr << "[" << m_profile.instance_name << "] Gain file [" << gain_fname << "] is too short" << std::endl;
          }
      }
      gain.close();
      std::cerr << "[iob] Gain file [" << gain_fname << "] opened" << std::endl;
    }else{
      std::cerr << "[iob] Gain file [" << gain_fname << "] not opened" << std::endl;
    }
    // initialize angleRef, old_ref and old with angle
    for(int i = 0; i < dof; ++i) {
      command[i] = qold_ref[i] = qold[i] = m_angleIn.data[i];
    }
}

int close_iob(void)
{
    std::cerr << "choreonoid IOB is closed" << std::endl;
    return TRUE;
}

int reset_body(void)
{
    for (int i=0; i<number_of_joints(); i++){
        power[i] = servo[i] = OFF;
    }
    return TRUE;
}

int joint_calibration(int id, double angle)
{
    return FALSE;
}

int read_gyro_sensor_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        offset[i] = gyro_offset[id][i];
    }
    return TRUE;
}

int write_gyro_sensor_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        gyro_offset[id][i] = offset[i];
    }
    return TRUE;
}

int read_accelerometer_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        offset[i] = accel_offset[id][i];
    }
    return TRUE;
}

int write_accelerometer_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        accel_offset[id][i] = offset[i];
    }
    return TRUE;
}

int read_force_offset(int id, double *offsets)
{
    for (int i=0; i<6; i++){
        offsets[i] = force_offset[id][i];
    }
    return TRUE;
}

int write_force_offset(int id, double *offsets)
{
    for (int i=0; i<6; i++){
        force_offset[id][i] = offsets[i];
    }
    return TRUE;
}

int write_attitude_sensor_offset(int id, double *offset)
{
    return FALSE;
}

int read_calib_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    int v = id/2;
    *s = v%2==0 ? ON : OFF;
    return TRUE;
}

int lock_iob()
{
    if (isLocked) return FALSE;

    isLocked = true;
    return TRUE;
}
int unlock_iob()
{
    isLocked = false;
    return TRUE;
}

int read_lock_owner(pid_t *pid)
{
  return FALSE;
}

int read_limit_angle(int id, double *angle)
{
  return FALSE;
}

int read_angle_offset(int id, double *angle)
{
  return FALSE;
}

int write_angle_offset(int id, double angle)
{
  return FALSE;
}

int read_ulimit_angle(int id, double *angle)
{
  return FALSE;
}
int read_llimit_angle(int id, double *angle)
{
  return FALSE;
}
int read_encoder_pulse(int id, double *ec)
{
  return FALSE;
}
int read_gear_ratio(int id, double *gr)
{
  return FALSE;
}
int read_torque_const(int id, double *tc)
{
  return FALSE;
}
int read_torque_limit(int id, double *limit)
{
  return FALSE;
}

unsigned long long read_iob_frame()
{
    ++frame;
    if (frame == 5) frame = 0;
    return frame;
}

int number_of_substeps()
{
    return 5;
}

int read_power(double *voltage, double *current)
{
    *voltage = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*1+48;
    *current = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+1;
    return TRUE;
}

#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
int number_of_batteries()
{
    return 1;
}

int read_battery(int id, double *voltage, double *current, double *soc)
{
    *voltage = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*1+48;
    *current = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+1;
    *soc = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+50;
    return TRUE;
}

int number_of_thermometers()
{
    return 0;
}

#endif

int read_driver_temperature(int id, unsigned char *v)
{
    *v = id * 2;
    return TRUE;
}

void timespec_add_ns(timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec > 1e9){
        ts->tv_sec += 1;
        ts->tv_nsec -= 1e9;
    }
}

double timespec_compare(timespec *ts1, timespec *ts2)
{
    double dts = ts1->tv_sec - ts2->tv_sec;
    double dtn = ts1->tv_nsec - ts2->tv_nsec;
    return dts*1e9+dtn;
}

int wait_for_iob_signal()
{
    //fprintf(stderr, "wfis\n");
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_ts, 0);
    timespec_add_ns(&g_ts, g_period_ns);
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = timespec_compare(&g_ts, &now);
    if (dt <= 0){
        //printf("overrun(%d[ms])\n", -dt*1e6);
        do {
            timespec_add_ns(&g_ts, g_period_ns);
        }while(timespec_compare(&g_ts, &now)<=0);
    }
    return 0;
}

size_t length_of_extra_servo_state(int id)
{
    return 0;
}

int read_extra_servo_state(int id, int *state)
{
    return TRUE;
}

int set_signal_period(long period_ns)
{
    g_period_ns = period_ns;
    return TRUE;
}

long get_signal_period()
{
    return g_period_ns;
}

int initializeJointAngle(const char *name, const char *option)
{
    sleep(3);
    return TRUE;
}

int read_digital_input(char *dinput)
{
    return FALSE;
}

int length_digital_input()
{
    return 0;
}

int write_digital_output(const char *doutput)
{
    return FALSE;
}

int write_digital_output_with_mask(const char *doutput, const char *mask)
{
    return FALSE;
}

int length_digital_output()
{
    return 0;
}

int read_digital_output(char *doutput)
{
    return FALSE;
}


