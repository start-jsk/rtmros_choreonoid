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
static std::vector<bool> isPosTq;

Time iob_time;

#define FORCE_AVERAGE 8
#define TORQUE_AVERAGE 2

static int force_counter;
static std::vector< std::vector<std::vector<double> > > force_queue;
static int torque_counter;
static std::vector<std::vector<double> > torque_queue;

#define CHECK_JOINT_ID(id) if ((id) < 0 || (id) >= number_of_joints()) return E_ID
#define CHECK_FORCE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_force_sensors()) return E_ID
#define CHECK_ACCELEROMETER_ID(id) if ((id) < 0 || (id) >= number_of_accelerometers()) return E_ID
#define CHECK_GYRO_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_gyro_sensors()) return E_ID
#define CHECK_ATTITUDE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_attitude_sensors()) return E_ID

using namespace RTC;

RobotHardware_choreonoid *self_ptr;

//* *//
static TimedDoubleSeq m_angleIn;
static InPort<TimedDoubleSeq> *ip_angleIn;
static TimedDoubleSeq m_torqueOut;
static OutPort<TimedDoubleSeq> *op_torqueOut;

//* *//
static TimedDoubleSeq m_qvel_sim;
static TimedDoubleSeq m_torque_sim;
static TimedDoubleSeq m_rfsensor_sim;
static TimedDoubleSeq m_lfsensor_sim;
static TimedDoubleSeq m_rhsensor_sim;
static TimedDoubleSeq m_lhsensor_sim;
static TimedAcceleration3D m_gsensor_sim;
static TimedAngularVelocity3D m_gyrometer_sim;

static InPort<TimedDoubleSeq> *ip_qvel_sim;
static InPort<TimedDoubleSeq> *ip_torque_sim;
static InPort<TimedDoubleSeq> *ip_rfsensor_sim;
static InPort<TimedDoubleSeq> *ip_lfsensor_sim;
static InPort<TimedDoubleSeq> *ip_rhsensor_sim;
static InPort<TimedDoubleSeq> *ip_lhsensor_sim;
static InPort<TimedAcceleration3D> *ip_gsensor_sim;
static InPort<TimedAngularVelocity3D> *ip_gyrometer_sim;

static void readGainFile();

static double dt;
static std::ifstream gain;
static std::string gain_fname;
static std::vector<double> qold, qold_ref, Pgain, Dgain, initial_Pgain, initial_Dgain;
static std::vector<double> tqold, tqold_ref, tqPgain, tqDgain, initial_tqPgain, initial_tqDgain;
static std::vector<double> Pgain_orig, Dgain_orig;
static std::vector<double> tlimit;
static size_t dof, loop;
static unsigned int m_debugLevel;

static int iob_step;
static int iob_nstep;

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
    fprintf(stderr, "set_number_of_joints %d\n", num);
    command.resize(num);
    act_angle.resize(num);
    com_torque.resize(num);
    act_torque.resize(num);
    act_vel.resize(num);
    power.resize(num);
    servo.resize(num);
    isPosTq.resize(num);

    for (int i=0; i<num; i++){
        command[i] = com_torque[i] = power[i] = servo[i] = 0;
        isPosTq[i] = false;
    }

    torque_counter = 0;
    torque_queue.resize(TORQUE_AVERAGE);
    for (int j = 0; j < TORQUE_AVERAGE; j++) {
      torque_queue[j].resize(num);
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
    if(isPosTq[id]){
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 4
      *s = JCM_POSITION_TORQUE;
#endif
    }else{
      *s = JCM_POSITION;
    }
    return TRUE;
}

int write_control_mode(int id, joint_control_mode s)
{
    CHECK_JOINT_ID(id);
    if(s == JCM_POSITION){
      isPosTq[id] = false;
    }
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 4
    if(s == JCM_POSITION_TORQUE){
      isPosTq[id] = true;
    }
#endif
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
    iob_step = iob_nstep;
    return TRUE;
}

int read_pgain(int id, double *gain)
{
    CHECK_JOINT_ID(id);
#if 0
    if (id == 9)
    std::cerr << "read_pgain: [" << id << "] " << Pgain[id] << std::endl;
#endif
    if (initial_Pgain[id]==0) {
      *gain = 0;
    } else {
      *gain = Pgain[id] / initial_Pgain[id];
    }
    return TRUE;
}

int write_pgain(int id, double gain)
{
    CHECK_JOINT_ID(id);
#if 0
    if (id == 9)
    std::cerr << "write_pgain: [" << id << "] " << gain << std::endl;
#endif
    Pgain[id] = gain * initial_Pgain[id];
    return TRUE;
}

int read_dgain(int id, double *gain)
{
    CHECK_JOINT_ID(id);
#if 0
    if (id == 9)
    std::cerr << "read_dgain: [" << id << "] " << Dgain[id] << std::endl;
#endif
    if (initial_Dgain[id]==0) {
      *gain = 0;
    } else {
      *gain = Dgain[id] / initial_Dgain[id];
    }
    return TRUE;
}

int write_dgain(int id, double gain)
{
    CHECK_JOINT_ID(id);
#if 0
    if (id == 9)
    std::cerr << "write_dgain: [" << id << "] " << gain << std::endl;
#endif
    Dgain[id] = gain * initial_Dgain[id];
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
    //printf("iob_write_servo %d %d\n", id, com);
    servo[id] = com;
    return TRUE;
}

int write_dio(unsigned short buf)
{
    return FALSE;
}

///
///
///
int open_iob(void)
{
    std::cerr << "choreonoid IOB will open" << std::endl;
    for (int i=0; i<number_of_joints(); i++){
        command[i] = 0.0;
        com_torque[i] = 0.0;
        power[i] = OFF;
        servo[i] = OFF;
        isPosTq[i] = false;
    }
    clock_gettime(CLOCK_MONOTONIC, &g_ts);

    self_ptr->bindParameter("pdgains_sim_file_name", gain_fname, "");
    self_ptr->bindParameter("debugLevel", m_debugLevel, "0");
    //
    dt = 0.001; // fixed number or read from param
    {
      double hrpsys_dt = 0.002;
      RTC::Properties& prop = self_ptr->getProperties();
      coil::stringTo(hrpsys_dt, prop["dt"].c_str());
      iob_step = hrpsys_dt/dt;
      iob_nstep = iob_step;
      std::cerr << "hrpsys cycle = " << hrpsys_dt << " [sec]";
      std::cerr << ", iob_step = " << iob_step << std::endl;
    }

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

    // initial servo and power on
    for(int i = 0; i < servo.size(); i++) {
      power[i] = 1;
      servo[i] = 1;
    }

    std::cerr << "choreonoid IOB is opened" << std::endl;
    return TRUE;
}
///
/// called from RobotHardware_choreonoid
/// update sensor date
///
void iob_update(void)
{
    if(ip_angleIn->isNew()) {
      ip_angleIn->read();
      if (dof == 0) {
        dof = m_angleIn.data.length();
        m_torqueOut.data.length(dof);
        readGainFile();
      }
      for(int i = 0; i < m_angleIn.data.length() && i < dof; i++) {
        act_angle[i] = m_angleIn.data[i];
      }
      iob_time.sec = m_angleIn.tm.sec;
      iob_time.nsec = m_angleIn.tm.nsec;
    }
    if(ip_qvel_sim->isNew()) {
      ip_qvel_sim->read();
      for(int i = 0; i < m_qvel_sim.data.length() && i < dof; i++) {
        act_vel[i] = m_qvel_sim.data[i];
      }
    }
    if(ip_torque_sim->isNew()) {
      ip_torque_sim->read();
      for(int i = 0; i < m_torque_sim.data.length() && i < dof; i++) {
        //act_torque[i] = m_torque_sim.data[i];
        torque_queue[torque_counter][i] = m_torque_sim.data[i];
      }
      torque_counter++;
      if (torque_counter >= TORQUE_AVERAGE) {
        torque_counter = 0;
      }
    }
    //* *//
    if(ip_rfsensor_sim->isNew()) {
      ip_rfsensor_sim->read();
      if(number_of_force_sensors() >= 1) {
        for(int i = 0; i < 6; i++) {
          //forces[0][i] = m_rfsensor_sim.data[i];
          force_queue[force_counter][0][i] = m_rfsensor_sim.data[i];
        }
      }
    }
    if(ip_lfsensor_sim->isNew()) {
      ip_lfsensor_sim->read();
      if(number_of_force_sensors() >= 2) {
        for(int i = 0; i < 6; i++) {
          //forces[1][i] = m_lfsensor_sim.data[i];
          force_queue[force_counter][1][i] = m_lfsensor_sim.data[i];
        }
      }
    }
    if(ip_rhsensor_sim->isNew()) {
      ip_rhsensor_sim->read();
      if(number_of_force_sensors() >= 3) {
        for(int i = 0; i < 6; i++) {
          //forces[2][i] = m_rhsensor_sim.data[i];
          force_queue[force_counter][2][i] = m_rhsensor_sim.data[i];
        }
      }
    }
    if(ip_lhsensor_sim->isNew()) {
      ip_lhsensor_sim->read();
      if(number_of_force_sensors() >= 4) {
        for(int i = 0; i < 6; i++) {
          //forces[3][i] = m_lhsensor_sim.data[i];
          force_queue[force_counter][3][i] = m_lhsensor_sim.data[i];
        }
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
    for(int i = 0; i < dof; i++) {
      act_torque[i] = 0;
    }
    for(int cnt = 0; cnt < TORQUE_AVERAGE; cnt++) {
      for(int i = 0; i < dof; i++) {
        act_torque[i] += torque_queue[cnt][i];
      }
    }
    for(int i = 0; i < dof; i++) {
      act_torque[i] /= TORQUE_AVERAGE;
    }

    //* *//
    if(ip_gsensor_sim->isNew()) {
      ip_gsensor_sim->read();
      if(number_of_accelerometers() >= 1) {
        accelerometers[0][0] = m_gsensor_sim.data.ax;
        accelerometers[0][1] = m_gsensor_sim.data.ay;
        accelerometers[0][2] = m_gsensor_sim.data.az;
      }
    }
    if(ip_gyrometer_sim->isNew()) {
      ip_gyrometer_sim->read();
      if(number_of_gyro_sensors() >= 1) {
        gyros[0][0] = m_gyrometer_sim.data.avx;
        gyros[0][1] = m_gyrometer_sim.data.avy;
        gyros[0][2] = m_gyrometer_sim.data.avz;
      }
    }
    //std::cerr << "tm: " << m_angleIn.tm.sec << " / " << m_angleIn.tm.nsec << std::endl; 
}
///
/// called from RobotHardware_choreonoid
///
void iob_set_torque_limit(std::vector<double> &vec)
{
  tlimit.resize(vec.size());
  for(int i = 0; i < vec.size(); i++) {
    tlimit[i] = vec[i];
    std::cerr << "joint: " << i;
    std::cerr << ", tlimit: " << tlimit[i] << std::endl;
  }
}
///
/// called from RobotHardware_choreonoid
/// update command to simulated robot
///
void iob_finish(void)
{
    //* *//
    for(int i=0; i<dof; i++) {
      // position
      double q = act_angle[i]; // current angle
      double q_ref = iob_step > 0 ? qold_ref[i] + (command[i] - qold_ref[i])/iob_step : qold_ref[i];
      double dq = (q - qold[i]) / dt;
      double dq_ref = (q_ref - qold_ref[i]) / dt;
      qold[i] = q;
      qold_ref[i] = q_ref;
      // torque
      double tq = act_torque[i]; // current torque
      double tq_ref = iob_step > 0 ? tqold_ref[i] + (com_torque[i] - tqold_ref[i])/iob_step : tqold_ref[i];
      double dtq = (tq - tqold[i]) / dt;
      double dtq_ref = (tq_ref - tqold_ref[i]) / dt;
      tqold[i] = tq;
      tqold_ref[i] = tq_ref;

      double ctq;
      if(isPosTq[i]){
        // position & torque control
        //ctq = -(q - q_ref) * Pgain[i] - (dq - dq_ref) * Dgain[i] - (tq - tq_ref) * tqPgain[i] - (dtq - dtq_ref) * tqDgain[i];
        ctq = -(q - q_ref) *   Pgain[i] / (tqPgain[i] + 1) - (dq  -  dq_ref) *   Dgain[i] / (tqPgain[i] + 1)
                +  tq_ref  * tqPgain[i] / (tqPgain[i] + 1) - (dtq - dtq_ref) * tqDgain[i] / (tqPgain[i] + 1);
      } else {
        ctq = -(q - q_ref) * Pgain[i] - (dq - dq_ref) * Dgain[i]; // simple PD control
      }

      m_torqueOut.data[i] = std::max(std::min(ctq, tlimit[i]), -tlimit[i]);
#if 0
      if (i == 23) {
        std::cerr << "[iob] step = " << iob_step << ", joint = "
                  << i << ", tq = " << m_torqueOut.data[i] << ", q,qref = (" << q << ", " << q_ref << "), dq,dqref = (" << dq << ", " << dq_ref << "), pd = (" << Pgain[i] << ", " << Dgain[i] << "), tlimit = " << tlimit[i] << std::endl;
      }
#endif
    }
    iob_step--;
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
    tqold.resize(dof);
    tqold_ref.resize(dof);
    Pgain.resize(dof);
    Dgain.resize(dof);
    tqPgain.resize(dof);
    tqDgain.resize(dof);
    initial_Pgain.resize(dof);
    initial_Dgain.resize(dof);
    initial_tqPgain.resize(dof);
    initial_tqDgain.resize(dof);
    gain.open(gain_fname.c_str());
    if (gain.is_open()) {
      std::cerr << "[iob] Gain file [" << gain_fname << "] opened" << std::endl;
      double tmp;
      int i = 0;
      for (; i < dof; i++) {

      retry:
        {
          std::string str;
          if (std::getline(gain, str)) {
            if (str.empty())   goto retry;
            if (str[0] == '#') goto retry;

            std::istringstream sstrm(str);
            sstrm >> tmp;
            Pgain[i] = initial_Pgain[i] = tmp;
            if(sstrm.eof()) goto next;
            sstrm >> tmp;
            Dgain[i] = initial_Dgain[i] = tmp;
            if(sstrm.eof()) goto next;
            sstrm >> tmp;
            tqPgain[i] = initial_tqPgain[i] = tmp;
            if(sstrm.eof()) goto next;
            sstrm >> tmp;
            tqDgain[i] = initial_tqDgain[i] = tmp;
          } else {
            i--;
            break;
          }
        }

      next:
        std::cerr << "joint: " << i;
        std::cerr << ", P: " << Pgain[i];
        std::cerr << ", D: " << Dgain[i];
        std::cerr << ", tqP: " << tqPgain[i];
        std::cerr << ", tqD: " << tqDgain[i] << std::endl;
      }
      gain.close();
      if (i != dof) {
        std::cerr << "[iob] Gain file [" << gain_fname << "] does not contain gains for all joints" << std::endl;
      }
    } else {
      std::cerr << "[iob] Gain file [" << gain_fname << "] not opened" << std::endl;
    }
    // initialize angleRef, old_ref and old with angle
    for(int i = 0; i < dof; ++i) {
      command[i] = qold_ref[i] = qold[i] = m_angleIn.data[i];
      com_torque[i] = tqold_ref[i] = tqold[i] = 0/*act_torque[i]*/;
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
    //CHECK_JOINT_ID(id);
    //int v = id/2;
    //*s = v%2==0 ? ON : OFF;
    *s = 0;
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

#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 3
int write_command_acceleration(int id, double acc)
{
    return FALSE;
}

int write_command_accelerations(const double *accs)
{
    return FALSE;
}

int write_joint_inertia(int id, double mn)
{
    return FALSE;
}

int write_joint_inertias(const double *mns)
{
    return FALSE;
}

int read_pd_controller_torques(double *torques)
{
    return FALSE;
}

int write_disturbance_observer(int com)
{
    return FALSE;
}

int write_disturbance_observer_gain(double gain)
{
    return FALSE;
}
#endif

#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 4
int read_torque_pgain(int id, double *gain)
{
    CHECK_JOINT_ID(id);
#if 0
    if (id == 9)
    std::cerr << "read_pgain: [" << id << "] " << Pgain[id] << std::endl;
#endif
    if (initial_tqPgain[id]==0) {
      *gain = 0;
    } else {
      *gain = tqPgain[id] / initial_tqPgain[id];
    }
    return TRUE;
}

int write_torque_pgain(int id, double gain)
{
    CHECK_JOINT_ID(id);
#if 0
    if (id == 9)
    std::cerr << "write_pgain: [" << id << "] " << gain << std::endl;
#endif
    tqPgain[id] = gain * initial_tqPgain[id];
    return TRUE;
}

int read_torque_dgain(int id, double *gain)
{
    CHECK_JOINT_ID(id);
#if 0
    if (id == 9)
    std::cerr << "read_dgain: [" << id << "] " << Dgain[id] << std::endl;
#endif
    if (initial_tqDgain[id]==0) {
      *gain = 0;
    } else {
      *gain = tqDgain[id] / initial_tqDgain[id];
    }
    return TRUE;
}

int write_torque_dgain(int id, double gain)
{
    CHECK_JOINT_ID(id);
#if 0
    if (id == 9)
    std::cerr << "write_dgain: [" << id << "] " << gain << std::endl;
#endif
    tqDgain[id] = gain * initial_tqDgain[id];
    return TRUE;
}
#endif

int read_driver_temperature(int id, unsigned char *v)
{
    //*v = id * 2;
    *v = 0;
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


