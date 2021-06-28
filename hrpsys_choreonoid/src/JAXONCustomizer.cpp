#include <stdlib.h>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <yaml-cpp/yaml.h>

#define CNOID_BODY_CUSTOMIZER
#ifdef CNOID_BODY_CUSTOMIZER
#include <cnoid/BodyCustomizerInterface>
#else
#include <BodyCustomizerInterface.h>
#endif

#include <iostream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif /* Windows */

#if defined(HRPMODEL_VERSION_MAJOR) && defined(HRPMODEL_VERSION_MINOR)
#if HRPMODEL_VERSION_MAJOR >= 3 && HRPMODEL_VERSION_MINOR >= 1
#include <hrpUtil/Tvmet3dTypes.h>
#define NS_HRPMODEL hrp
#endif
#endif

#ifdef CNOID_BODY_CUSTOMIZER
#define NS_HRPMODEL cnoid
cnoid::Matrix3 trans(const cnoid::Matrix3& M) { return M.transpose(); }
double dot(const cnoid::Vector3& a, const cnoid::Vector3& b) { return a.dot(b); }
typedef cnoid::Matrix3 Matrix33;
#endif


#ifndef NS_HRPMODEL
#define NS_HRPMODEL OpenHRP
typedef OpenHRP::vector3 Vector3;
typedef OpenHRP::matrix33 Matrix33;
#endif

using namespace std;
using namespace boost;
using namespace NS_HRPMODEL;

static BodyInterface* bodyInterface = 0;

static BodyCustomizerInterface bodyCustomizerInterface;

struct JointValSet
{
  double* valuePtr;
  double* velocityPtr;
  double* torqueForcePtr;
  double spring;
  double damping;
};

struct JAXONCustomizer
{
  BodyHandle bodyHandle;

  std::vector<JointValSet> jointValSets;

  double tilt_upper_bound;
  double tilt_lower_bound;
  double tilt_positive_speed;
  double tilt_negative_speed;

  //* *//
  bool hasMultisenseJoint;
  JointValSet multisense_joint;

  //* *//
  bool hasTiltLaserJoint;
  JointValSet tilt_laser_joint;
};


static const char** getTargetModelNames()
{
  char *rname = getenv("CHOREONOID_ROBOT");
  if (rname != NULL) {
    std::cerr << "CHOREONID_ROBOT =" << rname << std::endl;
  }
  static const char* names[] = {
    "JAXON_JVRC",
    "JAXON_BLUE",
    "CHIDORI",
    "TABLIS",
    rname,
    0 };

  return names;
}

static void getVirtualbushJoints(JAXONCustomizer* customizer, BodyHandle body, const YAML::Node& param)
{
  double default_springT  = 1.1e6; // N/m
  double default_dampingT = 1.1e3; // N/(m/s)
  double default_springR  = 2.5e3; // Nm / rad
  double default_dampingR = 2.5;   // Nm / (rad/s)
  if (param && param["bush"]) {
    if (param["bush"]["springT"]) default_springT = param["bush"]["springT"].as<double>();
    if (param["bush"]["dampingT"]) default_dampingT = param["bush"]["dampingT"].as<double>();
    if (param["bush"]["springR"]) default_springR = param["bush"]["springR"].as<double>();
    if (param["bush"]["dampingR"]) default_dampingR = param["bush"]["dampingR"].as<double>();
  }

  std::vector<std::string> bushes{"RLEG", "LLEG", "RARM", "LARM"};
  if (param && param["bush"] && param["bush"]["bushes"]) bushes = param["bush"]["bushes"].as<std::vector<std::string>>();
  std::vector<std::string> types{"X", "Y", "Z", "ROLL", "PITCH", "YAW"};

  for(int i=0; i < bushes.size(); ++i){
    for(int j=0; j < types.size(); ++j){
      std::string bush_name = bushes[i] + "_BUSH_" + types[j];
      int bushIndex = bodyInterface->getLinkIndexFromName(body, bush_name.c_str());

      if(bushIndex < 0){
        std::cerr << "[Customizer] failed to find out : " << bush_name << std::endl;
      }else{
        std::cerr << "[Customizer] find out : " << bush_name << std::endl;

        JointValSet jointValSet;
        jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, bushIndex);
        jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, bushIndex);
        jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, bushIndex);

        if (j<3) {
          jointValSet.spring = default_springT;
          jointValSet.damping = default_dampingT;
          if (param && param["bush"] && param["bush"][bushes[i]]){
            if (param["bush"][bushes[i]]["springT"]) jointValSet.spring = param["bush"][bushes[i]]["springT"].as<double>();
            if (param["bush"][bushes[i]]["dampingT"]) jointValSet.damping = param["bush"][bushes[i]]["dampingT"].as<double>();
          }
        } else {
          jointValSet.spring = default_springR;
          jointValSet.damping = default_dampingR;
          if (param && param["bush"] && param["bush"][bushes[i]]){
            if (param["bush"][bushes[i]]["springR"]) jointValSet.spring = param["bush"][bushes[i]]["springR"].as<double>();
            if (param["bush"][bushes[i]]["dampingR"]) jointValSet.damping = param["bush"][bushes[i]]["dampingR"].as<double>();
          }
        }

        customizer->jointValSets.push_back(jointValSet);
      }
    }
  }

  //* additional motor joints *//
  {
    int bindex = bodyInterface->getLinkIndexFromName(body, "motor_joint");
    customizer->hasMultisenseJoint = true;
    if(bindex < 0){
      customizer->hasMultisenseJoint = false;
    } else {
      JointValSet& jointValSet = customizer->multisense_joint;
      jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, bindex);
      jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, bindex);
      jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, bindex);
    }
  }

  //* additional motor joints *//
  {
    int bindex = bodyInterface->getLinkIndexFromName(body, "tilt_laser_joint");
    customizer->hasTiltLaserJoint = true;
    if(bindex < 0){
      customizer->hasTiltLaserJoint = false;
    } else {
      JointValSet& jointValSet = customizer->tilt_laser_joint;
      jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, bindex);
      jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, bindex);
      jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, bindex);
    }
  }
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
  JAXONCustomizer* customizer = 0;

  std::cerr << "create customizer : " << std::string(modelName) << std::endl;
  customizer = new JAXONCustomizer;

  customizer->bodyHandle = bodyHandle;

  customizer->tilt_upper_bound = 1.35; // rad
  customizer->tilt_lower_bound = -0.7; // rad
  customizer->tilt_positive_speed = 1.0; // rad/s
  customizer->tilt_negative_speed = -1.0; // rad/s

  YAML::Node param;
  char* config_file_path = getenv("CUSTOMIZER_CONF_FILE");
  if (config_file_path) {
    ifstream ifs(config_file_path);
    if (ifs.is_open()) {
      std::cerr << "[JAXONCustomizer] Config file is: " << config_file_path << std::endl;
      param = YAML::LoadFile(config_file_path);
      if (param["tilt_laser"]) {
        if (param["tilt_laser"]["TILT_UPPER_BOUND"]) customizer->tilt_upper_bound = param["tilt_laser"]["TILT_UPPER_BOUND"].as<double>();
        if (param["tilt_laser"]["TILT_POSITIVE_SPEED"]) customizer->tilt_positive_speed = param["tilt_laser"]["TILT_POSITIVE_SPEED"].as<double>();
        if (param["tilt_laser"]["TILT_LOWER_BOUND"]) customizer->tilt_lower_bound = param["tilt_laser"]["TILT_LOWER_BOUND"].as<double>();
        if (param["tilt_laser"]["TILT_NEGATIVE_SPEED"]) customizer->tilt_negative_speed = param["tilt_laser"]["TILT_NEGATIVE_SPEED"].as<double>();
      }
    } else {
      std::cerr << "[JAXONCustomizer] " << config_file_path << " is not found" << std::endl;
    }
  }

  getVirtualbushJoints(customizer, bodyHandle, param);

  return static_cast<BodyCustomizerHandle>(customizer);
}


static void destroy(BodyCustomizerHandle customizerHandle)
{
  JAXONCustomizer* customizer = static_cast<JAXONCustomizer*>(customizerHandle);
  if(customizer){
    delete customizer;
  }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
  JAXONCustomizer* customizer = static_cast<JAXONCustomizer*>(customizerHandle);

  for(int i=0; i < customizer->jointValSets.size(); ++i){
    JointValSet& bush = customizer->jointValSets[i];
    *(bush.torqueForcePtr) = - bush.spring * (*bush.valuePtr) - bush.damping * (*bush.velocityPtr);
    //std::cerr << i << " " << 0 << " " << *(bush.torqueForcePtr) << " = " << -bush.spring << " x " << *bush.valuePtr << " + " <<  - bush.damping << " x " << *bush.velocityPtr << std::endl;
  }

  if(customizer->hasMultisenseJoint) {
    JointValSet& trans = customizer->multisense_joint;
    static double dq_old = 0.0;
    double dq = *(trans.velocityPtr);
    double ddq = (dq - dq_old)/0.001; // dt = 0.001
    double tq = -(dq - 1.0) * 100 - 0.2 * ddq;
    double tlimit = 200;
    *(trans.torqueForcePtr) = std::max(std::min(tq, tlimit), -tlimit);
    dq_old = dq;
  }

  if(customizer->hasTiltLaserJoint) {
    JointValSet& trans = customizer->tilt_laser_joint;
    static double dq_old = 0.0;
    static bool move_positive = true;
    double  q = *(trans.valuePtr);
    if (q > customizer->tilt_upper_bound) {
      move_positive = false;
    } else if (q < customizer->tilt_lower_bound) {
      move_positive = true;
    }

    double dq = *(trans.velocityPtr);
    double tq;
    double ddq = (dq - dq_old) / 0.001; // dt = 0.001
    if (move_positive) {
      tq = -(dq - customizer->tilt_positive_speed) * 100 - 0.2 * ddq;
    } else {
      tq = -(dq - customizer->tilt_negative_speed) * 100 - 0.2 * ddq;
    }
    double tlimit = 200;
    *(trans.torqueForcePtr) = std::max(std::min(tq, tlimit), -tlimit);
    dq_old = dq;
  }
}

extern "C" DLL_EXPORT
NS_HRPMODEL::BodyCustomizerInterface* getHrpBodyCustomizerInterface(NS_HRPMODEL::BodyInterface* bodyInterface_)
{
  bodyInterface = bodyInterface_;

  bodyCustomizerInterface.version = NS_HRPMODEL::BODY_CUSTOMIZER_INTERFACE_VERSION;
  bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
  bodyCustomizerInterface.create = create;
  bodyCustomizerInterface.destroy = destroy;
  bodyCustomizerInterface.initializeAnalyticIk = NULL;
  bodyCustomizerInterface.calcAnalyticIk = NULL;
  bodyCustomizerInterface.setVirtualJointForces = setVirtualJointForces;

  return &bodyCustomizerInterface;
}
