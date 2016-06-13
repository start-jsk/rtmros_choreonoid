
#include <cmath>
#include <cstring>
#include <boost/function.hpp>
#include <boost/bind.hpp>

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
};

struct DOORCustomizer
{
  BodyHandle bodyHandle;

  bool hasVirtualJoints;
  JointValSet jointValSets[2];
  //double springT;
  //double dampingT;
  //double springR;
  //double dampingR;
};


static const char** getTargetModelNames()
{
  static const char* names[] = {
    "DOOR_WALL",
    0 };
  return names;
}


static void getHingeJoints(DOORCustomizer* customizer, BodyHandle body)
{
  customizer->hasVirtualJoints = true;

  int hinge_joint = bodyInterface->getLinkIndexFromName(body, "DOOR_WALL_JOINT0");
  int handle_joint = bodyInterface->getLinkIndexFromName(body, "DOOR_WALL_JOINT1");

  if (hinge_joint >= 0) {
    JointValSet& jointValSet = customizer->jointValSets[0];
    jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, hinge_joint);
    jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, hinge_joint);
    jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, hinge_joint);
  } else {
    customizer->hasVirtualJoints = false;
    std::cerr << "fail to find out joint 0" << std::endl;
  }
  if (handle_joint >= 0) {
    JointValSet& jointValSet = customizer->jointValSets[1];
    jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, handle_joint);
    jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, handle_joint);
    jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, handle_joint);
  } else {
    customizer->hasVirtualJoints = false;
    std::cerr << "fail to find out joint 1" << std::endl;
  }
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
  DOORCustomizer* customizer = 0;

  std::cerr << "create customizer : " << std::string(modelName) << std::endl;
  customizer = new DOORCustomizer;

  customizer->bodyHandle = bodyHandle;
  customizer->hasVirtualJoints = false;

  getHingeJoints(customizer, bodyHandle);

  return static_cast<BodyCustomizerHandle>(customizer);
}


static void destroy(BodyCustomizerHandle customizerHandle)
{
  DOORCustomizer* customizer = static_cast<DOORCustomizer*>(customizerHandle);
  if(customizer){
    delete customizer;
  }
}

#define HANDLE_P_GAIN 400
#define HANDLE_D_GAIN 40
#define HANDLE_MAX_ANG 1.6
#define HANDLE_DAMPING 10

#define HINGE_P_GAIN 1000
#define HINGE_D_GAIN 100
#define HINGE_MAX_ANG 3.0
#define HINGE_DAMPING 10

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
  DOORCustomizer* customizer = static_cast<DOORCustomizer*>(customizerHandle);

  if(customizer->hasVirtualJoints){
    JointValSet& hinge_info = customizer->jointValSets[0];
    JointValSet& handle_info = customizer->jointValSets[1];
    { // handle
      if (*handle_info.valuePtr < 0) {
        *(handle_info.torqueForcePtr) = - HANDLE_P_GAIN * (*handle_info.valuePtr)
          - HANDLE_D_GAIN * (*handle_info.velocityPtr);
      } else if (*handle_info.valuePtr > HANDLE_MAX_ANG) {
        *(handle_info.torqueForcePtr) = - HANDLE_P_GAIN * ((*handle_info.valuePtr) - HANDLE_MAX_ANG)
          - HANDLE_D_GAIN * (*handle_info.velocityPtr);
      } else {
        *(handle_info.torqueForcePtr) = - HANDLE_DAMPING * (*handle_info.velocityPtr);
      }
    }
    { // hinge
      bool lached = lached;
      if (((*handle_info.valuePtr) < 0.35) && // about 20 deg
          ((*handle_info.valuePtr) > -0.1)) {
        lached = true;
      }
      //std::cerr << "j1 : ";
      //std::cerr << (*handle_info.valuePtr) << " / " << *(handle_info.torqueForcePtr) << std::endl;
      //std::cerr << "j0 : ";
      if (*hinge_info.valuePtr < 0) {
        //std::cerr << "0 : ";
        *(hinge_info.torqueForcePtr) = - HINGE_P_GAIN * (*hinge_info.valuePtr)
          - HINGE_D_GAIN * (*hinge_info.velocityPtr);
      } else if (*hinge_info.valuePtr > HINGE_MAX_ANG) {
        //std::cerr << "1 : ";
        *(hinge_info.torqueForcePtr) = - HINGE_P_GAIN * ((*hinge_info.valuePtr) - HINGE_MAX_ANG)
          - HINGE_D_GAIN * (*hinge_info.velocityPtr);
      } else {
        if (lached && (*hinge_info.valuePtr < 0.1)) {
          //std::cerr << "2 : ";
          *(hinge_info.torqueForcePtr) = - HINGE_P_GAIN * (*hinge_info.valuePtr)
            - HINGE_D_GAIN * (*hinge_info.velocityPtr);
        } else {
          //std::cerr << "3 : ";
          // case 0  //free
          //*(hinge_info.torqueForcePtr) = 0;
          // case 1  //just damping
          *(hinge_info.torqueForcePtr) = - HINGE_DAMPING * (*hinge_info.velocityPtr);
          // case 2  // coor clooser
          //*(hinge_info.torqueForcePtr) = - 50 * (*hinge_info.valuePtr) - 10 * (*hinge_info.velocityPtr);
        }
      }
      //std::cerr << (*hinge_info.valuePtr) << " / " << *(hinge_info.torqueForcePtr) << std::endl;
    }
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
