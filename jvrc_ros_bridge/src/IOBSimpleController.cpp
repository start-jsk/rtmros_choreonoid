/**
   Sample controller
*/

#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>
//#include <cnoid/BodyMotion>
//#include <cnoid/ExecutablePath>
//#include <cnoid/FileUtil>

// For ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <iostream>

using namespace std;
using namespace cnoid;

class IOBSimpleController : public cnoid::SimpleController
{
  BodyPtr body;
  boost::shared_ptr<ros::NodeHandle> nh_;

public:
  virtual bool initialize() {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle("~"));

    std::cerr << "IOBSimpleContoller initialized" << std::endl;


    body = ioBody();
    std::cerr << "numJoints: " << body->numJoints() << std::endl;
    std::cerr << "numLinks: " << body->numLinks() << std::endl;
    std::cerr << "numDevies: " << body->numDevices() << std::endl;

    DeviceList<ForceSensor> forceSensors = body->devices();
    //forceSensors.size();
    for(DeviceList<ForceSensor> ::iterator it = forceSensors.begin();
	it != forceSensors.end();
	it++) {
      std::cerr << "idx: " << (*it)->index() << std::endl;
      std::cerr << "id: " << (*it)->id() << std::endl;
      std::cerr << "name: " << (*it)->name() << std::endl;
      /// it->sigStateChanged().connect(
    }
    DeviceList<RateGyroSensor> rateSensors = body->devices();
    for(DeviceList<RateGyroSensor> ::iterator it = rateSensors.begin();
	it != rateSensors.end();
	it++) {
      std::cerr << "idx: " << (*it)->index() << std::endl;
      std::cerr << "id: " << (*it)->id() << std::endl;
      std::cerr << "name: " << (*it)->name() << std::endl;
      /// it->sigStateChanged().connect(
    }
    DeviceList<AccelSensor> accelSensors = body->devices();
    for(DeviceList<AccelSensor> ::iterator it = accelSensors.begin();
	it != accelSensors.end();
	it++) {
      std::cerr << "idx: " << (*it)->index() << std::endl;
      std::cerr << "id: " << (*it)->id() << std::endl;
      std::cerr << "name: " << (*it)->name() << std::endl;
      /// it->sigStateChanged().connect(
    }
    DeviceList<Camera> cameras = body->devices();
    for(DeviceList<Camera> ::iterator it = cameras.begin();
	it != cameras.end();
	it++) {
      std::cerr << "idx: " << (*it)->index() << std::endl;
      std::cerr << "id: " << (*it)->id() << std::endl;
      std::cerr << "name: " << (*it)->name() << std::endl;
      /// it->sigStateChanged().connect(
    }
    DeviceList<RangeSensor> rangeSensors = body->devices();
    for(DeviceList<RangeSensor> ::iterator it = rangeSensors.begin();
	it != rangeSensors.end();
	it++) {
      std::cerr << "idx: " << (*it)->index() << std::endl;
      std::cerr << "id: " << (*it)->id() << std::endl;
      std::cerr << "name: " << (*it)->name() << std::endl;
      /// it->sigStateChanged().connect(
    }
    DeviceList<RangeCamera> rcameras = body->devices();
    for(DeviceList<RangeCamera> ::iterator it = rcameras.begin();
	it != rcameras.end();
	it++) {
      std::cerr << "idx: " << (*it)->index() << std::endl;
      std::cerr << "id: " << (*it)->id() << std::endl;
      std::cerr << "name: " << (*it)->name() << std::endl;
      /// it->sigStateChanged().connect(
    }
    return true;
  }

  virtual bool control() {
    //std::cerr << "control" << std::endl;
    //sleep(1);
    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(IOBSimpleController)
