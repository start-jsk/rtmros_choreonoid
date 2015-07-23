/**
   Vision sensor reader for the JVRC robot model.
*/

#include "VisionReaderRTC.h"
//#include <cnoid/BodyMotion>
//#include <cnoid/ExecutablePath>
//#include <cnoid/FileUtil>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const char* samplepd_spec[] =
{
    "implementation_id", "VisionReaderRTC",
    "type_name",         "VisionReaderRTC",
    "description",       "Vision data bridge",
    "version",           "0.1",
    "vendor",            "JSK",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};
}


VisionReaderRTC::VisionReaderRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      //m_angleIn("q", m_angle),
      //m_gsensorIn("gsensor", m_gsensor),
      //m_gyrometerIn("gyrometer", m_gyrometer),
      //m_lfsensorIn("lfsensor", m_lfsensor),
      //m_rfsensorIn("rfsensor", m_rfsensor),
      m_lcameraIn("lcamera", m_lcamera),
      m_rcameraIn("rcamera", m_rcamera),
      m_lcameraIn("lcamera", m_lhcamera),
      m_rcameraIn("rcamera", m_rhcamera),
      m_rangerIn("ranger", m_ranger)
{

}

VisionReaderRTC::~VisionReaderRTC()
{

}


RTC::ReturnCode_t VisionReaderRTC::onInitialize()
{
    // Set InPort buffers
    //addInPort("q", m_angleIn);
    //addInPort("gsensor", m_gsensorIn);
    //addInPort("gyrometer", m_gyrometerIn);
    //addInPort("lfsensor", m_lfsensorIn);
    //addInPort("rfsensor", m_rfsensorIn);
    addInPort("lcamera", m_lcameraIn);
    addInPort("rcamera", m_rcameraIn);
    addInPort("lhcamera", m_lhcameraIn);
    addInPort("rhcamera", m_rhcameraIn);
    addInPort("ranger", m_rangerIn);

    cout << "Vision Sensor RTC initialized" << endl;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t VisionReaderRTC::onActivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t VisionReaderRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t VisionReaderRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_lcameraIn.isNew()) {
      m_lcameraIn.read();
      std::cout << "lcamera" << std::endl;
      //std::cout << "width: " << m_lcamera.data.image.width << std::endl;
      //std::cout << "height: " << m_lcamera.data.image.height << std::endl;
      //std::cout << "fmt: " << m_lcamera.data.image.format << std::endl;
      std::cout << "width: " << m_lcamera.width << std::endl;
      std::cout << "height: " << m_lcamera.height << std::endl;
      std::cout << "type: " << m_lcamera.type << std::endl;
      // for(size_t i=0; i < m_lcamera.data.image.raw_data.length(); ++i){
      // 	cout << "m_lcamera.data.image.raw_data[" << i <<
      // 	  "] is " << m_lcamera.data.image.raw_data[i] << std::endl;
      // }
    }
    if(m_rcameraIn.isNew()) {
      m_rcameraIn.read();
      std::cout << "rcamera" << std::endl;
      std::cout << "width: " << m_rcamera.data.image.width << std::endl;
      std::cout << "height: " << m_rcamera.data.image.height << std::endl;
      std::cout << "fmt: " << m_rcamera.data.image.format << std::endl;
      // for(size_t i=0; i < m_rcamera.data.image.raw_data.length(); ++i){
      // 	cout << "m_rcamera.data.image.raw_data[" << i <<
      // 	  "] is " << m_rcamera.data.image.raw_data[i] << std::endl;
      // }
    }
    if(m_lhcameraIn.isNew()) {
      m_lhcameraIn.read();
      std::cout << "lhcamera" << std::endl;
    }
    if(m_rhcameraIn.isNew()) {
      m_rhcameraIn.read();
      std::cout << "rhcamera" << std::endl;
    }
    if(m_rangerIn.isNew()){
      m_rangerIn.read();
      //for(size_t i=0; i < m_ranger.ranges.length(); ++i){
      //cout << "m_ranger.ranges[" << i << "] is " << m_ranger.ranges[i] << std::endl;
      //}
    }

    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void VisionReaderRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(samplepd_spec);
        manager->registerFactory(profile,
                                 RTC::Create<VisionReaderRTC>,
                                 RTC::Delete<VisionReaderRTC>);
    }
};
