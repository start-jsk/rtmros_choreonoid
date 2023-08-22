#ifndef CNOIDROSEXTPLUGIN_DEPTHCAMERAPUBLISHER_ITEM_H
#define CNOIDROSEXTPLUGIN_DEPTHCAMERAPUBLISHER_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/RangeCamera>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace cnoid {

  class DepthCameraPublisherItem : public ControllerItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    DepthCameraPublisherItem();

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;

    virtual double timeStep() const override { return timeStep_;};
    virtual void input() override {}
    virtual bool control() override { return true;}
    virtual void output() override {}
    virtual void stop() override {}

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

  protected:
    void setupROS(); bool setupROSDone_ = false;
    void updateVisionSensor();

    image_transport::Publisher imagePub_;
    ros::Publisher infoPub_;
    image_transport::Publisher depthImagePub_;
    ros::Publisher depthInfoPub_;
    ros::Publisher pointCloudPub_;

    std::string cameraName_;
    std::string imageTopicName_;
    std::string cameraInfoTopicName_;
    std::string depthImageTopicName_;
    std::string depthCameraInfoTopicName_;
    std::string pointCloudTopicName_;
    std::string frameId_;
    double minDistance_ = 0.0;
    bool publishColor_ = true;
    bool publishDepth_ = true;
    bool publishPointCloud_ = true;

    cnoid::ControllerIO* io_;
    cnoid::RangeCameraPtr sensor_;
    double timeStep_;
  };

  typedef ref_ptr<DepthCameraPublisherItem> DepthCameraPublisherItemPtr;
}

#endif
