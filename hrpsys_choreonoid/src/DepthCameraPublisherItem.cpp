#include "DepthCameraPublisherItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>

namespace cnoid {

  void DepthCameraPublisherItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<DepthCameraPublisherItem>("DepthCameraPublisherItem");
  }

  DepthCameraPublisherItem::DepthCameraPublisherItem(){
    if(!ros::isInitialized()){
      QStringList argv_list = QCoreApplication::arguments();
      int argc = argv_list.size();
      char* argv[argc];
      //なぜかわからないがargv_list.at(i).toUtf8().data()のポインタをそのままargvに入れるとros::initがうまく解釈してくれない.
      for(size_t i=0;i<argv_list.size();i++){
        char* data = argv_list.at(i).toUtf8().data();
        size_t dataSize = 0;
        for(size_t j=0;;j++){
          if(data[j] == '\0'){
            dataSize = j;
            break;
          }
        }
        argv[i] = (char *)malloc(sizeof(char) * dataSize+1);
        for(size_t j=0;j<dataSize;j++){
          argv[i][j] = data[j];
        }
        argv[i][dataSize] = '\0';
      }
      ros::init(argc,argv,"choreonoid");
      for(size_t i=0;i<argc;i++){
        free(argv[i]);
      }
    }

  }

  void DepthCameraPublisherItem::setupROS() {
    if(this->setupROSDone_) return;
    this->setupROSDone_ = true;

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    if(this->publishColor_){
      std::string topicName;
      if(this->imageTopicName_!="") topicName = this->imageTopicName_;
      else topicName = this->cameraName_+"/color/image_raw";
      this->imagePub_ = it.advertise(topicName, 1);

      std::string infoName;
      if(this->cameraInfoTopicName_!="") infoName = this->cameraInfoTopicName_;
      else infoName = this->cameraName_+"/color/camera_info";
      this->infoPub_ = nh.advertise<sensor_msgs::CameraInfo>(infoName, 1);
    }

    if(this->publishDepth_){
      std::string depthTopicName;
      if(this->depthImageTopicName_!="") depthTopicName = this->depthImageTopicName_;
      else depthTopicName = this->cameraName_+"/depth/image_raw";
      this->depthImagePub_ = it.advertise(depthTopicName, 1);

      std::string depthInfoName;
      if(this->depthCameraInfoTopicName_!="") depthInfoName = this->depthCameraInfoTopicName_;
      else depthInfoName = this->cameraName_+"/depth/camera_info";
      this->depthInfoPub_ = nh.advertise<sensor_msgs::CameraInfo>(depthInfoName, 1);
    }

    if(this->publishPointCloud_){
      std::string pointTopicName;
      if(this->pointCloudTopicName_!="") pointTopicName = this->pointCloudTopicName_;
      else pointTopicName = this->cameraName_+"/depth_registered/points";
      this->pointCloudPub_ = nh.advertise<sensor_msgs::PointCloud2>(pointTopicName, 1);
    }
  }

  bool DepthCameraPublisherItem::initialize(ControllerIO* io) {
    this->io_ = io;
    this->timeStep_ = io->worldTimeStep();

    setupROS(); // コンストラクタやcallLaterだとname()やrestore()が未完了

    return true;
  }

  bool DepthCameraPublisherItem::start() {
    this->sensor_ = this->io_->body()->findDevice<cnoid::RangeCamera>(this->cameraName_);
    if (this->sensor_) {
      this->sensor_->sigStateChanged().connect(boost::bind(&DepthCameraPublisherItem::updateVisionSensor, this));
      return true;
    }else{
      this->io_->os() << "\e[0;31m" << "[DepthCameraPublisherItem] camera [" << this->cameraName_ << "] not found"  << "\e[0m" << std::endl;
      return false;
    }
  }

  void DepthCameraPublisherItem::updateVisionSensor() {
    std_msgs::Header header;
    header.stamp.fromSec(std::max(0.0, this->io_->currentTime() - this->sensor_->delay()));
    if(this->frameId_.size()!=0) header.frame_id = this->frameId_;
    else header.frame_id = this->sensor_->name();

    sensor_msgs::CameraInfo info;
    {
      info.header = header;
      info.width  = this->sensor_->image().width();
      info.height = this->sensor_->image().height();
      info.distortion_model = "plumb_bob";
      info.K[0] = std::min(info.width, info.height) / 2 / tan(this->sensor_->fieldOfView()/2);
      info.K[2] = (this->sensor_->image().width()-1)/2.0;
      info.K[4] = info.K[0];
      info.K[5] = (this->sensor_->image().height()-1)/2.0;
      info.K[8] = 1;
      info.P[0] = info.K[0];
      info.P[2] = info.K[2];
      info.P[5] = info.K[4];
      info.P[6] = info.K[5];
      info.P[10] = 1;
      info.R[0] = info.R[4] = info.R[8] = 1;
    }

    if(this->publishColor_){
      sensor_msgs::Image vision;
      {
        vision.header = header;
        vision.height = this->sensor_->image().height();
        vision.width = this->sensor_->image().width();
        if (this->sensor_->image().numComponents() == 3)
          vision.encoding = sensor_msgs::image_encodings::RGB8;
        else if (this->sensor_->image().numComponents() == 1)
          vision.encoding = sensor_msgs::image_encodings::MONO8;
        else {
          ROS_WARN("unsupported image component number: %i", this->sensor_->image().numComponents());
        }
        vision.is_bigendian = 0;
        vision.step = this->sensor_->image().width() * this->sensor_->image().numComponents();
        vision.data.resize(vision.step * vision.height);
        std::memcpy(&(vision.data[0]), &(this->sensor_->image().pixels()[0]), vision.step * vision.height);
      }
      this->imagePub_.publish(vision);

      this->infoPub_.publish(info);
    }

    if(this->publishDepth_){
      sensor_msgs::Image depth;
      {
        depth.header = header;
        depth.width = this->sensor_->resolutionX();
        depth.height = this->sensor_->resolutionY();
        depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        depth.step  = depth.width * 4;
        depth.data.resize(depth.step * depth.height);
        float* dst_ptr = (float *)depth.data.data();
        const std::vector<Vector3f>& points = this->sensor_->constPoints();
        for(int i = 0; i < points.size(); i++) {
          // OpenHRP3 camera, front direction is -Z axis, ROS camera is Z axis
          // http://www.openrtp.jp/openhrp3/jp/create_model.html
          float z = points[i].z() * -1;
          if( z < this->minDistance_ ) z = -std::numeric_limits<float>::infinity(); // Detections that are too close to the sensor to quantify shall be represented by -Inf. https://www.ros.org/reps/rep-0117.html
          *dst_ptr++ = z;
        }
      }
      this->depthImagePub_.publish(depth);

      this->depthInfoPub_.publish(info);
    }

    if(this->publishPointCloud_){
      sensor_msgs::PointCloud2 pointcloud;
      {
        pointcloud.header = header;
        pointcloud.width = this->sensor_->resolutionX();
        pointcloud.height = this->sensor_->resolutionY();
        pointcloud.is_bigendian = false;
        pointcloud.is_dense = true;
        if (this->sensor_->imageType() == cnoid::Camera::COLOR_IMAGE) {
          pointcloud.fields.resize(4);
          pointcloud.fields[3].name = "rgb";
          pointcloud.fields[3].offset = 12;
          pointcloud.fields[3].count = 1;
          pointcloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
          pointcloud.point_step = 16;
        } else {
          pointcloud.fields.resize(3);
          pointcloud.point_step = 12;
        }
        pointcloud.fields[0].name = "x";
        pointcloud.fields[0].offset = 0;
        pointcloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud.fields[0].count = 1;
        pointcloud.fields[1].name = "y";
        pointcloud.fields[1].offset = 4;
        pointcloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud.fields[1].count = 1;
        pointcloud.fields[2].name = "z";
        pointcloud.fields[2].offset = 8;
        pointcloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud.fields[2].count = 1;
        pointcloud.row_step = pointcloud.point_step * pointcloud.width;
        const std::vector<Vector3f>& points = this->sensor_->constPoints();
        const unsigned char* pixels = this->sensor_->constImage().pixels();
        pointcloud.data.resize(points.size() * pointcloud.point_step);
        unsigned char* dst = (unsigned char*)&(pointcloud.data[0]);
        for (size_t j = 0; j < points.size(); ++j) {
          // OpenHRP3 camera, front direction is -Z axis, ROS camera is Z axis
          // http://www.openrtp.jp/openhrp3/jp/create_model.html
          float x = points[j].x();
          float y = - points[j].y();
          float z = - points[j].z();
          if( z < this->minDistance_ ) z = -std::numeric_limits<float>::infinity(); // Detections that are too close to the sensor to quantify shall be represented by -Inf. https://www.ros.org/reps/rep-0117.html
          std::memcpy(&dst[0], &x, 4);
          std::memcpy(&dst[4], &y, 4);
          std::memcpy(&dst[8], &z, 4);
          if (this->sensor_->imageType() == cnoid::Camera::COLOR_IMAGE) {
            dst[14] = *pixels++;
            dst[13] = *pixels++;
            dst[12] = *pixels++;
            dst[15] = 0;
          }
          dst += pointcloud.point_step;
        }
      }
      this->pointCloudPub_.publish(pointcloud);
    }
  }

  bool DepthCameraPublisherItem::store(Archive& archive) {
    archive.write("cameraName", this->cameraName_);
    archive.write("imageTopicName", this->imageTopicName_);
    archive.write("cameraInfoTopicName", this->cameraInfoTopicName_);
    archive.write("depthImageTopicName", this->depthImageTopicName_);
    archive.write("depthCameraInfoTopicName", this->depthCameraInfoTopicName_);
    archive.write("pointCloudTopicName", this->pointCloudTopicName_);
    archive.write("frameId", this->frameId_);
    archive.write("minDistance", this->minDistance_);
    archive.write("publishColor", this->publishColor_);
    archive.write("publishDepth", this->publishDepth_);
    archive.write("publishPointCloud", this->publishPointCloud_);
    return true;
  }

  bool DepthCameraPublisherItem::restore(const Archive& archive) {
    archive.read("cameraName", this->cameraName_);
    archive.read("imageTopicName", this->imageTopicName_);
    archive.read("cameraInfoTopicName", this->cameraInfoTopicName_);
    archive.read("depthImageTopicName", this->depthImageTopicName_);
    archive.read("depthCameraInfoTopicName", this->depthCameraInfoTopicName_);
    archive.read("pointCloudTopicName", this->pointCloudTopicName_);
    archive.read("frameId", this->frameId_);
    archive.read("minDistance", this->minDistance_);
    archive.read("publishColor", this->publishColor_);
    archive.read("publishDepth", this->publishDepth_);
    archive.read("publishPointCloud", this->publishPointCloud_);
    return true;
  }

}
