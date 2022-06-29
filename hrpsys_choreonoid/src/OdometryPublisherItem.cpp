#include "OdometryPublisherItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <nav_msgs/Odometry.h>

namespace cnoid {

  void OdometryPublisherItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<OdometryPublisherItem>("OdometryPublisherItem");
  }

  OdometryPublisherItem::OdometryPublisherItem(){
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

  void OdometryPublisherItem::setupROS() {
    if(this->setupROSDone_) return;
    this->setupROSDone_ = true;

    ros::NodeHandle nh;

    std::string topicName;
    if(this->odometryTopicName_!="") topicName = this->odometryTopicName_;
    else topicName = this->targetName_+"/odom";
    this->pub_ = nh.advertise<nav_msgs::Odometry>(topicName, 1);
  }

  bool OdometryPublisherItem::initialize(ControllerIO* io) {
    this->io_ = io;
    this->timeStep_ = io->worldTimeStep();

    setupROS(); // コンストラクタやcallLaterだとname()やrestore()が未完了

    return true;
  }

  bool OdometryPublisherItem::start() {
    this->link_ = this->io_->body()->link(this->targetName_);
    if(!link_) this->sensor_ = this->io_->body()->findDevice<cnoid::Camera>(this->targetName_);
    if (this->link_ || this->sensor_) {
      return true;
    }else{
      this->io_->os() << "\e[0;31m" << "[OdometryPublisherItem] camera [" << this->targetName_ << "] not found"  << "\e[0m" << std::endl;
      return false;
    }
  }

  bool OdometryPublisherItem::control() {
    if(!this->link_ && !this->sensor_) return true;

    if(this->timeStep_ < 1.0 / this->publishRate_){
      this->time_ += this->timeStep_;
      if(this->time_ < 1.0 / this->publishRate_) return true;
      this->time_ -= 1.0 / this->publishRate_;
    }

    std_msgs::Header header;
    header.stamp.fromSec(this->io_->currentTime());
    if(this->frameId_.size()!=0) header.frame_id = this->frameId_;
    else header.frame_id = "odom";

    nav_msgs::Odometry odom;
    {
      odom.header = header;
      if(this->childFrameId_.size()!=0) odom.child_frame_id = this->childFrameId_;
      else odom.child_frame_id = this->targetName_;

      cnoid::Position pose;
      if(this->link_){
        pose = this->link_->T();
      }else{
        pose = this->sensor_->link()->T() * this->sensor_->T_local();
        // Rotate sensor->localR 180[deg] because OpenHRP3 camera -Z axis equals to ROS camera Z axis
        // http://www.openrtp.jp/openhrp3/jp/create_model.html
        pose.linear() = (pose.linear() * cnoid::AngleAxis(M_PI, cnoid::Vector3::UnitX())).eval();
      }
      odom.pose.pose.position.x = pose.translation()[0];
      odom.pose.pose.position.y = pose.translation()[1];
      odom.pose.pose.position.z = pose.translation()[2];
      cnoid::Quaternion quat = cnoid::Quaternion(pose.linear());
      odom.pose.pose.orientation.x = quat.x();
      odom.pose.pose.orientation.y = quat.y();
      odom.pose.pose.orientation.z = quat.z();
      odom.pose.pose.orientation.w = quat.w();
      for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
          if(i==j) odom.pose.covariance[i*6+j] = this->poseCovariance_;
          else odom.pose.covariance[i*6+j] = 0.0;
        }
      }

      cnoid::Vector6 twist;
      twist.head<3>() = pose.linear().transpose() * (pose.translation() - this->prevPose_.translation()) / this->timeStep_;
      cnoid::AngleAxis angleAxis = cnoid::AngleAxis(this->prevPose_.linear().transpose() * pose.linear());
      twist.tail<3>() = angleAxis.angle()*angleAxis.axis() / this->timeStep_;
      odom.twist.twist.linear.x = twist[0];
      odom.twist.twist.linear.y = twist[1];
      odom.twist.twist.linear.z = twist[2];
      odom.twist.twist.angular.x = twist[3];
      odom.twist.twist.angular.y = twist[4];
      odom.twist.twist.angular.z = twist[5];
      for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
          if(i==j) odom.twist.covariance[i*6+j] = this->twistCovariance_;
          else odom.twist.covariance[i*6+j] = 0.0;
        }
      }

      this->pub_.publish(odom);

      this->prevPose_ = pose;
    }

    return true;
  }

  bool OdometryPublisherItem::store(Archive& archive) {
    archive.write("targetName", this->targetName_);
    archive.write("odometryTopicName", this->odometryTopicName_);
    archive.write("frameId", this->frameId_);
    archive.write("childFrameId", this->childFrameId_);
    archive.write("poseCovariance", this->poseCovariance_);
    archive.write("twistCovariance", this->twistCovariance_);
    archive.write("publishRate", this->publishRate_);
    return true;
  }

  bool OdometryPublisherItem::restore(const Archive& archive) {
    archive.read("targetName", this->targetName_);
    archive.read("odometryTopicName", this->odometryTopicName_);
    archive.read("frameId", this->frameId_);
    archive.read("childFrameId", this->childFrameId_);
    archive.read("poseCovariance", this->poseCovariance_);
    archive.read("twistCovariance", this->twistCovariance_);
    archive.read("publishRate", this->publishRate_);
    return true;
  }

}

