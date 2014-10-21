/*
 * ba_display.cpp
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */

#include <rviz_ba_viewer/ba_display.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/display_context.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <QtGlobal>
#include <QString>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace rviz {

BAGraphDisplay::BAGraphDisplay() : Display() {
  //  type of message our topic accepts
  const std::string msg_name = 
      ros::message_traits::datatype<rviz_ba_viewer::BaGraph>();
  topic_property_ = 
      new RosTopicProperty("Topic","",QString::fromStdString(msg_name),
                           "rviz_ba_viewer::BaGraph topic to subscribe to.",
                           this,SLOT(updateTopic()));
  render_every_property_ = 
      new IntProperty("Render Every",1,"Render every n'th keyframe",this,
                      SLOT(updateRenderEvery()));
  scale_property_ = 
      new FloatProperty("Frustum Scale", 1, "Depth to image plane (1 = default)",
                        this,SLOT(updateScale()));
  
  image_enabled_property_ = 
      new BoolProperty("Display images",true,"Display images in keyframes",
                       this,SLOT(updateImageEnabled()));
}

BAGraphDisplay::~BAGraphDisplay() {
  unsubscribe();
  cleanup();
}

void BAGraphDisplay::onInitialize() {
  ROS_INFO("BAGraphDisplay onInitialize");
}

void BAGraphDisplay::fixedFrameChanged() {
  //  apply a new transform to the scene node
  applyFixedTransform();
  dirty_ = true;
}

void BAGraphDisplay::reset() {
  unsubscribe();
  cleanup();
  subscribe();
}

void BAGraphDisplay::update(float,float) {

  //  we will only render every n'th keyframe
  int count=0;
  const int render_every = render_every_property_->getValue().toInt();
  
  for (std::pair<const int,KeyFrameObject::Ptr>& pair : kf_objects_) {
    if (count++ % render_every) {
      pair.second->sceneNode()->setVisible(false);
    } else {
      pair.second->createGeometry();
      pair.second->setImageEnabled(image_enabled_);
      pair.second->sceneNode()->setScale(Ogre::Vector3(scale_,scale_,scale_));
      pair.second->sceneNode()->setVisible(true);
    }
  }
  
  //  transform everything to correct frame
  applyFixedTransform();
  
  //  trigger a re-draw
  context_->queueRender();
}

void BAGraphDisplay::onEnable() {
  subscribe();
}

void BAGraphDisplay::onDisable() {
  unsubscribe();
  cleanup();
}

void BAGraphDisplay::subscribe() {
  if (!isEnabled()) {
    return;
  }
  
  const std::string topic = topic_property_->getTopic().toStdString();
  if (!topic.empty()) {
    try {
      //  try subscribing to the provided topic
      ROS_INFO("Subscribing to %s", topic.c_str());
      sub_graph_ = update_nh_.subscribe(topic,1,&BAGraphDisplay::topicCallback,this);
      setStatus(StatusProperty::Ok, "Topic", "Ok");
    }
    catch (ros::Exception &e) {
      setStatus(StatusProperty::Error,"Topic",
                QString("Error subscribing to: ") + e.what());
    }
  }
}

void BAGraphDisplay::unsubscribe() {
  sub_graph_.shutdown();
}

void BAGraphDisplay::updateTopic() {
  unsubscribe();
  cleanup();
  subscribe();
}

void BAGraphDisplay::updateScale() {
  scale_ = scale_property_->getValue().toDouble();
  if (scale_ < 0.01) {
    scale_ = 0.01;
    scale_property_->setValue(QVariant(scale_));  //  apply a minimum
  }
  dirty_ = true;
  ROS_INFO("updateScale");
}

void BAGraphDisplay::updateImageEnabled() {
  image_enabled_ = image_enabled_property_->getValue().toBool();
  dirty_ = true;
  ROS_INFO("updateImageEnabled");
}

void BAGraphDisplay::updateRenderEvery() {
  int value = render_every_property_->getValue().toInt();
  if (value < 1) {
    render_every_property_->setValue(QVariant(static_cast<int>(1)));
  }
  dirty_ = true;
  ROS_INFO("updateRenderEvery");
}

void BAGraphDisplay::cleanup() {
  setStatus(StatusProperty::Warn, "Message", "No message received");
  // destroy all graphical objects here
  kf_objects_.clear();
}

void BAGraphDisplay::applyFixedTransform() {
  if (frame_.empty()) {
    //  no frame provided in message, assume world
    frame_ = "world";
  }
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  
  //  all geometry is assumed to be in 'frame_', which is identity wrt. itself
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  //  this will calculate the world transform
  if (!context_->getFrameManager()->transform(frame_, ros::Time(), pose,
                                              position, orientation)) {
    ROS_DEBUG("Error transforming map '%s' from frame '%s' to frame '%s'",
              qPrintable(getName()), frame_.c_str(), qPrintable(fixed_frame_));
    setStatus(StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame_) +
                  "] to [" + fixed_frame_ + "]");
  } else {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void BAGraphDisplay::topicCallback(const rviz_ba_viewer::BaGraphConstPtr& msg) {
  //  update frame
  frame_ = msg->header.frame_id;
  setStatus(StatusProperty::Ok, "Message", 
            QString("Ok"));
  //  update all the key-frames
  for (const rviz_ba_viewer::KeyFrame& kf : msg->keyframes) {
    std::map<int,KeyFrameObject::Ptr>::iterator ite = kf_objects_.find(kf.id);
    KeyFrameObject::Ptr kfo;
    if (ite != kf_objects_.end()) {
      //  old key-frame to update
      kfo = ite->second;
    } else {
      //  new key-frame to insert
      kfo.reset(new KeyFrameObject(context_->getSceneManager(), kf.id));
      scene_node_->addChild(kfo->sceneNode());
      kf_objects_[kf.id] = kfo;
    }
    
    //  update position/orientation
    //  in this case, dirty_ is not set and the geometry is not rebuilt
    const geometry_msgs::Point& p = kf.pose.pose.position;
    const geometry_msgs::Quaternion& q = kf.pose.pose.orientation;
    kfo->setPose(Ogre::Vector3(p.x,p.y,p.z),
                 Ogre::Quaternion(q.w,q.x,q.y,q.z));
    
    //  image data is included, convert to cv::Mat
    if (!kf.image.data.empty()) {
      cv::Mat rgbMat;
      cv::Mat scaled;
      try {
        auto cv = cv_bridge::toCvCopy(kf.image, "rgb8");
        rgbMat = cv->image;
        //  shrink down by factor of 4
        scaled = cv::Mat(rgbMat.rows/4,rgbMat.cols/4,rgbMat.type());
        cv::resize(rgbMat,scaled,scaled.size());
      } catch (std::exception& e) {
        setStatus(StatusProperty::Warn, "Message", 
                  QString("Failed to extract image in message") + 
                  e.what());
      }
      
      kfo->setImage(scaled);
      
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(kf.cinfo);
      kfo->setCameraModel(model,rgbMat.cols,rgbMat.rows);
    }
  }
  
  //  update any points we have been given
  //  these don't need to be in any particular order
//  for (const rviz_ba_viewer::BaPoint& point : msg->points) {
//    //  find the corresponding keyframe
//    std::map<int,KeyFrame::Ptr>::iterator ite = 
//        keyframes_.find(point.keyframe_id);
//    if (ite == keyframes_.end()) {
//      //  this is invalid, issue a warning
//      setStatus(StatusProperty::Warn, "Message", 
//                QString("Graph is incomplete. Some points refer to keyframes") + 
//                QString(" which do not exist!"));
//    } else {
      
      
//    }
//  }
  
  dirty_ = true;  //  need to redraw
}

} //  namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::BAGraphDisplay, rviz::Display)
