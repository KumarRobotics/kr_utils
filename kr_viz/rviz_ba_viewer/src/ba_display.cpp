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
#include <rviz/display_context.h>

#include <OGRE/OgreSceneNode.h>

#include <QtGlobal>
#include <QString>

namespace rviz {

BAGraphDisplay::BAGraphDisplay() : Display() {
  //  type of message our topic accepts
  const std::string msg_name = 
      ros::message_traits::datatype<rviz_ba_viewer::BaGraph>();
  topic_property_ = 
      new RosTopicProperty("Topic","",QString::fromStdString(msg_name),
                           "rviz_ba_viewer::BaGraph topic to subscribe to.",
                           this,SLOT(updateTopic()));
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
  if (!dirty_) {
    //  nothing new to draw
    return;
  }
  dirty_ = false;
  ROS_INFO("BAGraphDisplay update");
  
  for (std::pair<const int,KeyFrameObject::Ptr>& pair : keyframes_) {
    //  re-create geometry, textures are not loaded again by this action
    pair.second->createGeometry();
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

void BAGraphDisplay::cleanup() {
  setStatus(StatusProperty::Warn, "Message", "No message received");
  // destroy all graphical objects here
  keyframes_.clear();
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
  
  
  dirty_ = true;  /// @todo: enable conditionally?
}

} //  namespace rviz
