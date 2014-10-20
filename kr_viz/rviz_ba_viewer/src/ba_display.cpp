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

void BAGraphDisplay::onInitialize() {}

void BAGraphDisplay::fixedFrameChanged() {
  
}

void BAGraphDisplay::reset() {
  
}

void BAGraphDisplay::update(float,float) {
  
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
}

void BAGraphDisplay::topicCallback(const rviz_ba_viewer::BaGraphConstPtr& msg) {
  
  
  
}

} //  namespace rviz
