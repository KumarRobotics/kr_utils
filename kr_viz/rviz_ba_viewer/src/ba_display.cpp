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

BADisplay::BADisplay() : Display() {
  //  type of message our topic accepts
  const std::string msg_name = 
      ros::message_traits::datatype<rviz_ba_viewer::BaGraph>();
  topic_property_ = 
      new RosTopicProperty("Topic","",QString::fromStdString(msg_name),
                           "rviz_ba_viewer::BaGraph topic to subscribe to.",
                           this,SLOT(updateTopic()));

  
  
}

BADisplay::~BADisplay() {
  unsubscribe();
  cleanup();
}

void BADisplay::onInitialize() {}

void BADisplay::fixedFrameChanged() {
  
}

void BADisplay::reset() {
  
}

void BADisplay::update(float,float) {
  
}

void BADisplay::onEnable() {
  subscribe();
}

void BADisplay::onDisable() {
  unsubscribe();
  cleanup();
}

void BADisplay::subscribe() {
  if (!isEnabled()) {
    return;
  }
  
  const std::string topic = topic_property_->getTopic().toStdString();
  if (!topic.empty()) {
    try {
      //  try subscribing to the provided topic
      ROS_INFO("Subscribing to %s", topic.c_str());
      sub_graph_ = update_nh_.subscribe(topic,1,&BADisplay::topicCallback,this);
      setStatus(StatusProperty::Ok, "Topic", "Ok");
    }
    catch (ros::Exception &e) {
      setStatus(StatusProperty::Error,"Topic",
                QString("Error subscribing to: ") + e.what());
    }
  }
}

void BADisplay::unsubscribe() {
  sub_graph_.shutdown();
}

void BADisplay::updateTopic() {
  unsubscribe();
  cleanup();
  subscribe();
}

void BADisplay::cleanup() {
  setStatus(StatusProperty::Warn, "Message", "No message received");
  
  // destroy all graphical objects here
}

void BADisplay::topicCallback(const rviz_ba_viewer::BaGraphConstPtr& msg) {
  
  
  
}

} //  namespace rviz
