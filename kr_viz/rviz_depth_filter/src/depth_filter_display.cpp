/*
 * depth_filter_display.cpp
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */

#include <QtGlobal>

#include <rviz_depth_filter/depth_filter_display.h>

#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/frame_manager.h>
#include <rviz/display_context.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <geometry_msgs/Pose.h>

namespace rviz {

DepthFilterDisplay::DepthFilterDisplay() : Display() {
  static int cur_id=0;
  id_ = cur_id++;
  
  const std::string msg_name = 
      ros::message_traits::datatype<rviz_depth_filter::DepthFilter>();
  topic_property_ = 
      new RosTopicProperty("Topic","",QString::fromStdString(msg_name),
                           "rviz_ba_viewer::DepthFilter topic to subscribe to.",
                           this,SLOT(updateTopic()));
  line_width_property_ = 
      new FloatProperty("Line Width", 0.025, "Width of lines", this,
                        SLOT(updateLineWidth()));
  color_property_ = 
      new ColorProperty("Color", QColor(247,0,248,255), "Line color", this,
                        SLOT(updateLineColor()));
}

DepthFilterDisplay::~DepthFilterDisplay() {
  unsubscribe();
  cleanup();
}

void DepthFilterDisplay::onInitialize() {}

void DepthFilterDisplay::fixedFrameChanged() {
  applyFixedTransform();
}

void DepthFilterDisplay::reset() {
  unsubscribe();
  cleanup();
  subscribe();
}

void DepthFilterDisplay::update(float, float) {
  //  transform everything to correct frame
  applyFixedTransform();
  //  trigger a re-draw
  context_->queueRender();
}

void DepthFilterDisplay::createGeometry() {
  
  lines_.clear(); //  get rid of old geometry
  
  /// @todo: rendering the cloud with individual rviz lines is not very efficient
  /// if scalability is desired, this will need to be updated later
  for (size_t i=0; i < cloud_.positions.size(); i++) {
    std::shared_ptr<rviz::BillboardLine> line;
    line.reset( new rviz::BillboardLine(scene_manager_,scene_node_) );
    line->setColor(color_.x,color_.y,color_.z,color_.w);
    line->setLineWidth(line_width_);
    
    //  calculate start and end points
    const geometry_msgs::Point& origin = cloud_.origins[i];
    const geometry_msgs::Point& center = cloud_.positions[i];
    const Ogre::Vector3 o(origin.x,origin.y,origin.z);
    const Ogre::Vector3 c(center.x,center.y,center.z);
    const double length = cloud_.sigmas[i]*2; //  do two standard deviations
          
    Ogre::Vector3 minPt, maxPt;
    //  use length to find min/max point
    Ogre::Vector3 n = c - o;
    n.normalise();
    minPt = c - n*length;
    maxPt = c + n*length;
    line->addPoint(minPt);
    line->addPoint(maxPt);
    lines_.push_back(line);
  }
}

void DepthFilterDisplay::updateTopic() {
  unsubscribe();
  cleanup();
  subscribe();
}

void DepthFilterDisplay::updateLineWidth() {
  double value = line_width_property_->getValue().toDouble();
  if (value < 0.001) {
    value = 0.001;
    line_width_property_->setValue(QVariant(value));
  }
  line_width_ = value;
}

void DepthFilterDisplay::updateLineColor() {
  const QColor color = color_property_->getColor();
  color_.x = color.red() / 255.0;
  color_.y = color.green() / 255.0;
  color_.z = color.blue() / 255.0;
  color_.w = color.alpha() / 255.0;
}

void DepthFilterDisplay::onEnable() {
  subscribe();
}

void DepthFilterDisplay::onDisable() {
  unsubscribe();
  cleanup();
}

void DepthFilterDisplay::subscribe() {
  if (!isEnabled()) {
    return;    
  }
  const std::string topic = topic_property_->getTopic().toStdString();
  if (!topic.empty()) {
    try {
      //  try subscribing to the provided topic
      ROS_INFO("Subscribing to %s", topic.c_str());
      sub_cloud_ = update_nh_.subscribe(topic,1,
                                        &DepthFilterDisplay::topicCallback,this);
      setStatus(StatusProperty::Ok, "Topic", "Ok");
    }
    catch (ros::Exception &e) {
      setStatus(StatusProperty::Error,"Topic",
                QString("Error subscribing to: ") + e.what());
    }
  }
}

void DepthFilterDisplay::unsubscribe() {
  sub_cloud_.shutdown();
}

void DepthFilterDisplay::applyFixedTransform() {
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

void DepthFilterDisplay::topicCallback(
    const rviz_depth_filter::DepthFilterConstPtr &msg) {
  frame_ = msg->header.frame_id;
  setStatus(StatusProperty::Ok, "Message", 
            QString("Ok"));
  
  const size_t N = msg->origins.size();
  if (N != msg->positions.size() || N != msg->sigmas.size()) {
    setStatus(StatusProperty::Error, "Message", 
              QString("All message arrays must be the same dimension!"));
  } else {
    cloud_ = *msg; 
    has_data_ = true;
    //  re-build
    createGeometry();
  }
}

void DepthFilterDisplay::cleanup() {
//  if (initialized_) {
//    //scene_node_->detachObject(point_object_);
//    //point_material_.setNull();
//    initialized_ = false;
//  }
  
  lines_.clear();
}

} //  namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::DepthFilterDisplay, rviz::Display)
