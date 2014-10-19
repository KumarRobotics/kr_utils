/*
 * ba_display.h
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */

#ifndef RVIZ_BA_DISPLAY_H_
#define RVIZ_BA_DISPLAY_H_

#include <QObject>

#include <ros/ros.h>
#include <rviz/display.h>

#include <rviz_ba_viewer/keyframe_object.hpp>
#include <rviz_ba_viewer/BaGraph.h>   //  generated message

namespace rviz {

class Property;
class IntProperty;
class RosTopicProperty;

/**
 * @brief RViz plugin for rendering a bundle-adjustment graph.
 */
class BADisplay : public Display {
  Q_OBJECT
public:
  
  BADisplay();
  virtual ~BADisplay();
  
  /// Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void update(float,float);
  
protected slots:
  void updateTopic();
  
protected:
  
  /// Properties for the GUI
  RosTopicProperty * topic_property_;
  
  /// ROS objects
  ros::Subscriber sub_graph_;
  
  /// Overrides from Display
  virtual void onEnable();
  virtual void onDisable();
  virtual void subscribe();
  virtual void unsubscribe();
  
  /// Callback initiated by ROS topic.
  void topicCallback(const rviz_ba_viewer::BaGraphConstPtr&msg);
  
  /// Destroy the scene and ogre objects.
  void cleanup();
};

} //  namespace rviz

#endif
