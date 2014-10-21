/*
 * depth_filter_display.h
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */

#ifndef DEPTH_CLOUD_DISPLAY_H
#define DEPTH_CLOUD_DISPLAY_H

#include <rviz/display.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <QObject>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>

#include <rviz_depth_filter/DepthFilter.h>

namespace rviz {

class Property;
class RosTopicProperty;

class DepthFilterDisplay : public Display {
  Q_OBJECT
public:
  DepthFilterDisplay();
  ~DepthFilterDisplay();
  
  /// Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void update(float,float);
  
  /// Rebuild OGRE objects as required.
  void createGeometry();
  
protected slots:
  void updateTopic();
  
private:
  
  /// Properties for the GUI
  RosTopicProperty * topic_property_;
  
  /// ROS Objects
  ros::Subscriber sub_cloud_;
  std::string frame_;
  rviz_depth_filter::DepthFilter cloud_;
  bool has_data_{false};
  bool initialized_{false};
  
  /// OGRE objects
  std::vector<std::shared_ptr<rviz::BillboardLine>> lines_; 
  //Ogre::ManualObject * point_object_;
  //Ogre::MaterialPtr point_material_;
  Ogre::Vector4 color_{0.207,0.247,1,1};
  int id_;
  
  /// Overrides from Display
  virtual void onEnable();
  virtual void onDisable();
  virtual void subscribe();
  virtual void unsubscribe();
  
  /// Apply the fixed-frame transform.
  void applyFixedTransform();
  
  /// Callback initiated by ROS topic.
  void topicCallback(const rviz_depth_filter::DepthFilterConstPtr &msg);
  
  /// Destroy scene and ogre objects.
  void cleanup();
};

} //  namespace rviz

#endif // DEPTH_CLOUD_DISPLAY_H
