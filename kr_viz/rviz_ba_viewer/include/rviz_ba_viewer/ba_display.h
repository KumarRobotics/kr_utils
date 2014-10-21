/*
 * ba_display.h
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */

#ifndef RVIZ_BA_DISPLAY_H_
#define RVIZ_BA_DISPLAY_H_

#include <rviz/display.h>

#include <QObject>
#include <map>

#include <rviz_ba_viewer/keyframe_object.hpp>
#include <rviz_ba_viewer/feature_rays_object.hpp>
#include <rviz_ba_viewer/BaGraph.h>   //  generated message
#include <rviz_ba_viewer/KeyFrame.h>
#include <rviz_ba_viewer/BaPoint.h>

namespace rviz {

class Property;
class IntProperty;
class RosTopicProperty;
class FloatProperty;
class BoolProperty;

/**
 * @brief RViz plugin for rendering a bundle-adjustment graph.
 */
class BAGraphDisplay : public Display {
  Q_OBJECT
public:
  
  BAGraphDisplay();
  virtual ~BAGraphDisplay();
  
  /// Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void update(float,float);
  
protected slots:
  void updateTopic();         /// When topic is changed.
  void updateRenderEvery();   /// When 'render every' option is changed.
  void updateScale();         /// When scale option is changed.
  void updateImageEnabled();  /// When images are enabled/disabled
  
protected:
  
  /// Properties for the GUI
  RosTopicProperty * topic_property_;
  IntProperty * render_every_property_;
  FloatProperty * scale_property_;
  BoolProperty * image_enabled_property_;
  
  /// ROS objects
  ros::Subscriber sub_graph_;
  
  /// Graph objects
  /// @todo: as this plugin scales up, this should be probably refactored into a 
  /// proper class
  struct KeyFrame {
    typedef std::shared_ptr<KeyFrame> Ptr;
    std::map<int, rviz_ba_viewer::BaPoint> points;
  };
  std::map<int, KeyFrame::Ptr> keyframes_;
  
  /// Display objects
  std::map<int, KeyFrameObject::Ptr> kf_objects_;
  std::map<int, FeatureRaysObject::Ptr> rays_;
  bool dirty_{false};
  double scale_{1};
  bool image_enabled_{true};
  std::string frame_;
  
  /// Overrides from Display
  virtual void onEnable();
  virtual void onDisable();
  virtual void subscribe();
  virtual void unsubscribe();
  
  /// Apply the fixed-frame transform.
  void applyFixedTransform();
  
  /// Callback initiated by ROS topic.
  void topicCallback(const rviz_ba_viewer::BaGraphConstPtr&msg);
  
  /// Destroy the scene and ogre objects.
  void cleanup();
};

} //  namespace rviz

#endif
