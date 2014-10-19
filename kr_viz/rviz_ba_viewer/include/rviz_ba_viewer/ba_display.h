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

namespace rviz {

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
  
protected:
  
  /// Overrides from Display
  virtual void onEnable();
  virtual void onDisable();
  virtual void subscribe();
  virtual void unsubscribe();
};

} //  namespace rviz

#endif
