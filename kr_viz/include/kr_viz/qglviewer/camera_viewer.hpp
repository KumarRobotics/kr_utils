#ifndef KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_
#define KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_

#include <memory>

#include <QGLViewer/qglviewer.h>

#include "kr_viz/qglviewer/camera.hpp"

namespace kr {
namespace viz {

class CameraViewer : public QGLViewer {
 public:
  void addCamera(Camera* camera) { cameras_.push_back(camera); }

 protected:
  virtual void draw();
  virtual void init();
  virtual QString helpString() const;

 private:
  std::vector<Camera*> cameras_;
};

}  // namespace viz
}  // namespace kr

#endif  // KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_
