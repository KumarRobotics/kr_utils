#ifndef KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_
#define KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_

#include <memory>

#include <QGLViewer/qglviewer.h>

#include "kr_viz/qglviewer/camera.hpp"

namespace kr {
namespace viz {

class CameraViewer : public QGLViewer {
 public:
  void set_camera(const Camera* camera) { camera_ = camera; }

 protected:
  virtual void draw();
  virtual void init();
  virtual QString helpString() const;

 private:
  const Camera* camera_{nullptr};
};

}  // namespace viz
}  // namespace kr

#endif  // KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_
