#ifndef KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_
#define KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_

#include <memory>

#include <QGLViewer/qglviewer.h>

namespace kr {
namespace viz {

class CameraViewer : public QGLViewer {
 protected:
  virtual void draw();
  virtual void init();
  virtual QString helpString() const;

 private:
  std::unique_ptr<qglviewer::Camera> camera_;
};

}  // namespace viz
}  // namespace kr

#endif  // KR_VIZ_QGLVIEWER_CAMERA_VIEWER_HPP_
