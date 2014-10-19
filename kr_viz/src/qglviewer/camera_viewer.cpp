#include "kr_viz/qglviewer/camera_viewer.hpp"
#include "kr_viz/qglviewer/spiral.hpp"

namespace kr {
namespace viz {

void CameraViewer::draw() { DrawSpiral(); }

void CameraViewer::init() {
}

QString CameraViewer::helpString() const {
  QString text("camera viewer");
  return text;
}

}  // namespace viz
}  // namespace kr
