#ifndef KR_VIZ_QGLVIEWER_CAMERA_HPP_
#define KR_VIZ_QGLVIEWER_CAMERA_HPP_

#include <QGLViewer/camera.h>
#include <opencv2/core/core.hpp>

namespace kr {
namespace viz {

class Camera : public qglviewer::Camera {
 public:
  void set_image(const cv::Mat& image) { image_ = image; }

 private:
  bool display_srt_{false};
  cv::Mat image_;
  cv::Matx33d K_;
  double width_{}, height_{};
};

}  // namespace viz
}  // namespace kr

#endif  // KR_VIZ_QGLVIEWER_CAMERA_HPP_
