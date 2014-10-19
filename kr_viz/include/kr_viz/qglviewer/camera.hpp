#ifndef KR_VIZ_QGLVIEWER_CAMERA_HPP_
#define KR_VIZ_QGLVIEWER_CAMERA_HPP_

#include <QGLViewer/camera.h>
#include <opencv2/core/core.hpp>
#include <kr_viz/qglviewer/texture.hpp>

namespace kr {
namespace viz {

class Camera : public qglviewer::Camera {
 public:
  void set_image(const cv::Mat& image) { 
    image_ = image;
    image_dirty_= true;
  }

  /**
   * @brief draw
   * @param drawFarPlane Unused in this implementation.
   * @param scale Distance to the image plane, recommended = 1.
   */
  virtual void draw(bool drawFarPlane=true, float scale=1.0) const;
  
  void setFrustumColor(const qglviewer::Vec& c) { frustumColor_ = c; }
  
 private:
  bool display_srt_{false};
  qglviewer::Vec frustumColor_{1,1,1};
  
  cv::Mat image_;
  mutable bool image_dirty_{false};
  mutable Texture::Ptr texture_;
  
  cv::Matx33d K_;
  double width_{}, height_{};
};

}  // namespace viz
}  // namespace kr

#endif  // KR_VIZ_QGLVIEWER_CAMERA_HPP_
