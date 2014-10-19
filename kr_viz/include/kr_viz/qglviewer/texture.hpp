#ifndef TEXTURE_HPP
#define TEXTURE_HPP

#include <opencv2/opencv.hpp>
#include <GL/gl.h>
#include <GL/glu.h>
#include <memory>

namespace kr {
namespace viz {

class Texture {
public:
  typedef std::shared_ptr<Texture> Ptr;
  
  Texture() = default;
  virtual ~Texture();
  
  /**
   * @brief setFromCvMat
   * @param mat
   */
  void setFromCvMat(const cv::Mat& mat);
  
  /**
   * @brief valid
   * @return 
   */
  bool valid() const; 
  
  /**
   * @brief bind
   */
  void bind() const;
  
  /**
   * @brief getTarget
   * @return 
   */
  GLenum getTarget() const { return target_; }
  
  //  Non-copyable
  Texture(const Texture&) = delete;
  Texture& operator = (const Texture&) = delete;
  
private:
  void destroyResources();
  
  GLuint texture_resc_{0};
  GLenum target_;
};

}
}

#endif // TEXTURE_HPP
