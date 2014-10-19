#include <kr_viz/qglviewer/texture.hpp>
#include <stdexcept>

namespace kr {
namespace viz {

bool isPowerOf2(int x) {
  return (x & (x - 1)) == 0;
}

Texture::~Texture() {
  destroyResources();  
}

void Texture::setFromCvMat(const cv::Mat& mat) {
  //  clear old texture
  destroyResources();
  //  check if we need non power of 2 support
  target_ = GL_TEXTURE_2D;
  if (!isPowerOf2(mat.rows) || !isPowerOf2(mat.cols)) {
    const char * tex_rect = "GL_ARB_texture_rectangle";
    if (!gluCheckExtension(reinterpret_cast<const GLubyte*>(&tex_rect[0]), 
                           glGetString(GL_EXTENSIONS))) {
      throw std::runtime_error("Non-pow2 textures are not supported");
    }
    target_ = GL_TEXTURE_RECTANGLE_ARB;
  }
  //  for now only support RGB, resolve this later
  const int type = mat.type();
  if (type != CV_8UC3) {
    throw std::runtime_error("CvMat must have RGB24 format");
  }
  
  glGenTextures(1,&texture_resc_);
  glBindTexture(target_,texture_resc_);
  
  if (!glIsTexture(texture_resc_)) {
    throw std::runtime_error("Failed to create a new texture");
  }
  //  use linear texture filtering
  glTexParameteri(target_, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(target_, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  //  pixels are densely packed
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  
  //  send data to the GPU
  glTexImage2D(target_,0,GL_RGBA,mat.cols,mat.rows,0,
               GL_RGB,GL_UNSIGNED_BYTE,mat.data);
  
  //  deselect the texture when finished
  glBindTexture(target_, 0);
}

bool Texture::valid() const {
  return texture_resc_ != 0;
}

void Texture::bind() const {
  if (!texture_resc_) {
    throw std::runtime_error("Attempted to bind an invalid texture");
  }
  glBindTexture(target_,texture_resc_);
}

void Texture::destroyResources() {
  if (texture_resc_) {
    glDeleteTextures(1,&texture_resc_);
    texture_resc_=0;
  }
}

} //  namespace viz
} //  namespace kr
