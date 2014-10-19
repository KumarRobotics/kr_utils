#include <kr_viz/qglviewer/camera.hpp>

namespace kr {
namespace viz {

void Camera::draw(bool drawFarPlane, float scale) const {
  qglviewer::Camera::draw(drawFarPlane,scale);
  return; //  let super handle for now
  
  if (image_dirty_) {
    //  need to update the texture
    texture_.reset( new Texture() );    
    if (!image_.empty()) {
      texture_->setFromCvMat(image_);
    }
    image_dirty_ = false;
  }
  
  /// @todo: do transforms for camera here
  glPushMatrix();
  
  if (texture_ && texture_->valid()) {
    glEnable(texture_->getTarget());
    texture_->bind();
  }
  //  draw textured rectangle
  glBegin(GL_QUADS);
  {
    glColor3d(frustumColor_.x,frustumColor_.y,frustumColor_.z);
    
    glTexCoord2f(0,0);
    glVertex2f(-1,-1);
    
    glTexCoord2f(1,0);
    glVertex2f(1,-1);
    
    glTexCoord2f(1,1);
    glVertex2f(1,1);
    
    glTexCoord2f(0,1);
    glVertex2f(-1,1);
  }
  glEnd();
  
  //  done drawing with texture
  if (texture_) {
    glBindTexture(texture_->getTarget(),0);
    glDisable(texture_->getTarget());
  }
  //  go back to world coordinates
  glPopMatrix();
}

} //  namespace viz
} //  namespace kr
