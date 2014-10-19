/*
 * keyframe_object.hpp
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */
#ifndef KEYFRAME_OBJECT_HPP
#define KEYFRAME_OBJECT_HPP

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace Ogre {
class SceneManager;
class SceneNode;
class ManualObject;
}

class KeyFrameObject {
public:
  
  KeyFrameObject(Ogre::SceneManager * scene_manager, int id);
  virtual ~KeyFrameObject();
    
  const int& id() const { return id_; }
  
  void setPose(const Ogre::Vector3& position,
               const Ogre::Quaternion& orientation);
  
  void createGeometry();
  
private:
  
  const int id_;
  
  /// Ogre objects
  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode    * scene_node_;
  Ogre::ManualObject * frustum_object_;
  Ogre::ManualObject * plane_object_;
  Ogre::TexturePtr     texture_;
  Ogre::MaterialPtr    material_;
};

#endif // KEYFRAME_OBJECT_HPP
