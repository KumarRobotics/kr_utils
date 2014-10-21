/*
 * feature_rays_object.hpp
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */
#ifndef FEATURE_RAYS_OBJECT_HPP
#define FEATURE_RAYS_OBJECT_HPP

#include <OGRE/OgreMaterial.h>

namespace Ogre {
class SceneManager;
class SceneNode;
class ManualObject;
}

class FeatureRaysObject {
public:
  FeatureRaysObject(Ogre::SceneManager * manager, int id);
  virtual ~FeatureRaysObject();
  
  void setOrigin(const Ogre::Vector3& origin);
  
  void setFeatures(const std::vector<Ogre::Vector3>& centres,
                   const std::vector<double>& lengths);
  
  void createGeometry();
  
private:
  int id_;
  Ogre::Vector3 origin_{0,0,0};
  std::vector<Ogre::Vector3> centres_;
  std::vector<double> lengths_;
  
  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode    * scene_node_;
  Ogre::ManualObject * lines_object_;
  Ogre::MaterialPtr    lines_material_;
};

#endif // FEATURE_RAYS_OBJECT_HPP
