/*
 * feature_rays_object.hpp
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */
#ifndef FEATURE_RAYS_OBJECT_HPP
#define FEATURE_RAYS_OBJECT_HPP

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreVector4.h>

//
//  Note: presently unused
//

namespace Ogre {
class SceneManager;
class SceneNode;
class ManualObject;
}

class FeatureRaysObject {
public:
  typedef std::shared_ptr<FeatureRaysObject> Ptr;
  
  FeatureRaysObject(Ogre::SceneManager * manager, int id);
  virtual ~FeatureRaysObject();
  
  Ogre::SceneNode* sceneNode() const { return scene_node_; }
  
  void setOrigin(const Ogre::Vector3& origin);
  
  void setCentres(const std::vector<Ogre::Vector3>& centres);
  
  void setLengths(const std::vector<double>& lengths);
  
  void setDrawsLengths(bool draws_lengths);
  
  void createGeometry();
  
private:
  int id_;
  Ogre::Vector3 origin_{0,0,0};
  std::vector<Ogre::Vector3> centres_;
  std::vector<double> lengths_;
  bool draws_lengths_{true};
  Ogre::Vector4 color_{0.207,0.247,1,1};  /// @todo: add option for this
  
  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode    * scene_node_;
  Ogre::ManualObject * lines_object_;
  Ogre::MaterialPtr    lines_material_;
};

#endif // FEATURE_RAYS_OBJECT_HPP
