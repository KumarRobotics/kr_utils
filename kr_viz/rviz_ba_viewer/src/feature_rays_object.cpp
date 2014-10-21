/*
 * feature_rays_object.cpp
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */

#include <rviz_ba_viewer/feature_rays_object.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

FeatureRaysObject::FeatureRaysObject(Ogre::SceneManager * manager, int id) :
 id_(id), scene_manager_(manager) {

  scene_node_ = scene_manager_->createSceneNode();
  scene_node_->setVisible(true);
  lines_object_ = scene_manager_->createManualObject();
  scene_node_->attachObject(lines_object_);
    
  //  create a material
  const Ogre::String& group = 
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::MaterialManager& man = Ogre::MaterialManager::getSingleton();
  //  some boring settings...
  lines_material_ = man.create("kf_lines_" + std::to_string(id), group);
  lines_material_->setReceiveShadows(false);
  lines_material_->getTechnique(0)->setLightingEnabled(false);
  lines_material_->setCullingMode(Ogre::CULL_NONE);
  lines_material_->setDepthWriteEnabled(true);
  lines_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);  
}

FeatureRaysObject::~FeatureRaysObject() {
  scene_node_->detachAllObjects();
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(lines_object_);
}

void FeatureRaysObject::setOrigin(const Ogre::Vector3& origin) {
  origin_ = origin;
}

void FeatureRaysObject::setLines(const std::vector<Ogre::Vector3>& centres,
                                 const std::vector<double>& lengths) {
  centres_ = centres;
  lengths_ = lengths;
}

void FeatureRaysObject::createGeometry() {
  /// @todo: add dirty flag to check
  
  
  
}
