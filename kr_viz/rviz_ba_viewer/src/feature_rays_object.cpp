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

void FeatureRaysObject::setCentres(const std::vector<Ogre::Vector3>& centres) {
  centres_ = centres;
}

void FeatureRaysObject::setLengths(const std::vector<double>& lengths) {
  lengths_ = lengths;
}

void FeatureRaysObject::setDrawsLengths(bool draws_lengths) {
  draws_lengths_ = draws_lengths;
}

void FeatureRaysObject::createGeometry() {
  /// @todo: add dirty flag to check
  
  //  only use lengths when rendering if we have the right number
  const bool draw_lengths = draws_lengths_ && 
      (centres_.size() == lengths_.size());
  
  lines_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
  lines_object_->begin(lines_material_->getName(),
                       Ogre::RenderOperation::OT_LINE_LIST);
  {
    const size_t N = centres_.size();
    for (size_t i=0; i < N; i++) {
      const Ogre::Vector3& c = centres_[i];
      Ogre::Vector3 minPt, maxPt;
      
      if (draw_lengths) {
        //  use length to find min/max point
        const double& len = lengths_[i];
        Ogre::Vector3 n = c - origin_;
        n.normalise();
        minPt = c - n*len;
        maxPt = c + n*len;
      } else {
        //  simply draw from origin to the point
        minPt = origin_;
        maxPt = c;
      }
      
      //  add a line segment
      lines_object_->position(minPt);
      lines_object_->position(maxPt);
      lines_object_->colour(color_.x,color_.y,color_.z,color_.w);
    }
  }
  lines_object_->end();
}
