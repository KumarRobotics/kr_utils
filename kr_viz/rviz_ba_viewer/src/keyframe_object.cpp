/*
 * keyframe_object.cpp
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */

#include <rviz_ba_viewer/keyframe_object.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

KeyFrameObject::KeyFrameObject(Ogre::SceneManager * scene_manager, int id) :
  id_(id), scene_manager_(scene_manager) {
  
  //  create objects and nodes
  scene_node_ = scene_manager_->createSceneNode();
  frustum_object_= scene_manager_->createManualObject();
  plane_object_ = scene_manager_->createManualObject();
}

KeyFrameObject::~KeyFrameObject() {
  //  teardown ogre objects
  scene_node_->detachAllObjects();
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(frustum_object_);
  scene_manager_->destroyManualObject(plane_object_);
}

void KeyFrameObject::setPose(const Ogre::Vector3& position,
                             const Ogre::Quaternion& orientation) {
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void KeyFrameObject::createGeometry() {
  
}
