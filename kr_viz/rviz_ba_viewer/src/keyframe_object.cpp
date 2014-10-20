/*
 * keyframe_object.cpp
 *
 * Copyright (c) 2014 Gareth Cross, Chao Qu. All rights reserved.
 */

#include <rviz_ba_viewer/keyframe_object.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

Ogre::TexturePtr textureFromMat(const cv::Mat& mat, const std::string& name) {
  assert(!mat.empty());
  //  place mat data into a stream for ogre
  Ogre::TexturePtr texture;
  Ogre::DataStreamPtr data_stream;
  data_stream.bind(new Ogre::MemoryDataStream((void *)mat.data,
                                              mat.total() * mat.elemSize()));

  const Ogre::String res_group =
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::TextureManager &texture_manager = Ogre::TextureManager::getSingleton();
  //  swap byte order when going to Ogre
  texture = texture_manager.loadRawData(name, res_group, data_stream,
                                        mat.cols, mat.rows,
                                        Ogre::PF_B8G8R8, Ogre::TEX_TYPE_2D, 0);
  std::cout << "Created texture with dims: " << mat.cols << "," << mat.rows << "\n";
  return texture;
}

KeyFrameObject::KeyFrameObject(Ogre::SceneManager * scene_manager, int id) :
  id_(id), scene_manager_(scene_manager) {
  
  //  create objects and nodes
  scene_node_ = scene_manager_->createSceneNode();
  scene_node_->setVisible(true);
  frustum_object_= scene_manager_->createManualObject();
  plane_object_ = scene_manager_->createManualObject();
  scene_node_->attachObject(frustum_object_);
  scene_node_->attachObject(plane_object_);
    
  //  create a material
  const Ogre::String& group = 
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::MaterialManager& man = Ogre::MaterialManager::getSingleton();
  //  some boring settings...
  material_ = man.create("kf_mat_" + std::to_string(id), group);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(true);
  material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  //material_->setSceneBlending(Ogre::SBT_REPLACE);
  //material_->setDepthBias(-16.0f, 0.0f); 
  
  frustum_material_ = man.create("kf_mat_frust_" + std::to_string(id), group);
  frustum_material_->setReceiveShadows(false);
  frustum_material_->getTechnique(0)->setLightingEnabled(false);
  frustum_material_->setCullingMode(Ogre::CULL_NONE);
  frustum_material_->setDepthWriteEnabled(true);
  frustum_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
}

KeyFrameObject::~KeyFrameObject() {
  //  teardown ogre objects
  scene_node_->detachAllObjects();
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(frustum_object_);
  scene_manager_->destroyManualObject(plane_object_);
}

void KeyFrameObject::setImage(const cv::Mat& image) {
  image_ = image;
  //  also re-create the ogre texture
  texture_ = textureFromMat(image, "kf_tex_" + std::to_string(id_));
  std::cout << "Created texture " << texture_->getName() << "\n";
  std::cout << "Dimensions: " << texture_->getWidth() << ","
            << texture_->getHeight() << std::endl;
  dirty_ = true;
}

void KeyFrameObject::setCameraModel(
    const image_geometry::PinholeCameraModel& model,
    int width, int height) {
  cam_model_ = model;
  width_ = width;
  height_ = height;
  dirty_ = true;
}

void KeyFrameObject::setPose(const Ogre::Vector3& position,
                             const Ogre::Quaternion& orientation) {
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void KeyFrameObject::setColor(const Ogre::Vector4& color) { 
  if (color_ != color) {
    color_ = color;
    dirty_ = true;
  }
}

void KeyFrameObject::setImageEnabled(bool imageEnabled) {
  if (imageEnabled != imageEnabled_) {
    imageEnabled_ = imageEnabled;
    dirty_ = true;
  }
}

void KeyFrameObject::createGeometry() {
  if (!dirty_) {
    return; //  no need to re-create geometry
  }
  dirty_ = false;
  
  const double fx = cam_model_.fx(), fy = cam_model_.fy();
  const double cx = cam_model_.cx(), cy = cam_model_.cy();
  //  generate coordinates of the frustum
  Ogre::Vector3 points[] = {{0,0,1},            //  top left
                            {0,height_,1},      //  bottom left
                            {width_,height_,1}, //  bottom right
                            {width_,0,1}};      //  top right
  for (int i=0; i < 4; i++) {
    points[i].x = (points[i].x - cx) / fx;
    points[i].y = (points[i].y - cy) / fy;
  }
  const Ogre::Vector3 centre(0,0,0);  //  optical centre
  
  frustum_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
  frustum_object_->begin(frustum_material_->getName(),
                         Ogre::RenderOperation::OT_LINE_LIST);
  {
    frustum_object_->colour(color_.x,color_.y,color_.z,color_.w);
    //  left side
    frustum_object_->position(points[0]);
    frustum_object_->position(centre);
    frustum_object_->position(centre);
    frustum_object_->position(points[1]);
    //  right side
    frustum_object_->position(points[2]);
    frustum_object_->position(centre);
    frustum_object_->position(centre);
    frustum_object_->position(points[3]);
    //  front rectangle
    for (int i=0; i < 4; i++) {
      frustum_object_->position(points[i]);
      frustum_object_->position(points[(i+1) % 4]);
    }
  }
  frustum_object_->end();
  
  if (imageEnabled_ && texture_->isLoaded()) {
    //  now the textured quad w/ our image
    //  first set up the material for textured drawing
    Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState * tex_unit;
    if (pass->getNumTextureUnitStates() > 0) {
      tex_unit = pass->getTextureUnitState(0);
    } else {
      tex_unit = pass->createTextureUnitState();
    }
    tex_unit->setTextureName(texture_->getName());
    tex_unit->setTextureFiltering(Ogre::TFO_BILINEAR);
    tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                                Ogre::LBS_CURRENT, color_.w); //  use alpha
    
    plane_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
    plane_object_->begin(material_->getName(),
                         Ogre::RenderOperation::OT_TRIANGLE_LIST);
    {
      //plane_object_->colour(1.0,0,0,1);
      plane_object_->position(points[0]); //  top left
      plane_object_->textureCoord(0,0);
      
      plane_object_->position(points[1]); //  bottom left
      plane_object_->textureCoord(0,1);
      
      plane_object_->position(points[2]); //  bottom right
      plane_object_->textureCoord(1,1);
      
      plane_object_->position(points[2]); //  bottom right
      plane_object_->textureCoord(1,1);
      
      plane_object_->position(points[3]); //  top right
      plane_object_->textureCoord(1,0);
      
      plane_object_->position(points[0]); //  top left
      plane_object_->textureCoord(0,0);
    }
    plane_object_->end();
  } else {
    std::cout << "Shit!\n";
    plane_object_->clear(); //  lazy: just clear it for now
  }
}
