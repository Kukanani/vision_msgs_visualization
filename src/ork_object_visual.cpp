/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>

#include <OGRE/OgreCommon.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreVector3.h>

#include <rviz/display_context.h>
#include <rviz/display_factory.h>
#include <rviz/default_plugin/marker_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/movable_text.h>

#include "ork_object_visual.h"

namespace vision_msgs_visualization
{
  OrkObjectVisual::OrkObjectVisual(Ogre::SceneManager* scene_manager,
                                   Ogre::SceneNode* parent_node,
                                   rviz::DisplayContext* display_context) :
    display_context_(display_context),
    mesh_entity_(0)
  {
    scene_manager_ = scene_manager;

    // Ogre::SceneNodes form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    frame_node_ = parent_node->createChildSceneNode();
    object_node_ = frame_node_->createChildSceneNode();

    // Initialize the axes
    axes_.reset(new rviz::Axes(scene_manager_, object_node_));
    axes_->setScale(Ogre::Vector3(0.1, 0.1, 0.1));

    // Initialize the name
    name_.reset(new rviz::MovableText("EMPTY"));
    name_->setTextAlignment(rviz::MovableText::H_CENTER,
                            rviz::MovableText::V_CENTER);
    name_->setCharacterHeight(0.08);
    name_->showOnTop();
    name_->setColor(Ogre::ColourValue::White);
    name_->setVisible(false);

    object_node_->attachObject(name_.get());
  }

  OrkObjectVisual::~OrkObjectVisual()
  {
    // Destroy the frame node since we don't need it anymore.
    if (mesh_entity_) {
      display_context_->getSceneManager()->destroyEntity(mesh_entity_);
      mesh_entity_ = 0;
    }
    scene_manager_->destroySceneNode(object_node_);
    scene_manager_->destroySceneNode(frame_node_);
  }

  void OrkObjectVisual::setMessage(const vision_msgs::Detection3D& object,
                                   const std::string& name, bool do_display_id,
                                   bool do_display_name,
                                   bool do_display_confidence)
  {
    vision_msgs::ObjectHypothesisWithPose best_result = object.results[0];
    if(hypothesisPoseContainsNaNs(best_result))
    {
      ROS_WARN_STREAM("NaN pose supplied for object");
    }
    else
    {
      Ogre::Vector3 position(best_result.pose.position.x,
                             best_result.pose.position.y,
                             best_result.pose.position.z);
      Ogre::Quaternion orientation(best_result.pose.orientation.w,
                                   best_result.pose.orientation.x,
                                   best_result.pose.orientation.y,
                                   best_result.pose.orientation.z);
      object_node_->setOrientation(orientation);
      object_node_->setPosition(position);

      std::cout << "original visual, set by message: position "
                << position << " with orientation "
                << orientation << std::endl;

      std::stringstream caption;
      caption << "id: " << best_result.id << std::endl;

      // Set the caption of the object
      if (do_display_confidence)
        caption << "score: " << best_result.score;

      if (caption.str().empty())
        name_->setVisible(false);
      else {
        name_->setCaption(caption.str());
        name_->setVisible(true);
        name_->setLocalTranslation(Ogre::Vector3(0.1, 0, 0));
      }

      attachMesh();
    }
  }

  // Position and orientation are passed through to the highest-level SceneNode.
  void OrkObjectVisual::setFramePosition(const Ogre::Vector3& position)
  {
    frame_node_->setPosition(position);
    std::cout << "frame position: " <<
      position.x << ", " <<
      position.y << ", " <<
      position.z <<std::endl;
  }

  void OrkObjectVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
  {
    frame_node_->setOrientation(orientation);
    std::cout << "frame orientation: " <<
      orientation.w << ", " <<
      orientation.x << ", " <<
      orientation.y << ", " <<
      orientation.z << std::endl;
  }

  bool OrkObjectVisual::hypothesisPoseContainsNaNs(
    vision_msgs::ObjectHypothesisWithPose hypothesis)
  {
    return (std::isnan(hypothesis.pose.position.x) ||
            std::isnan(hypothesis.pose.position.y) ||
            std::isnan(hypothesis.pose.position.z) ||
            std::isnan(hypothesis.pose.orientation.x) ||
            std::isnan(hypothesis.pose.orientation.y) ||
            std::isnan(hypothesis.pose.orientation.z) ||
            std::isnan(hypothesis.pose.orientation.w));
  }

  void OrkObjectVisual::attachMesh()
  {
    // TODO: if using a mesh instead of a prefab cube, be sure to check that
    // the mesh path in question actually exists before attempting to create
    // the mesh.
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "ork_mesh_resource_marker_" << count++;
    std::string id = ss.str();

    mesh_entity_ = display_context_->getSceneManager()
        ->createEntity(id,Ogre::SceneManager::PT_CUBE);
    mesh_entity_->setMaterial(
        Ogre::MaterialManager::getSingleton().create(
            id,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            true));

    // prefab cube size is quite large, so it must be scaled down.
    // To do this, add the mesh to a new child node and scale that node.
    // Otherwise, the text label attached to the object would also be
    // scaled down along with the mesh.
    //
    // Also note that in Ogre, mesh surface normals are not normalized if
    // object is not scaled, so if using a mesh, scale by a tiny amount to
    // force "abnormal normal normalization"
    Ogre::SceneNode* mesh_node_ = object_node_->createChildSceneNode();
    mesh_node_->attachObject(mesh_entity_);
    mesh_node_->setScale(Ogre::Vector3(0.001, 0.001, 0.001));
  }
}
