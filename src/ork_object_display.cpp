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

#include <fstream>
#include <stdio.h>
#include <string>

#include <boost/foreach.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/frame_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>


#include "ork_object_visual.h"

#include "ork_object_display.h"

namespace vision_msgs_visualization
{

OrkObjectDisplay::OrkObjectDisplay() {
  do_display_id_ = new rviz::BoolProperty("ID", false, "Display the DB ID or not.", this);
  do_display_name_ = new rviz::BoolProperty("Name", true, "Display the object name or not.", this);
  do_display_confidence_ = new rviz::BoolProperty("Confidence", true, "Display the match confidence or not.", this);
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void OrkObjectDisplay::onInitialize() {
  MFDClass::onInitialize();
}

// Clear the visuals by deleting their objects.
void
OrkObjectDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// This is our callback to handle an incoming message.
void
OrkObjectDisplay::processMessage(const vision_msgs::Detection3DArrayConstPtr& result)
{
  visuals_.clear();
  for(auto it = result->detections.begin(); it != result->detections.end(); ++it)
  {
    vision_msgs::Detection3D object = *it;

    // Create a new visual for that message
    auto visual = boost::shared_ptr<vision_msgs_visualization::OrkObjectVisual>(
        new vision_msgs_visualization::OrkObjectVisual(context_->getSceneManager(), scene_node_,
                            context_));
    visuals_.push_back(visual);

    // std::string mesh_resource = "package://vision_msgs_visualization/mesh/mug.dae";
    // if (rviz::loadMeshFromResource(mesh_resource).isNull()) {
    //   ROS_DEBUG_STREAM("Could not load " << mesh_resource);
    //   return;
    // }
    std::string name = "obj";

    // Define the visual
    visual->setMessage(object, name, do_display_id_->getBool(), do_display_name_->getBool(), do_display_confidence_->getBool());

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(object.header.frame_id, object.header.stamp, position, orientation))
    {
      ROS_DEBUG(
          "Error transforming from frame '%s' to frame '%s'", object.header.frame_id.c_str(), qPrintable( fixed_frame_ ));
      return;
    }
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);
  }
}

}

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_msgs_visualization::OrkObjectDisplay, rviz::Display)
