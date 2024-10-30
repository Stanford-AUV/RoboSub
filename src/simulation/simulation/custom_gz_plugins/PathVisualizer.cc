/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gz/gui/Application.hh>
//! [includeGuiEvents]
#include <gz/gui/GuiEvents.hh>
//! [includeGuiEvents]
#include <gz/gui/MainWindow.hh>
#include <gz/math/Rand.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>
#include <gz/custom_msgs/GeneratedPath.pb.h>

#include "PathVisualizer.hh"

/////////////////////////////////////////////////
void PathVisualizer::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  // This is necessary to receive the Render event on eventFilter
  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);

  // Initialize the path data vector with an arbitrary path in case there is no message
  this->generatedPath = gz::custom_msgs::GeneratedPath();


  // Subscribe to the path topic
  auto node = gz::transport::NodePtr(new gz::transport::Node());
  node->Init();
  this->sub = node->Subscribe("~/generated_path", &PathVisualizer::PathUpdateCallback, this);
}

/////////////////////////////////////////////////
void PathVisualizer::TogglePath()
{
  this->dirty = true;
  this->showPath = !this->showPath;  // Toggle path visibility
}

/////////////////////////////////////////////////
//! [eventFilter]
bool PathVisualizer::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in the render thread, so it's safe to make
    // rendering calls here
    this->PerformRenderingOperations();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}
//! [eventFilter]

void PathVisualizer::PathUpdateCallback(const gz::custom_msgs::GeneratedPath &_msg) {
  // Extract waypoints from the message
  this->dirty = true;  // Mark for redraw
  // Store the path data for use in PerformRenderingOperations
  this->generatedPath = _msg;
}

/////////////////////////////////////////////////
//! [performRenderingOperations]
void PathVisualizer::PerformRenderingOperations()
{
  if (!this->dirty)
  {
    return;
  }

  if (nullptr == this->scene)
  {
    this->FindScene();
  }

  if (nullptr == this->scene)
    return;

  // Remove existing path if any
  if (this->pathVisual)
  {
    this->scene->RootVisual()->RemoveChild(this->pathVisual);
    this->pathVisual.reset();
  }

  // Create and add a new path if showPath is true
  if (this->showPath)
  {
    this->pathVisual = this->scene->CreateVisual("path_visual");
    
    // Replace hardcoded points with message data
    for (size_t i = 0; i < std::size(this->generatedPath.poses); i++)  // Use stored path data
    {
      auto pathMarker = this->scene->CreateCone();
      auto pathMarkerVisual = this->scene->CreateVisual();
      pathMarkerVisual->AddGeometry(pathMarker);
      pathMarkerVisual->SetLocalScale(0.2, 0.2, 0.4);
      
      // Use the actual pose from the message
      pathMarkerVisual->SetLocalPosition(this->generatedPath.poses[i].position());
      pathMarkerVisual->SetLocalRotation(this->generatedPath.poses[i].orientation());
      
      // Color based on velocity -- TODO: make this actually work
      auto material = this->scene->CreateMaterial();
      auto twist = this->generatedPath.twists[i];
      auto linear_velocity = std::sqrt(twist.linear().x() * twist.linear().x() + twist.linear().y() * twist.linear().y() + twist.linear().z() * twist.linear().z());
      auto angular_velocity = std::sqrt(twist.angular().x() * twist.angular().x() + twist.angular().y() * twist.angular().y() + twist.angular().z() * twist.angular().z());
      // normalize velocity to 0-1
      auto linear_velocity_normalized = linear_velocity / 1.5; // need to normalize by max velocity
      auto angular_velocity_normalized = angular_velocity / 1.5; // need to normalize by max velocity
      auto red = 0.5 * linear_velocity_normalized + 0.5 * angular_velocity_normalized;
      auto green = 1 - linear_velocity_normalized;
      material->SetDiffuse(gz::math::Color(red, green, 0));
      pathMarkerVisual->SetMaterial(material);
      
      this->pathVisual->AddChild(pathMarkerVisual);
    }
    
    this->scene->RootVisual()->AddChild(this->pathVisual);
  }

  this->dirty = false;
}
//! [performRenderingOperations]

/////////////////////////////////////////////////
void PathVisualizer::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
      << "Using scene [" << scene->Name() << "]" << std::endl;
  }

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }

  this->scene = scenePtr;
}

// Register this plugin
GZ_ADD_PLUGIN(PathVisualizer,
                    gz::gui::Plugin);