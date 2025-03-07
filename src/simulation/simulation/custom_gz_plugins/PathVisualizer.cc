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

#include "PathVisualizer.hh"

/////////////////////////////////////////////////
void PathVisualizer::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  // This is necessary to receive the Render event on eventFilter
  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);

  // Initialize the path data vector with an arbitrary path in case there is no message
  this->pathData = gz::custom_msgs::GeneratedPath();


  // Subscribe to the path topic
  this->node.Subscribe("/generated_path", &PathVisualizer::PathUpdateCallback, this);

  this->showPath = false;
}

/////////////////////////////////////////////////
void PathVisualizer::TogglePath()
{
  this->showPath = !(this->showPath);
  this->dirty = true;
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

void PathVisualizer::PathUpdateCallback(const gz::custom_msgs::GeneratedPath &_msg)
{
    // Check if message is valid before storing
    if (_msg.poses_size() != _msg.twists_size()) {
        gzerr << "Invalid path message: poses and twists size mismatch" << std::endl;
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(this->pathMutex);
        this->pathData = _msg;
        this->dirty = true;
    }
}

/////////////////////////////////////////////////
//! [performRenderingOperations]
void PathVisualizer::PerformRenderingOperations()
{
    if (!this->dirty)
        return;

    if (this->scene == nullptr)
        this->FindScene();

    if (this->scene == nullptr)
        return;

    // Remove existing path markers
    for (auto &marker : this->pathMarkers)
    {
        if (marker)
        {
            marker->RemoveGeometries();
            this->scene->DestroyVisual(marker);
        }
    }
    this->pathMarkers.clear();

    if (this->showPath)
    {
        // Create a local copy of path data under lock
        gz::custom_msgs::GeneratedPath localPathData;
        {
            std::lock_guard<std::mutex> lock(this->pathMutex);
            localPathData = this->pathData;
        }

        // Calculate max linear and angular velocities
        double max_linear_velocity = 0.0;
        double max_angular_velocity = 0.0;
        for (int i = 0; i < localPathData.twists_size(); ++i) {
            auto twist = localPathData.twists(i);
            max_linear_velocity = std::max(max_linear_velocity, std::sqrt(twist.linear().x() * twist.linear().x() + twist.linear().y() * twist.linear().y() + twist.linear().z() * twist.linear().z()));
            max_angular_velocity = std::max(max_angular_velocity, std::sqrt(twist.angular().x() * twist.angular().x() + twist.angular().y() * twist.angular().y() + twist.angular().z() * twist.angular().z()));
        }

        // if (max_linear_velocity == 0.0 && max_angular_velocity == 0.0) {
        //     gzerr << "No path data to visualize" << std::endl;
        //     return;
        // }

        for (int i = 0; i < localPathData.poses_size(); ++i)
        {
            // Verify we have corresponding twist data
            if (i >= localPathData.twists_size())
                break;

            auto pathMarker = this->scene->CreateCone();
            if (!pathMarker)
                continue;

            auto pathMarkerVisual = this->scene->CreateVisual();
            if (!pathMarkerVisual)
                continue;

            pathMarkerVisual->AddGeometry(pathMarker);
            pathMarkerVisual->SetLocalScale(0.2, 0.2, 0.4);

            // Set the position from the message
            pathMarkerVisual->SetLocalPosition(
                localPathData.poses(i).position().x(),
                localPathData.poses(i).position().y(),
                localPathData.poses(i).position().z());

            // Use actual pose from the message
            gz::math::Quaterniond yRotation(0.707, 0.0, 0.707, 0.0); // 90 degrees around y axis to align with the world frame
            gz::math::Quaterniond originalRotation(
                localPathData.poses(i).orientation().w(),
                localPathData.poses(i).orientation().x(),
                localPathData.poses(i).orientation().y(),
                localPathData.poses(i).orientation().z());
            // Combine rotations
            gz::math::Quaterniond finalRotation = yRotation * originalRotation;
            pathMarkerVisual->SetLocalRotation(finalRotation);

            // Color based on velocity
            auto material = this->scene->CreateMaterial();
            if (!material)
                continue;

            auto twist = localPathData.twists(i);
            auto linear_velocity = std::sqrt(
                twist.linear().x() * twist.linear().x() +
                twist.linear().y() * twist.linear().y() +
                twist.linear().z() * twist.linear().z());
            auto angular_velocity = std::sqrt(
                twist.angular().x() * twist.angular().x() +
                twist.angular().y() * twist.angular().y() +
                twist.angular().z() * twist.angular().z());

            auto linear_velocity_normalized = max_linear_velocity > 0.0 ? linear_velocity / max_linear_velocity : 0.0;
            auto angular_velocity_normalized = max_angular_velocity > 0.0 ? angular_velocity / max_angular_velocity : 0.0;
            auto red = 0.5 * linear_velocity_normalized + 0.5 * angular_velocity_normalized;
            auto green = 1 - linear_velocity_normalized * 0.5 - angular_velocity_normalized * 0.5;
            material->SetDiffuse(gz::math::Color(red, green, 0));
            pathMarkerVisual->SetMaterial(material);

            // Add to scene and store reference
            this->scene->RootVisual()->AddChild(pathMarkerVisual);
            this->pathMarkers.push_back(pathMarkerVisual);
        }
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

PathVisualizer::~PathVisualizer()
{
  // Remove existing path markers
  for (auto &marker : this->pathMarkers)
  {
    if (marker)
    {
      marker->RemoveGeometries();
      if (this->scene)
        this->scene->DestroyVisual(marker);
    }
  }
  this->pathMarkers.clear();
}

// Register this plugin
GZ_ADD_PLUGIN(PathVisualizer,
                    gz::gui::Plugin);