#ifndef PATH_VISUALIZER_HH_
#define PATH_VISUALIZER_HH_

#include <gz/gui/qt.h>
#include <gz/gui/Plugin.hh>
#include <gz/rendering/Scene.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/custom_msgs/GeneratedPath.pb.h>

/// \brief Example of a GUI plugin that uses Gazebo Rendering.
/// This plugin works with Gazebo GUI's MinimalScene or any plugin providing
/// similar functionality.
class PathVisualizer : public gz::gui::Plugin
{
  Q_OBJECT

  ///\brief Called once at startup.
  public: void LoadConfig(const tinyxml2::XMLElement *) override;

  /// \brief Callback when user clicks button.
  public slots: void TogglePath();

  /// \brief Callback for path update messages.
  public slots: void PathUpdateCallback(const gz::custom_msgs::GeneratedPath &_msg);

  /// \brief Callback for all installed event filters.
  /// \param[in] _obj Object that received the event
  /// \param[in] _event Event
  private: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \brief All rendering operations must happen within this call
  private: void PerformRenderingOperations();

  /// \brief Encapsulates the logic to find the rendering scene through the
  /// render engine singleton.
  private: void FindScene();

  /// \brief Marks when a new change has been requested.
  private: bool dirty{false};

  /// \brief Pointer to the rendering scene.
  private: gz::rendering::ScenePtr scene{nullptr};

  /// \brief Flag to toggle path visibility.
  private: bool showPath{false};

  /// \brief Pointer to the path visual.
  private: gz::rendering::VisualPtr pathVisual{nullptr};

  /// \brief Subscriber for the path topic.
  private: gz::transport::Node node;

  /// \brief Stored path data from latest message
  private: gz::custom_msgs::GeneratedPath pathData;

};

#endif
