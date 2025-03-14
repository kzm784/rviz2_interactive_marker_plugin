#ifndef RVIZ2_INTERACTIVE_MARKER_PLUGIN_INTERACTIVE_MARKER_TOOL_HPP_
#define RVIZ2_INTERACTIVE_MARKER_PLUGIN_INTERACTIVE_MARKER_TOOL_HPP_

#include <rviz_common/tool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace interactive_marker_plugin
{

class InteractiveMarkerTool : public rviz_common::Tool
{
public:
  InteractiveMarkerTool();
  virtual ~InteractiveMarkerTool();

  virtual void onInitialize() override;
  virtual void activate() override;
  virtual void deactivate() override;
  virtual int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  // Creates a marker with position control (sphere), rotation control (default circular widget plus arrow), and a text label.
  visualization_msgs::msg::InteractiveMarker createInteractiveMarker(
    const std::string & marker_name, double pos_x, double pos_y, double pos_z);

  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  int marker_count_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
};

} // namespace interactive_marker_plugin

#endif  // RVIZ2_INTERACTIVE_MARKER_PLUGIN_INTERACTIVE_MARKER_TOOL_HPP_
