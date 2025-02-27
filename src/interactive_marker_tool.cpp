#include <QObject>
#include <QTimer>
#include <QMetaObject>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <chrono>
#include "rviz2_interactive_marker_plugin/interactive_marker_tool.hpp"

namespace interactive_marker_plugin
{

using namespace std::chrono_literals;

InteractiveMarkerTool::InteractiveMarkerTool() : rviz_common::Tool(), marker_count_(0)
{
}

InteractiveMarkerTool::~InteractiveMarkerTool()
{
}

void InteractiveMarkerTool::onInitialize()
{
  nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  marker_count_ = 0;

  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "interactive_marker_server",
    nh_->get_node_base_interface(),
    nh_->get_node_clock_interface(),
    nh_->get_node_logging_interface(),
    nh_->get_node_topics_interface(),
    nh_->get_node_services_interface(),
    rclcpp::QoS(10),
    rclcpp::QoS(10)
  );

  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

visualization_msgs::msg::InteractiveMarker InteractiveMarkerTool::createInteractiveMarker(
  const std::string & marker_name, double pos_x, double pos_y, double pos_z)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.name = marker_name;
  int_marker.description = "";
  int_marker.scale = 1.0;
  int_marker.pose.position.x = pos_x;
  int_marker.pose.position.y = pos_y;
  int_marker.pose.position.z = pos_z;

  // Position control (sphere)
  visualization_msgs::msg::InteractiveMarkerControl pos_control;
  pos_control.name = "move_position";
  pos_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  {
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = 0.2;
    sphere.scale.y = 0.2;
    sphere.scale.z = 0.2;
    sphere.color.r = 0.0;
    sphere.color.g = 1.0;
    sphere.color.b = 0.0;
    sphere.color.a = 1.0;
    pos_control.markers.push_back(sphere);
  }
  pos_control.always_visible = true;
  pos_control.orientation.w = 0.7071;
  pos_control.orientation.x = 0.0;
  pos_control.orientation.y = 0.7071;
  pos_control.orientation.z = 0.0;
  int_marker.controls.push_back(pos_control);

  visualization_msgs::msg::InteractiveMarkerControl rot_control_default;
  rot_control_default.name = "rotate_yaw_default";
  rot_control_default.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  rot_control_default.orientation.w = 0.7071;
  rot_control_default.orientation.x = 0.0;
  rot_control_default.orientation.y = -0.7071;
  rot_control_default.orientation.z = 0.0;
  rot_control_default.always_visible = true;
  int_marker.controls.push_back(rot_control_default);

  visualization_msgs::msg::InteractiveMarkerControl rot_control_arrow;
  rot_control_arrow.name = "rotate_yaw_arrow";
  rot_control_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
  rot_control_arrow.orientation.w = 0.7071;
  rot_control_arrow.orientation.x = 0.0;
  rot_control_arrow.orientation.y = -0.7071;
  rot_control_arrow.orientation.z = 0.0;
  {
    visualization_msgs::msg::Marker arrow;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.scale.x = 0.3;
    arrow.scale.y = 0.075;
    arrow.scale.z = 0.075;
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;
    rot_control_arrow.markers.push_back(arrow);
  }
  rot_control_arrow.always_visible = true;
  int_marker.controls.push_back(rot_control_arrow);

  // Text control
  visualization_msgs::msg::InteractiveMarkerControl text_control;
  text_control.name = "display_text";
  text_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
  text_control.always_visible = true;
  {
    visualization_msgs::msg::Marker text;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.scale.z = 0.2;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    std::string id_text = "ID:" + marker_name.substr(marker_name.find("_") + 1);
    text.text = id_text;
    text.pose.position.x = -0.3;
    text.pose.position.y = -0.3;
    text.pose.position.z = 0.3;
    text_control.markers.push_back(text);
  }
  int_marker.controls.push_back(text_control);

  return int_marker;
}

int InteractiveMarkerTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.leftDown())
  {
    auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);
    Ogre::Vector3 intersection = projection.second;
    if (projection.first)
    {
      std::string marker_name = "marker_" + std::to_string(marker_count_);
      auto int_marker = createInteractiveMarker(marker_name, intersection.x, intersection.y, intersection.z);
      server_->insert(int_marker);
      server_->applyChanges();
      marker_count_++;
      return Finished;
    }
    return Render;
  }
  return 0;
}

void InteractiveMarkerTool::activate() {}
void InteractiveMarkerTool::deactivate() {}

} // namespace interactive_marker_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(interactive_marker_plugin::InteractiveMarkerTool, rviz_common::Tool)
