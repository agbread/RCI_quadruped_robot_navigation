#include "elevator_bt/bt_nodes/get_cabin_goal.hpp"

#include <yaml-cpp/yaml.h>

namespace elevator_bt {

GetCabinGoal::GetCabinGoal(const std::string& name,
                           const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetCabinGoal::tick()
{
  std::string yaml_path;
  if (!getInput("elevator_yaml", yaml_path)) {
    throw BT::RuntimeError("GetCabinGoal: missing required input [elevator_yaml]");
  }

  int floor_idx = 0;
  // floor_idx는 현재 계산에는 사용하지 않지만, 포트만 읽어서 보관 가능
  getInput("floor_idx", floor_idx);

  auto goal = makeCabinGoal(yaml_path, floor_idx);
  setOutput("cabin_goal", goal);

  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped
GetCabinGoal::makeCabinGoal(const std::string& yaml_path, int /*floor_idx*/)
{
  YAML::Node root = YAML::LoadFile(yaml_path);

  auto zones = root["zones"];
  auto cabin_min = zones["cabin_min"];
  auto cabin_max = zones["cabin_max"];

  if (!cabin_min || !cabin_max || cabin_min.size() < 2 || cabin_max.size() < 2) {
    throw BT::RuntimeError("GetCabinGoal: invalid zones.cabin_min/max in yaml: " + yaml_path);
  }

  double min_x = cabin_min[0].as<double>();
  double min_y = cabin_min[1].as<double>();
  double max_x = cabin_max[0].as<double>();
  double max_y = cabin_max[1].as<double>();

  geometry_msgs::msg::PoseStamped goal;

  // frame_id는 world 기준이라고 가정 (추측입니다: elevator plugin도 world 기준으로 동작한다고 가정)
  goal.header.frame_id = "world";
  goal.header.stamp = rclcpp::Clock().now();

  goal.pose.position.x = 0.5 * (min_x + max_x);
  goal.pose.position.y = 0.5 * (min_y + max_y);

  // Nav2는 보통 2D로 쓰니까 z는 0으로 두어도 무방할 가능성이 높음 (추측입니다)
  goal.pose.position.z = 0.0;

  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = 0.0;
  goal.pose.orientation.w = 1.0;

  return goal;
}

}  // namespace elevator_bt
