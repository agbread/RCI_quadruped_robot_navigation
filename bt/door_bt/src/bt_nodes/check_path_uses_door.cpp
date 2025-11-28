#include "door_bt/bt_nodes/check_path_uses_door.hpp"

CheckPathUsesDoor::CheckPathUsesDoor(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("bt_check_path_uses_door");

  sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(1),
    std::bind(&CheckPathUsesDoor::pathCallback, this, std::placeholders::_1));
}

BT::PortsList CheckPathUsesDoor::providedPorts()
{
  return {
    BT::InputPort<double>("door_p1_x"),
    BT::InputPort<double>("door_p1_y"),
    BT::InputPort<double>("door_p2_x"),
    BT::InputPort<double>("door_p2_y")
  };
}

void CheckPathUsesDoor::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  last_path_ = msg;
}

BT::NodeStatus CheckPathUsesDoor::tick()
{
  // ★ 추가: 이 노드의 콜백을 직접 처리
  rclcpp::spin_some(node_);

  double x1, y1, x2, y2;
  if (!getInput("door_p1_x", x1) ||
      !getInput("door_p1_y", y1) ||
      !getInput("door_p2_x", x2) ||
      !getInput("door_p2_y", y2))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "CheckPathUsesDoor: missing input ports");
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path::SharedPtr path;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path = last_path_;
  }

  if (!path || path->poses.size() < 2)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "CheckPathUsesDoor: no valid path received yet");
    return BT::NodeStatus::FAILURE;
  }

  if (!path->poses.empty() && path->header.frame_id != "world")
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "CheckPathUsesDoor: expected frame_id=world, got [%s]",
      path->header.frame_id.c_str());
  }

  for (size_t i = 0; i + 1 < path->poses.size(); ++i)
  {
    const auto& p0 = path->poses[i].pose.position;
    const auto& p1 = path->poses[i+1].pose.position;

    if (segmentsIntersect(
          p0.x, p0.y,
          p1.x, p1.y,
          x1,  y1,
          x2,  y2))
    {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "CheckPathUsesDoor[%s]: path intersects door segment",
        this->name().c_str());
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}


// ----------------- 기하 유틸 -----------------

int CheckPathUsesDoor::orientation(double ax, double ay,
                                   double bx, double by,
                                   double cx, double cy)
{
  // (a,b,c) 방향성: 0=일직선, 1=시계방향, 2=반시계
  double val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
  const double eps = 1e-9;
  if (std::fabs(val) < eps) return 0;
  return (val > 0.0) ? 1 : 2;
}

bool CheckPathUsesDoor::onSegment(double ax, double ay,
                                  double bx, double by,
                                  double cx, double cy)
{
  // 점 b가 선분 ac 위에 있는지 (collinear 가정)
  const double eps = 1e-9;
  return (bx <= std::max(ax, cx) + eps &&
          bx + eps >= std::min(ax, cx) &&
          by <= std::max(ay, cy) + eps &&
          by + eps >= std::min(ay, cy));
}

bool CheckPathUsesDoor::segmentsIntersect(
  double p1x, double p1y,
  double p2x, double p2y,
  double q1x, double q1y,
  double q2x, double q2y)
{
  int o1 = orientation(p1x, p1y, p2x, p2y, q1x, q1y);
  int o2 = orientation(p1x, p1y, p2x, p2y, q2x, q2y);
  int o3 = orientation(q1x, q1y, q2x, q2y, p1x, p1y);
  int o4 = orientation(q1x, q1y, q2x, q2y, p2x, p2y);

  // 일반적인 교차 케이스
  if (o1 != o2 && o3 != o4)
    return true;

  // 특수 케이스: collinear + 선분 위에 있는 경우
  if (o1 == 0 && onSegment(p1x, p1y, q1x, q1y, p2x, p2y)) return true;
  if (o2 == 0 && onSegment(p1x, p1y, q2x, q2y, p2x, p2y)) return true;
  if (o3 == 0 && onSegment(q1x, q1y, p1x, p1y, q2x, q2y)) return true;
  if (o4 == 0 && onSegment(q1x, q1y, p2x, p2y, q2x, q2y)) return true;

  return false;
}
