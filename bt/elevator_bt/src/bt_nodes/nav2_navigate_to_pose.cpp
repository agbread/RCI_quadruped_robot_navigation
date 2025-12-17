#include "elevator_bt/bt_nodes/nav2_navigate_to_pose.hpp"

#include <chrono>
#include <cmath>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace elevator_bt
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;

Nav2NavigateToPose::Nav2NavigateToPose(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  logger_(rclcpp::get_logger("Nav2NavigateToPose")),
  goal_sent_(false),
  timeout_(rclcpp::Duration::from_seconds(300.0))  // 5분 타임아웃 (필요하면 조정)
{
  // 이 노드는 nav2_bt와 별도의 전용 rclcpp Node를 사용
  node_ = rclcpp::Node::make_shared("nav2_navigate_to_pose_bt_node");

  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_,
    "navigate_to_pose");  // nav2 기본 액션 이름

  // TF 버퍼 / 리스너
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 목표 근처 반경(성공 판정 거리) 파라미터
  node_->declare_parameter("success_radius", 0.4);  // 기본 0.4 m
  success_radius_ = node_->get_parameter("success_radius").as_double();
}

BT::NodeStatus Nav2NavigateToPose::onStart()
{
  // 1) goal 읽어서 멤버에 저장
  if (!getInput("goal", goal_pose_))
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose] missing input [goal]");
    return BT::NodeStatus::FAILURE;
  }

  // 2) 액션 서버 준비 확인
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose] navigate_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // 3) goal 메시지 구성
  NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal_pose_;

  // 4) goal 전송
  auto send_goal_options =
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  auto future_goal_handle =
    action_client_->async_send_goal(nav_goal, send_goal_options);

  // 5) goal handle 기다리기
  auto ret = rclcpp::spin_until_future_complete(
    node_, future_goal_handle, std::chrono::seconds(2));

  if (ret != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger_,
      "[Nav2NavigateToPose] Failed to send goal (ret=%d)", static_cast<int>(ret));
    return BT::NodeStatus::FAILURE;
  }

  current_goal_handle_ = future_goal_handle.get();
  if (!current_goal_handle_)
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose] Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  goal_sent_       = true;
  goal_start_time_ = node_->now();

  RCLCPP_INFO(logger_, "[Nav2NavigateToPose] Goal sent");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Nav2NavigateToPose::onRunning()
{
  if (!goal_sent_ || !current_goal_handle_)
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose] onRunning called without a goal");
    return BT::NodeStatus::FAILURE;
  }

  // 0) 전체 타임아웃 (선택)
  if (node_->now() - goal_start_time_ > std::chrono::seconds(120))
  {
    RCLCPP_WARN(logger_, "[Nav2NavigateToPose] navigation timeout, canceling goal");
    action_client_->async_cancel_goal(current_goal_handle_);
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  // 1) 현재 로봇 위치 가져오기 (goal frame 기준)
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    const std::string frame = goal_pose_.header.frame_id;
    tf = tf_buffer_->lookupTransform(
      frame, "base_link", tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "[Nav2NavigateToPose] waiting TF (%s->base_link): %s",
      goal_pose_.header.frame_id.c_str(), ex.what());

    // TF 못 받으면 일단 계속 RUNNING
    return BT::NodeStatus::RUNNING;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;

  const double gx = goal_pose_.pose.position.x;
  const double gy = goal_pose_.pose.position.y;

  const double dist = std::hypot(gx - rx, gy - ry);

  // 2) 일정 거리 안이면 성공으로 간주
  if (dist < success_radius_)
  {
    RCLCPP_INFO(
      logger_,
      "[Nav2NavigateToPose] close enough to goal (dist=%.3f < %.3f), treat as SUCCESS",
      dist, success_radius_);

    if (current_goal_handle_)
    {
      action_client_->async_cancel_goal(current_goal_handle_);
    }
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  // 3) 아직 멀면 액션 결과도 체크 (안정성을 위해 유지)
  auto result_future =
    action_client_->async_get_result(current_goal_handle_);

  auto ret = rclcpp::spin_until_future_complete(
    node_, result_future, std::chrono::milliseconds(10));

  if (ret == rclcpp::FutureReturnCode::TIMEOUT)
  {
    // 아직 진행 중 → 계속 RUNNING
    return BT::NodeStatus::RUNNING;
  }

  if (ret != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger_,
      "[Nav2NavigateToPose] Failed to get result (ret=%d)", static_cast<int>(ret));
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  auto wrapped_result = result_future.get();
  auto code = wrapped_result.code;

  RCLCPP_INFO(
    logger_,
    "[Nav2NavigateToPose] navigation finished with code %d",
    static_cast<int>(code));

  // 4) 취소된 경우만 실패, 나머지는 성공 처리
  if (code == rclcpp_action::ResultCode::CANCELED)
  {
    RCLCPP_WARN(logger_, "[Nav2NavigateToPose] navigation CANCELED");
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  goal_sent_ = false;
  current_goal_handle_.reset();
  return BT::NodeStatus::SUCCESS;
}

void Nav2NavigateToPose::onHalted()
{
  if (goal_sent_ && current_goal_handle_)
  {
    RCLCPP_INFO(logger_, "[Nav2NavigateToPose] Halted, cancel goal");
    action_client_->async_cancel_goal(current_goal_handle_);
  }
  goal_sent_ = false;
  current_goal_handle_.reset();
}

}  // namespace elevator_bt
