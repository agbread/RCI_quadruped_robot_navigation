#include "floor_patrol/bt_nodes/map_switch_slam_toolbox.hpp"
#include <chrono>
using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList MapSwitchSlamToolbox::providedPorts() {
  return {
    BT::InputPort<std::string>("slam_ns"),                                 // "slam_toolbox"
    BT::InputPort<std::map<int,std::string>>("serialized_map"),            // floor -> posegraph path
    BT::InputPort<int>("floor"),                                           // target floor
    BT::InputPort<int>("wait_map_ms", 300, "wait after load (ms)"),        // optional
    BT::InputPort<std::string>("clear_costmaps", std::string("true"),
                               "clear local/global costmaps after load")
  };
}

rclcpp::Node::SharedPtr MapSwitchSlamToolbox::getOrMakeNode_() {
  if (node_) return node_;
  node_ = rclcpp::Node::make_shared("bt_map_switch_slam_toolbox");
  return node_;
}

std::string MapSwitchSlamToolbox::serviceName_(const std::string& slam_ns) const {
  // ex) "/slam_toolbox/deserialize_map"
  if (slam_ns.empty() || slam_ns[0] != '/')
    return "/" + slam_ns + "/deserialize_map";
  return slam_ns + "/deserialize_map";
}

BT::NodeStatus MapSwitchSlamToolbox::tick() {
  auto node = getOrMakeNode_();

  const auto ns_res   = getInput<std::string>("slam_ns");
  const auto map_res  = getInput<std::map<int,std::string>>("serialized_map");
  const auto fl_res   = getInput<int>("floor");
  const auto wait_res = getInput<int>("wait_map_ms");
  const auto clr_res  = getInput<std::string>("clear_costmaps");

  if (!ns_res)   throw BT::RuntimeError("MapSwitchSlamToolbox: missing slam_ns: ", ns_res.error());
  if (!map_res)  throw BT::RuntimeError("MapSwitchSlamToolbox: missing serialized_map: ", map_res.error());
  if (!fl_res)   throw BT::RuntimeError("MapSwitchSlamToolbox: missing floor: ", fl_res.error());

  const std::string slam_ns = ns_res.value();
  const auto& ser_map = map_res.value();
  const int floor = fl_res.value();
  const int wait_ms = wait_res ? wait_res.value() : 300;
  const bool do_clear = !clr_res || clr_res.value() == "true";

  // 파일 경로 선택
  auto it = ser_map.find(floor);
  if (it == ser_map.end()) {
    RCLCPP_ERROR(node->get_logger(), "[MapSwitchSlamToolbox] serialized_map has no entry for floor=%d", floor);
    return BT::NodeStatus::FAILURE;
  }
  const std::string filename = it->second;

  // SLAM Toolbox deserialize service
  const std::string srv_name = serviceName_(slam_ns);
  auto cli_deser = node->create_client<Deserialize>(srv_name);
  if (!cli_deser->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "[MapSwitchSlamToolbox] service %s not available", srv_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<Deserialize::Request>();
  // --- 필드 주의 ---
  // Humble 기준으로 Request에 filename / match_type / initial_pose 등이 존재합니다.
  // 여기서는 단순 로드(정합 없이)로 사용:
  req->filename = filename;
  // 선택 필드(환경마다 다를 수 있음). 존재하면 아래 주석 해제해서 사용:
  // req->match_type = 0;       // 0: NO_MATCHING (확실하지 않음)
  // req->initial_pose.x = 0.0; // 필요없음
  // req->initial_pose.y = 0.0;
  // req->initial_pose.theta = 0.0;
  // req->set_initial_pose = false;

  auto fut = cli_deser->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, fut, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[MapSwitchSlamToolbox] deserialize_map call timeout");
    return BT::NodeStatus::FAILURE;
  }
  auto resp = fut.get();
  (void)resp; // 일반적으로 응답 본문 없음

  RCLCPP_INFO(node->get_logger(), "[MapSwitchSlamToolbox] loaded posegraph: %s", filename.c_str());

  // (선택) costmap clear
  if (do_clear) {
    auto clear_once = [&](const std::string& srv)->bool{
      auto c = node->create_client<std_srvs::srv::Empty>(srv);
      if (!c->wait_for_service(1s)) return false;
      auto r = std::make_shared<std_srvs::srv::Empty::Request>();
      auto f = c->async_send_request(r);
      return rclcpp::spin_until_future_complete(node_, f, 5s) == rclcpp::FutureReturnCode::SUCCESS;
    };

    bool ok_gl = clear_once("/global_costmap/clear");
    bool ok_lo = clear_once("/local_costmap/clear");
    RCLCPP_INFO(node->get_logger(), "[MapSwitchSlamToolbox] clear costmaps: global=%s local=%s",
                ok_gl ? "ok" : "skip", ok_lo ? "ok" : "skip");
  }

  // 맵/TF 안정화 대기
  if (wait_ms > 0) {
    rclcpp::sleep_for(std::chrono::milliseconds(wait_ms));
    rclcpp::spin_some(node);
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace floor_patrol
