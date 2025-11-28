#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

namespace gazebo
{

class WorldTfPublisher : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_  = gazebo_ros::Node::Get(sdf);   // <ros> 태그 기반 ROS2 노드

    // SDF 인자 (없으면 기본값 사용)
    world_frame_ = sdf->HasElement("world_frame") ?
      sdf->Get<std::string>("world_frame") : "world";
    base_frame_ = sdf->HasElement("base_frame") ?
      sdf->Get<std::string>("base_frame") : "base_link";

    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WorldTfPublisher::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(),
      "WorldTfPublisher loaded for model [%s], world_frame=[%s], base_frame=[%s]",
      model_->GetName().c_str(), world_frame_.c_str(), base_frame_.c_str());
  }

private:
  void OnUpdate()
  {
    // Gazebo world 기준 로봇 pose
    const ignition::math::Pose3d pose = model_->WorldPose();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_->now();
    t.header.frame_id = world_frame_;
    t.child_frame_id  = base_frame_;

    t.transform.translation.x = pose.Pos().X();
    t.transform.translation.y = pose.Pos().Y();
    t.transform.translation.z = pose.Pos().Z();

    t.transform.rotation.x = pose.Rot().X();
    t.transform.rotation.y = pose.Rot().Y();
    t.transform.rotation.z = pose.Rot().Z();
    t.transform.rotation.w = pose.Rot().W();

    tf_broadcaster_->sendTransform(t);
  }

  // --- 멤버 ---
  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  event::ConnectionPtr update_conn_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string world_frame_{"world"};
  std::string base_frame_{"base_link"};
};

GZ_REGISTER_MODEL_PLUGIN(WorldTfPublisher)

} // namespace gazebo
