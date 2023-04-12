// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pcdless_common/fix2mgrs.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ground_msgs/srv/ground.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2_ros/transform_broadcaster.h>

namespace pcdless::path_monitor
{
class Fix2Pose : public rclcpp::Node
{
public:
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using Ground = ground_msgs::srv::Ground;

  Fix2Pose() : Node("fix_to_pose")
  {
    using std::placeholders::_1;

    // Subscriber
    auto fix_cb = std::bind(&Fix2Pose::on_fix, this, _1);
    sub_fix_ = create_subscription<NavSatFix>("fix_topic", 10, fix_cb);

    // Publisher
    pub_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose_topic_ = pub_pose_stamped_->get_topic_name();

    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ =
      create_client<Ground>("ground", rmw_qos_profile_services_default, service_callback_group_);
  }

private:
  rclcpp::Client<Ground>::SharedPtr client_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string pose_topic_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped_;
  std::optional<geometry_msgs::msg::Pose> ground_pose_{std::nullopt};

  void publish_tf(const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time &)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = pose_topic_;
    t.transform.translation.x = pose.pose.position.x;
    t.transform.translation.y = pose.pose.position.y;
    t.transform.translation.z = pose.pose.position.z;
    t.transform.rotation.w = pose.pose.orientation.w;
    t.transform.rotation.x = pose.pose.orientation.x;
    t.transform.rotation.y = pose.pose.orientation.y;
    t.transform.rotation.z = pose.pose.orientation.z;

    tf_broadcaster_->sendTransform(t);
  }

  void call_ground_service(const Eigen::Vector3f & xyz)
  {
    auto request = std::make_shared<Ground::Request>();
    request->point.x = xyz.x();
    request->point.y = xyz.y();
    request->point.z = xyz.z();
    auto on_ground = [this](rclcpp::Client<Ground>::SharedFuture result) {
      ground_pose_ = result.get()->pose;
    };
    auto result = client_->async_send_request(request, on_ground);
  }

  void on_fix(const sensor_msgs::msg::NavSatFix & msg)
  {
    Eigen::Vector3d mgrs = common::fix_to_mgrs(msg);
    call_ground_service(mgrs.cast<float>());

    if (!ground_pose_.has_value()) return;
    float height = ground_pose_->position.z;

    RCLCPP_INFO_STREAM(
      this->get_logger(), mgrs.x() << " " << mgrs.y() << " (" << msg.latitude << ", "
                                   << msg.longitude << ") " << height);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = msg.header.stamp;
    pose.pose.position.x = mgrs.x();
    pose.pose.position.y = mgrs.y();
    pose.pose.position.z = height;

    pose.pose.orientation.w = 1;
    pose.pose.orientation.z = 0;

    pub_pose_stamped_->publish(pose);

    publish_tf(pose, msg.header.stamp);
  }
};

}  // namespace pcdless::path_monitor
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pcdless::path_monitor::Fix2Pose>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
