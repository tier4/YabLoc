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

#pragma once

#include <QLabel>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <optional>

namespace initialpose_plugins
{

class InitialPosePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  explicit InitialPosePanel(QWidget * parent = nullptr);
  virtual ~InitialPosePanel();

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void toggle();

private:
  double last_dt_{0};
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  QPushButton * toggle_button_{nullptr};
  QLabel * pose_label_{nullptr};
  QLabel * time_label_{nullptr};

  rclcpp::Clock::SharedPtr clock_{nullptr};
  std::optional<PoseCovStamped> last_initial_pose_{std::nullopt};

  void poseCallback(const PoseCovStamped & msg);

  void timerCallback();
};

}  // namespace initialpose_plugins
