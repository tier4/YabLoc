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
#include <std_srvs/srv/set_bool.hpp>

namespace set_bool_panel
{

class SetBoolPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  using SetBool = std_srvs::srv::SetBool;

  explicit SetBoolPanel(QWidget * parent = nullptr);
  virtual ~SetBoolPanel();

  void onInitialize() override;

private Q_SLOTS:
  void call_enable_service();
  void call_disable_service();

private:
  QPushButton * enable_button_{nullptr};
  QPushButton * disable_button_{nullptr};
  QLabel * label_{nullptr};

  rclcpp::Client<SetBool>::SharedPtr client_;
  void on_response(rclcpp::Client<SetBool>::SharedFuture future);
  void call_service(bool flag);
};

}  // namespace set_bool_panel
