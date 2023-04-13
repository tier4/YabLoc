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

#include "set_bool_panel.hpp"

#include <QHBoxLayout>
#include <QWidget>
#include <rviz_common/display_context.hpp>

#include <sstream>

namespace set_bool_panel
{
SetBoolPanel::SetBoolPanel(QWidget * parent) : Panel(parent)
{
  using std::placeholders::_1;

  // Button
  enable_button_ = new QPushButton();
  enable_button_->setText("enable");
  enable_button_->setFixedSize(QSize(60, 30));
  QObject::connect(enable_button_, SIGNAL(clicked()), this, SLOT(call_enable_service()));

  // Button
  disable_button_ = new QPushButton();
  disable_button_->setText("disable");
  disable_button_->setFixedSize(QSize(60, 30));
  QObject::connect(disable_button_, SIGNAL(clicked()), this, SLOT(call_disable_service()));

  // Label
  label_ = new QLabel();
  label_->setText("Status:");

  // Layout
  QHBoxLayout * main_layout = new QHBoxLayout;
  main_layout->setContentsMargins(10, 10, 10, 10);

  // Add widges
  main_layout->addWidget(enable_button_);
  main_layout->addWidget(disable_button_);
  main_layout->addWidget(label_);

  setLayout(main_layout);
}

SetBoolPanel::~SetBoolPanel() { client_.reset(); }

void SetBoolPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  client_ = node->create_client<SetBool>("/switch");
}

void SetBoolPanel::call_enable_service() { call_service(true); }
void SetBoolPanel::call_disable_service() { call_service(false); }

void SetBoolPanel::call_service(bool flag)
{
  label_->setText("call_service()");

  using std::placeholders::_1;
  using namespace std::literals::chrono_literals;

  if (!client_->wait_for_service(1s)) {
    label_->setText("server is not ready");
    return;
  }

  auto request = std::make_shared<SetBool::Request>();
  request->data = flag;
  auto result_future =
    client_->async_send_request(request, std::bind(&SetBoolPanel::on_response, this, _1));
  label_->setText("Status: requesting...");
}

void SetBoolPanel::on_response(rclcpp::Client<SetBool>::SharedFuture future)
{
  if (future.valid()) {
    label_->setText("Status: requested successfully");
  } else {
    label_->setText("Status: something went wrong");
  }
}
}  // namespace set_bool_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(set_bool_panel::SetBoolPanel, rviz_common::Panel)
