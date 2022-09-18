// Copyright 2020 Tier IV, Inc.
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

#include "plotter2d_overlay_display.hpp"

#include <QPainter>
#include <QStaticText>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/uniform_string_stream.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

namespace rviz_plugins
{
Plotter2dOverlayDisplay::Plotter2dOverlayDisplay()
{
  data_buffer_.resize(100);

  property_topic_name_ = new rviz_common::properties::StringProperty(
    "Topic", "/", "String", this, SLOT(updateVisualization()));
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 256, "Width of the plotter window", this, SLOT(updateVisualization()), this);
  property_width_->setMin(10);
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 256, "Height of the plotter window", this, SLOT(updateVisualization()), this);
  property_height_->setMin(10);

  property_fg_color_ = new rviz_common::properties::ColorProperty(
    "Foreground Color", QColor(255, 255, 40), "Foreground Color", this, SLOT(updateVisualization()),
    this);
  property_bg_color_ = new rviz_common::properties::ColorProperty(
    "Background Color", QColor(50, 50, 50), "Background Color", this, SLOT(updateVisualization()),
    this);

  property_fg_alpha_ = new rviz_common::properties::FloatProperty(
    "Foregrund Alpha", 0.8, "Foreground Alpha", this, SLOT(updateVisualization()), this);
  property_bg_alpha_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", 0.2, "Background Alpha", this, SLOT(updateVisualization()), this);
  property_fg_alpha_->setMin(0.0);
  property_fg_alpha_->setMax(1.0);
  property_bg_alpha_->setMin(0.0);
  property_bg_alpha_->setMax(1.0);

  property_font_size_ = new rviz_common::properties::IntProperty(
    "Font Size", 12, "Font Size", this, SLOT(updateVisualization()));
  property_font_size_->setMin(0);
}

Plotter2dOverlayDisplay::~Plotter2dOverlayDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void Plotter2dOverlayDisplay::onInitialize()
{
  static int count = 0;

  rviz_common::UniformStringStream ss;
  ss << "Plotter2dDisplay" << count++;
  auto logger = context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger();
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(scene_manager_, logger, ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
}

void Plotter2dOverlayDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void Plotter2dOverlayDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void Plotter2dOverlayDisplay::subscribe()
{
  topic_name_ = property_topic_name_->getStdString();
  if (topic_name_.length() > 0 && topic_name_ != "/") {
    rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    sub_float32_ = raw_node->create_subscription<std_msgs::msg::Float32>(
      topic_name_, 10,
      std::bind(&Plotter2dOverlayDisplay::processMessage, this, std::placeholders::_1));
  }
}

void Plotter2dOverlayDisplay::unsubscribe() { sub_float32_.reset(); }

void Plotter2dOverlayDisplay::processMessage(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) return;

  data_buffer_.push_back(msg_ptr->data);
  last_msg_ptr_ = msg_ptr;

  queueRender();
}

void Plotter2dOverlayDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::string current_topic_name = property_topic_name_->getStdString();
  if (current_topic_name != topic_name_) {
    topic_name_ = current_topic_name;
    subscribe();
  }

  updateVisualization();
}

void Plotter2dOverlayDisplay::updateVisualization()
{
  if (!last_msg_ptr_) return;

  QColor background_color(property_bg_color_->getColor());
  background_color.setAlpha(255 * property_bg_alpha_->getFloat());
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  hud_ = buffer.getQImage(*overlay_);
  hud_.fill(background_color);

  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  const int w = overlay_->getTextureWidth();
  const int h = overlay_->getTextureHeight();

  const int font_size_ = property_font_size_->getInt();

  QPainter painter(&hud_);
  QColor fg_color_(property_fg_color_->getColor());
  fg_color_.setAlpha(255 * property_fg_alpha_->getFloat());

  if (font_size_ != 0) {
    QFont font("Liberation Sans");
    font.setPointSize(font_size_);
    font.setBold(true);
    painter.setFont(font);
  }

  {
    const std::string text = property_topic_name_->getStdString();
    std::string color_wrapped_text =
      (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>") % text %
       fg_color_.red() % fg_color_.green() % fg_color_.blue() % fg_color_.alpha())
        .str();

    QStaticText static_text(
      boost::algorithm::replace_all_copy(color_wrapped_text, "\n", "<br >").c_str());

    static_text.setTextWidth(w);
    painter.drawStaticText(0, 0, static_text);
  }

  const double line_width = 1.0f;
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(fg_color_, line_width, Qt::SolidLine));

  {
    painter.drawLine(0, 0, 0, h);
    painter.drawLine(0, h, w, h);
    painter.drawLine(w, h, w, 0);
    painter.drawLine(w, 0, 0, 0);
  }

  if (!data_buffer_.empty()) {
    const double max_value = *std::max_element(data_buffer_.begin(), data_buffer_.end());
    const double min_value = *std::min_element(data_buffer_.begin(), data_buffer_.end());
    const double margined_max_value = max_value + (max_value - min_value) / 2;
    const double margined_min_value = min_value - (max_value - min_value) / 2;

    for (size_t i = 1; i < data_buffer_.size(); i++) {
      double v_prev =
        (margined_max_value - data_buffer_[i - 1]) / (margined_max_value - margined_min_value);
      double v = (margined_max_value - data_buffer_[i]) / (margined_max_value - margined_min_value);
      double u_prev = (i - 1) / (float)data_buffer_.size();
      double u = i / (float)data_buffer_.size();

      // chop within 0 ~ 1
      v_prev = std::max(std::min(v_prev, 1.0), 0.0);
      u_prev = std::max(std::min(u_prev, 1.0), 0.0);
      v = std::max(std::min(v, 1.0), 0.0);
      u = std::max(std::min(u, 1.0), 0.0);

      uint16_t x_prev = (int)(u_prev * w);
      uint16_t x = (int)(u * w);
      uint16_t y_prev = (int)(v_prev * h);
      uint16_t y = (int)(v * h);
      painter.drawLine(x_prev, y_prev, x, y);
    }
  }
  painter.end();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Plotter2dOverlayDisplay, rviz_common::Display)
