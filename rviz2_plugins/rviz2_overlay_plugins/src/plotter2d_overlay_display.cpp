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
  using namespace rviz_common::properties;

  property_topic_name_ =
    new StringProperty("Topic", "/", "String", this, SLOT(updateVisualization()));
  property_left_ = new IntProperty(
    "Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ =
    new IntProperty("Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_width_ = new IntProperty(
    "Width", 256, "Width of the plotter window", this, SLOT(updateVisualization()), this);
  property_width_->setMin(10);
  property_height_ = new IntProperty(
    "Height", 256, "Height of the plotter window", this, SLOT(updateVisualization()), this);
  property_height_->setMin(10);

  property_fg_color_ = new ColorProperty(
    "Foreground Color", QColor(255, 255, 40), "Foreground Color", this, SLOT(updateVisualization()),
    this);
  property_bg_color_ = new ColorProperty(
    "Background Color", QColor(50, 50, 50), "Background Color", this, SLOT(updateVisualization()),
    this);

  property_show_caption_ =
    new BoolProperty("Show Caption", true, "Show Caption", this, SLOT(updateVisualization()), this);

  property_fg_alpha_ = new FloatProperty(
    "Foregrund Alpha", 0.8, "Foreground Alpha", this, SLOT(updateVisualization()), this);
  property_fg_alpha_->setMin(0.0);
  property_fg_alpha_->setMax(1.0);

  property_bg_alpha_ = new FloatProperty(
    "Background Alpha", 0.2, "Background Alpha", this, SLOT(updateVisualization()), this);
  property_bg_alpha_->setMin(0.0);
  property_bg_alpha_->setMax(1.0);

  property_update_interval_ = new FloatProperty(
    "Update Interval", 0.05, "Update Interval", this, SLOT(updateVisualization()), this);
  property_update_interval_->setMin(0.0);
  property_update_interval_->setMax(100.0);

  property_font_size_ =
    new IntProperty("Font Size", 12, "Font Size", this, SLOT(updateVisualization()));
  property_font_size_->setMin(0);

  property_line_thick_ =
    new IntProperty("Line Thick", 2, "Line Thick", this, SLOT(updateVisualization()));
  property_line_thick_->setMin(1);

  property_buffer_length_ =
    new IntProperty("Buffer Length", 100, "buffer Length", this, SLOT(updateBufferLength()));
  property_buffer_length_->setMin(1);

  data_buffer_.resize(property_buffer_length_->getInt());
}

Plotter2dOverlayDisplay::~Plotter2dOverlayDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void Plotter2dOverlayDisplay::updateBufferLength()
{
  data_buffer_.resize(property_buffer_length_->getInt());
  updateVisualization();
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

  updateVisualization();
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

  last_msg_ptr_ = msg_ptr;

  max_value_ = *std::max_element(data_buffer_.begin(), data_buffer_.end());
  min_value_ = *std::min_element(data_buffer_.begin(), data_buffer_.end());

  max_value_ = std::max(last_msg_ptr_->data, max_value_);
  min_value_ = std::min(last_msg_ptr_->data, min_value_);
  if (std::abs(max_value_ - min_value_) < 1e-6f) {
    max_value_ += 0.5f;
    min_value_ -= 0.5f;
  }

  queueRender();
}

void Plotter2dOverlayDisplay::update(float wall_dt, float ros_dt)
{
  (void)ros_dt;

  std::string current_topic_name = property_topic_name_->getStdString();
  if (current_topic_name != topic_name_) {
    topic_name_ = current_topic_name;
    subscribe();
  }

  if (wall_dt + last_time_ > update_interval_) {
    last_time_ = 0.f;
    if (last_msg_ptr_)
      data_buffer_.push_back(last_msg_ptr_->data);
    else
      data_buffer_.push_back(0);
  } else {
    last_time_ += wall_dt;
  }
  updateVisualization();
}

void Plotter2dOverlayDisplay::updateVisualization()
{
  update_interval_ = property_update_interval_->getFloat();

  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();

  // Fill background
  {
    QColor background_color(property_bg_color_->getColor());
    background_color.setAlpha(255 * property_bg_alpha_->getFloat());
    hud_ = buffer.getQImage(*overlay_);
    hud_.fill(background_color);
  }

  {
    overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
    overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  }

  const int font_size_ = property_font_size_->getInt();
  const int line_width = property_line_thick_->getInt();

  QPainter painter(&hud_);

  {
    QColor fg_color_(property_fg_color_->getColor());
    fg_color_.setAlpha(255 * property_fg_alpha_->getFloat());
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, line_width, Qt::SolidLine));
  }

  {
    QFont font("Liberation Sans");
    font.setPointSize(font_size_);
    font.setBold(true);
    painter.setFont(font);
  }

  const int caption_offset = painter.fontMetrics().height();
  const int w = overlay_->getTextureWidth();
  const int h = overlay_->getTextureHeight() - caption_offset;

  // Draw topic name
  if (property_show_caption_->getBool()) {
    const std::string text = property_topic_name_->getStdString();
    painter.drawText(0, h, w, caption_offset, Qt::AlignCenter, text.c_str());
  }

  // Draw boarder
  {
    painter.drawLine(0, 0, 0, h);
    painter.drawLine(0, h, w, h);
    painter.drawLine(w, h, w, 0);
    painter.drawLine(w, 0, 0, 0);
  }

  const double margined_max_value = max_value_ + (max_value_ - min_value_) / 2;
  const double margined_min_value = min_value_ - (max_value_ - min_value_) / 2;

  // Plot graph
  for (size_t i = 1; i < data_buffer_.size(); i++) {
    double v_prev =
      (margined_max_value - data_buffer_[i - 1]) / (margined_max_value - margined_min_value);
    double v = (margined_max_value - data_buffer_[i]) / (margined_max_value - margined_min_value);
    double u_prev = (i - 1) / (float)data_buffer_.size();
    double u = i / (float)data_buffer_.size();

    // Clamp
    v_prev = std::clamp(v_prev, 0.0, 1.0);
    u_prev = std::clamp(u_prev, 0.0, 1.0);
    v = std::clamp(v, 0.0, 1.0);
    u = std::clamp(u, 0.0, 1.0);

    uint16_t x_prev = (int)(u_prev * w);
    uint16_t x = (int)(u * w);
    uint16_t y_prev = (int)(v_prev * h);
    uint16_t y = (int)(v * h);
    painter.drawLine(x_prev, y_prev, x, y);
  }

  // Draw value
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << data_buffer_.back();

    QFont font = painter.font();
    font.setPointSize(w / ss.str().size());
    font.setBold(true);
    painter.setFont(font);
    painter.drawText(0, 0, w, h, Qt::AlignCenter, ss.str().c_str());
  }

  painter.end();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Plotter2dOverlayDisplay, rviz_common::Display)
