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
