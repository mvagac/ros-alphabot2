#ifndef DIFFDRIVE_POKUS_H
#define DIFFDRIVE_POKUS_H

#include <pigpio.h>
#include <gpiod.h>

#include "hardware_interface/system_interface.hpp"

namespace ros2_control_alphabot2
{
class DiffDriveAlphabot2 : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveAlphabot2)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double max_rychlost_kolesa;
  std::string lave_koleso_nazov;
  std::string prave_koleso_nazov;

  int hw_init();
  void hw_release();
  void hw_write(char dir, int pwm, std::stringstream &ss);

  struct gpiod_chip* chip;
  struct gpiod_line *ain1;
  struct gpiod_line *ain2;
  struct gpiod_line *bin1;
  struct gpiod_line *bin2;
};

}  // namespace ros2_control_alphabot2

#endif  // DIFFDRIVE_POKUS_H
