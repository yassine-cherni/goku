#ifndef GOKU__DIFFBOT_SYSTEM_HPP_
#define GOKU__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <serial/serial.h>

namespace goku
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string serial_port_name_;
  int baud_rate_;
  serial::Serial serial_port_;
  double ticks_per_rev_;
  double wheel_radius_;
  double last_read_time_;
  std::vector<double> prev_positions_;  // For velocity calculation fallback

  bool parse_sensor_data(const std::string & data, double & left_ticks, double & right_ticks, 
                         double & left_vel, double & right_vel);
};
}  // namespace goku

#endif  // GOKU__DIFFBOT_SYSTEM_HPP_
