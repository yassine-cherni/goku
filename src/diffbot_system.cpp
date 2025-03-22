#include "goku/diffbot_system.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace goku
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_name_ = info_.hardware_parameters.at("serial_port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  ticks_per_rev_ = std::stod(info_.hardware_parameters.at("ticks_per_rev"));
  wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));

  try
  {
    serial_port_.setPort(serial_port_name_);
    serial_port_.setBaudrate(baud_rate_);
    serial_port_.setTimeout(serial::Timeout::simpleTimeout(100));
    serial_port_.open();
    RCLCPP_INFO(get_logger(), "Connected to STM32F4 on %s at %d baud", 
                serial_port_name_.c_str(), baud_rate_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(get_logger(), "Failed to connect to STM32F4: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has invalid command interface.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2 || 
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION || 
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has invalid state interfaces.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  last_read_time_ = rclcpp::Clock().now().seconds();
  prev_positions_ = {0.0, 0.0};  // Left, right
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring hardware...");
  for (const auto & [name, descr] : joint_state_interfaces_)
    set_state(name, 0.0);
  for (const auto & [name, descr] : joint_command_interfaces_)
    set_command(name, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating STM32F4...");
  try
  {
    serial_port_.write("START\n");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send START command: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating STM32F4...");
  try
  {
    serial_port_.write("STOP\n");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send STOP command: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::string response;
  try
  {
    response = serial_port_.readline();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to read from STM32F4: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  double left_ticks, right_ticks, left_vel, right_vel;
  if (!parse_sensor_data(response, left_ticks, right_ticks, left_vel, right_vel))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to parse STM32F4 data: %s", response.c_str());
    return hardware_interface::return_type::ERROR;
  }

  double ticks_to_rad = (2.0 * M_PI) / ticks_per_rev_;
  double now = time.seconds();
  double dt = now - last_read_time_;
  last_read_time_ = now;

  double left_pos = left_ticks * ticks_to_rad;
  double right_pos = right_ticks * ticks_to_rad;

  // Fallback velocity calculation if STM32F4 data is unreliable
  if (std::isnan(left_vel) || std::isnan(right_vel) || dt <= 0.0)
  {
    left_vel = (left_pos - prev_positions_[0]) / dt;
    right_vel = (right_pos - prev_positions_[1]) / dt;
  }

  set_state("left_front_wheel_joint/position", left_pos);
  set_state("left_front_wheel_joint/velocity", left_vel);
  set_state("right_front_wheel_joint/position", right_pos);
  set_state("right_front_wheel_joint/velocity", right_vel);
  set_state("left_rear_wheel_joint/position", left_pos);
  set_state("left_rear_wheel_joint/velocity", left_vel);
  set_state("right_rear_wheel_joint/position", right_pos);
  set_state("right_rear_wheel_joint/velocity", right_vel);

  prev_positions_[0] = left_pos;
  prev_positions_[1] = right_pos;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  double left_vel_cmd = get_command("left_front_wheel_joint/velocity");
  double right_vel_cmd = get_command("right_front_wheel_joint/velocity");

  const double MAX_VEL = 2.0; // rad/s
  int left_cmd = static_cast<int>((left_vel_cmd / MAX_VEL) * 1000);
  int right_cmd = static_cast<int>((right_vel_cmd / MAX_VEL) * 1000);
  left_cmd = std::max(-1000, std::min(1000, left_cmd));
  right_cmd = std::max(-1000, std::min(1000, right_cmd));

  std::stringstream cmd;
  cmd << "M:" << left_cmd << " " << right_cmd << "\n";
  try
  {
    serial_port_.write(cmd.str());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to write to STM32F4: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool DiffBotSystemHardware::parse_sensor_data(const std::string & data, double & left_ticks, 
                                             double & right_ticks, double & left_vel, 
                                             double & right_vel)
{
  std::istringstream iss(data);
  std::string token;
  try
  {
    std::getline(iss, token, ' '); // "L:1234"
    left_ticks = std::stod(token.substr(2));
    std::getline(iss, token, ' '); // "R:5678"
    right_ticks = std::stod(token.substr(2));
    std::getline(iss, token, ' '); // "V:1.2"
    left_vel = std::stod(token.substr(2));
    std::getline(iss, token);      // "-1.3"
    right_vel = std::stod(token);
    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Parsing error: %s", e.what());
    return false;
  }
}

}  // namespace goku

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(goku::DiffBotSystemHardware, hardware_interface::SystemInterface)
