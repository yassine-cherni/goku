#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spidev.h>
#include <vector>
#include <string>

class DiffDriveHWInterface : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // Initialize SPI
    spi_fd_ = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHWInterface"), "Failed to open SPI device");
      return CallbackReturn::ERROR;
    }
    uint8_t mode = SPI_MODE_0;
    uint32_t speed = 1000000; // 1 MHz
    ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    // Joints and states
    wheel_l_vel_ = 0.0;
    wheel_r_vel_ = 0.0;
    wheel_l_pos_ = 0.0;
    wheel_r_pos_ = 0.0;

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("wheel_left_joint", "position", &wheel_l_pos_);
    state_interfaces.emplace_back("wheel_right_joint", "position", &wheel_r_pos_);
    state_interfaces.emplace_back("wheel_left_joint", "velocity", &wheel_l_vel_);
    state_interfaces.emplace_back("wheel_right_joint", "velocity", &wheel_r_vel_);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back("wheel_left_joint", "velocity", &wheel_l_vel_cmd_);
    command_interfaces.emplace_back("wheel_right_joint", "velocity", &wheel_r_vel_cmd_);
    return command_interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    // Read encoder data from STM32F4 via SPI
    uint8_t tx_data[4] = {0xFF, 0xFF, 0xFF, 0xFF}; // Dummy data to trigger response
    uint8_t rx_data[4] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_data,
        .rx_buf = (unsigned long)rx_data,
        .len = 4,
        .speed_hz = 1000000,
        .bits_per_word = 8,
    };
    ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);

    // Assume rx_data[0,1] = left encoder, rx_data[2,3] = right encoder
    wheel_l_pos_ = (rx_data[0] << 8) | rx_data[1]; // Example decoding
    wheel_r_pos_ = (rx_data[2] << 8) | rx_data[3];
    // Compute velocities (simplified, use time delta in practice)
    wheel_l_vel_ = (wheel_l_pos_ - prev_l_pos_) * 10.0; // 10 Hz update rate
    wheel_r_vel_ = (wheel_r_pos_ - prev_r_pos_) * 10.0;
    prev_l_pos_ = wheel_l_pos_;
    prev_r_pos_ = wheel_r_pos_;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    // Send velocity commands to STM32F4 via SPI
    uint8_t tx_data[4];
    int16_t left_vel = static_cast<int16_t>(wheel_l_vel_cmd_ * 100); // Scale as needed
    int16_t right_vel = static_cast<int16_t>(wheel_r_vel_cmd_ * 100);
    tx_data[0] = (left_vel >> 8) & 0xFF;
    tx_data[1] = left_vel & 0xFF;
    tx_data[2] = (right_vel >> 8) & 0xFF;
    tx_data[3] = right_vel & 0xFF;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_data,
        .rx_buf = 0,
        .len = 4,
        .speed_hz = 1000000,
        .bits_per_word = 8,
    };
    ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);

    return hardware_interface::return_type::OK;
  }

private:
  int spi_fd_;
  double wheel_l_vel_, wheel_r_vel_, wheel_l_pos_, wheel_r_pos_;
  double wheel_l_vel_cmd_ = 0.0, wheel_r_vel_cmd_ = 0.0;
  double prev_l_pos_ = 0.0, prev_r_pos_ = 0.0;
};

#include <hardware_interface/resource_manager.hpp>
EXPORT_PLUGIN(DiffDriveHWInterface, "diff_drive_hw_interface", "DiffDriveHWInterface")
