#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/poll.h>
#include <cstring>
#include <memory>

class DiffDriveHWInterface : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHWInterface"), "Failed to initialize base class");
      return CallbackReturn::ERROR;
    }

    // Open SPI device
    spi_fd_ = open("/dev/spidev0.0", O_RDWR | O_NONBLOCK);
    if (spi_fd_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHWInterface"), "Failed to open SPI device: %s", strerror(errno));
      return CallbackReturn::ERROR;
    }

    // Configure SPI
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 1000000; // 1 MHz
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHWInterface"), "Failed to configure SPI: %s", strerror(errno));
      close(spi_fd_);
      return CallbackReturn::ERROR;
    }

    // Initialize state variables
    wheel_l_vel_ = 0.0;
    wheel_r_vel_ = 0.0;
    wheel_l_pos_ = 0.0;
    wheel_r_pos_ = 0.0;
    wheel_l_vel_cmd_ = 0.0;
    wheel_r_vel_cmd_ = 0.0;
    prev_l_pos_ = 0.0;
    prev_r_pos_ = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHWInterface"), "SPI initialized successfully");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &) override {
    if (spi_fd_ >= 0) {
      close(spi_fd_);
      spi_fd_ = -1;
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveHWInterface"), "SPI closed");
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface("wheel_left_joint", "position", &wheel_l_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("wheel_right_joint", "position", &wheel_r_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("wheel_left_joint", "velocity", &wheel_l_vel_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("wheel_right_joint", "velocity", &wheel_r_vel_));
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface("wheel_left_joint", "velocity", &wheel_l_vel_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface("wheel_right_joint", "velocity", &wheel_r_vel_cmd_));
    return command_interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    static rclcpp::Time last_time = time;
    double elapsed = (time - last_time).seconds();
    if (elapsed > 0.011) { // 10% tolerance over 10ms (100 Hz)
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DiffDriveHWInterface"), *rclcpp::get_clock(), 1000, 
                           "Read deadline missed: %f s", elapsed);
    }
    last_time = time;

    struct pollfd pfd = {spi_fd_, POLLIN, 0};
    int ret = poll(&pfd, 1, 1); // 1ms timeout
    if (ret < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHWInterface"), "SPI poll failed: %s", strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
    if (ret > 0 && (pfd.revents & POLLIN)) {
      uint8_t tx_data[4] = {0xFF, 0xFF, 0xFF, 0xFF}; // Dummy data to trigger response
      uint8_t rx_data[4] = {0};
      struct spi_ioc_transfer tr = {
          .tx_buf = (unsigned long)tx_data,
          .rx_buf = (unsigned long)rx_data,
          .len = sizeof(rx_data),
          .speed_hz = 1000000,
          .bits_per_word = 8,
      };
      if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHWInterface"), "SPI read failed: %s", strerror(errno));
        return hardware_interface::return_type::ERROR;
      }
      wheel_l_pos_ = static_cast<double>((rx_data[0] << 8) | rx_data[1]);
      wheel_r_pos_ = static_cast<double>((rx_data[2] << 8) | rx_data[3]);
      wheel_l_vel_ = (wheel_l_pos_ - prev_l_pos_) / (elapsed > 0 ? elapsed : 0.01); // Avoid division by zero
      wheel_r_vel_ = (wheel_r_pos_ - prev_r_pos_) / (elapsed > 0 ? elapsed : 0.01);
      prev_l_pos_ = wheel_l_pos_;
      prev_r_pos_ = wheel_r_pos_;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    static rclcpp::Time last_time = time;
    double elapsed = (time - last_time).seconds();
    if (elapsed > 0.011) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DiffDriveHWInterface"), *rclcpp::get_clock(), 1000, 
                           "Write deadline missed: %f s", elapsed);
    }
    last_time = time;

    uint8_t tx_data[4];
    int16_t left_vel = static_cast<int16_t>(wheel_l_vel_cmd_ * 100.0); // Scale to match STM32
    int16_t right_vel = static_cast<int16_t>(wheel_r_vel_cmd_ * 100.0);
    tx_data[0] = (left_vel >> 8) & 0xFF;
    tx_data[1] = left_vel & 0xFF;
    tx_data[2] = (right_vel >> 8) & 0xFF;
    tx_data[3] = right_vel & 0xFF;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_data,
        .rx_buf = 0,
        .len = sizeof(tx_data),
        .speed_hz = 1000000,
        .bits_per_word = 8,
    };
    if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHWInterface"), "SPI write failed: %s", strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

private:
  int spi_fd_ = -1;
  double wheel_l_vel_ = 0.0, wheel_r_vel_ = 0.0;
  double wheel_l_pos_ = 0. Search term0, wheel_r_pos_ = 0.0;
  double wheel_l_vel_cmd_ = 0.0, wheel_r_vel_cmd_ = 0.0;
  double prev_l_pos_ = 0.0, prev_r_pos_ = 0.0;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(DiffDriveHWInterface, hardware_interface::SystemInterface)
