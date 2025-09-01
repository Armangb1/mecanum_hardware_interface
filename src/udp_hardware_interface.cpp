#include "udp_hardware_interface/udp_hardware_interface.hpp"

namespace mecanum_udp_hw
{

  CallbackReturn UdpHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    
    // 4 motors
    hw_commands_.assign(4, 0.0);
    hw_positions_.assign(4, 0.0);
    hw_velocities_.assign(4, 0.0);

    // Simple IMU (roll, pitch, yaw or ax,ay,az)
    imu_data_.assign(6, 0.0);

    // Setup UDP sockets
    command_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (command_sock_ < 0) return CallbackReturn::ERROR;

    memset(&command_addr_, 0, sizeof(command_addr_));
    command_addr_.sin_family = AF_INET;
    command_addr_.sin_port = htons(8888);
    inet_pton(AF_INET, "192.168.1.2", &command_addr_.sin_addr);

    state_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (state_sock_ < 0) return CallbackReturn::ERROR;

    sockaddr_in state_addr;
    memset(&state_addr, 0, sizeof(state_addr));
    state_addr.sin_family = AF_INET;
    state_addr.sin_port = htons(8889);
    state_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(state_sock_, (struct sockaddr*)&state_addr, sizeof(state_addr)) < 0) return CallbackReturn::ERROR;

    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> UdpHardware::export_state_interfaces()
  {
    std::vector<StateInterface> state_interfaces;
    for (size_t i = 0; i < hw_positions_.size(); i++) {
      state_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION, &hw_positions_[i]);
      state_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    // IMU example (not tied to a joint)
    for (size_t i = 0; i < imu_data_.size(); i++) {
      state_interfaces.emplace_back("imu", "imu_" + std::to_string(i), &imu_data_[i]);
    }
    return state_interfaces;
  }

  std::vector<CommandInterface> UdpHardware::export_command_interfaces()
  {
    std::vector<CommandInterface> command_interfaces;
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      command_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &hw_commands_[i]);
    }
    return command_interfaces;
  }

  return_type UdpHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    char buffer[1024];
    int n = recv(state_sock_, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (n > 0) {
      // Example: assume payload is 4 encoder doubles + 6 imu doubles
      if (n >= (int)((4+6) * sizeof(double))) {
        double *data = reinterpret_cast<double*>(buffer);
        for (int i = 0; i < 4; i++) {
          hw_positions_[i] = data[i];
        }
        for (int i = 0; i < 6; i++) {
          imu_data_[i] = data[4+i];
        }
      }
    }
    // TODO: use set state
    return return_type::OK;
  }

  return_type UdpHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // TODO: use get command
    // Send 4 doubles for motor commands
    sendto(command_sock_, hw_commands_.data(), hw_commands_.size() * sizeof(double),
           0, (struct sockaddr*)&command_addr_, sizeof(command_addr_));
    return return_type::OK;
  }

}; // namespace mecanum_udp_hw

PLUGINLIB_EXPORT_CLASS(mecanum_udp_hw::UdpHardware, hardware_interface::SystemInterface)
