// Copyright (c) 2025 Armangb1
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

#ifndef UDP_HARDWARE_INTERFACE_HPP
#define UDP_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace hardware_interface;

namespace mecanum_hardware_interface
{
class UdpHardware : public SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<CommandInterface> export_command_interfaces() override;
  std::vector<StateInterface> export_state_interfaces() override;


  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> imu_data_;

  int command_sock_;
  sockaddr_in command_addr_;
  int state_sock_;
};

} // namespace mecanum_udp_hw


#endif