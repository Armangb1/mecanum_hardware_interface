#ifndef UDP_HARDWARE_INTERFACE_HPP
#define UDP_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "std_msgs/msg/float32_multi_array.hpp"


using namespace hardware_interface;

namespace mecanum_hardware_interface
{
class MicroRosInterface : public SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;


  std::vector<CommandInterface> export_command_interfaces() override;
  std::vector<StateInterface> export_state_interfaces() override;


  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:

  // Motor data
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  
  // IMU data (ax, ay, az, gx, gy, gz)
  std::vector<double> imu_data_;
  
  // Node pointer for ROS communication
  std::shared_ptr<rclcpp::Node> node_;
  
  // Publishers and subscribers
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
  
  // Callback for receiving state data from micro-ROS
  void state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  void imu_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  // Thread safety
  std::mutex state_mutex_;
};

} // namespace mecanum_hw_interface


#endif