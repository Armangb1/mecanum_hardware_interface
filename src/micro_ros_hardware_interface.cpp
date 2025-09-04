#include "mecanum_hardware_interface/micro_ros_hardware_interface.hpp"

namespace mecanum_hardware_interface
{

  CallbackReturn MicroRosInterface::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("MicroRosInterface"), "Initializing MicroROS Hardware Interface...");
    
    // Validate joint configuration
    if (info_.joints.size() != 4) {
      RCLCPP_ERROR(rclcpp::get_logger("MicroRosInterface"), 
                  "Expected 4 joints, got %zu", info_.joints.size());
      return CallbackReturn::ERROR;
    }
    
    // Initialize motor data (4 motors for mecanum drive)
    hw_commands_.assign(4, 0.0);
    hw_positions_.assign(4, 0.0);
    hw_velocities_.assign(4, 0.0);

    // Initialize IMU data (ax, ay, az, gx, gy, gz)
    imu_data_.assign(6, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("MicroRosInterface"), 
                "Initialized with %zu joints and %zu IMU values", 
                info_.joints.size(), imu_data_.size());

    return CallbackReturn::SUCCESS;
  }
 


  CallbackReturn MicroRosInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MicroRosInterface"), "Configuring MicroROS Hardware Interface...");
  
  // Get the node from the resource manager
  node_ = get_node();
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("MicroRosInterface"), "Failed to get node");
    return CallbackReturn::ERROR;
  }

  // Create publisher for sending commands to micro-ROS
  command_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "voltage_cmds", rclcpp::QoS(10).reliable());
  
  // Create subscriber for receiving state from micro-ROS
  state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "encoder", rclcpp::SensorDataQoS(),
    std::bind(&MicroRosInterface::state_callback, this, std::placeholders::_1));

  imu_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&MicroRosInterface::imu_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("MicroRosInterface"), "Successfully configured publishers and subscribers");

  return CallbackReturn::SUCCESS;
}

CallbackReturn MicroRosInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MicroRosInterface"), "Activating MicroROS Hardware Interface...");
  // 
  // Reset all commands to zero on activation
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  // 
  return CallbackReturn::SUCCESS;
}

CallbackReturn MicroRosInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MicroRosInterface"), "Deactivating MicroROS Hardware Interface...");
  
  // Send zero commands to stop motors
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  
  // Send final zero command
  std_msgs::msg::Float32MultiArray msg;
  msg.data.assign(4, 0);
  command_pub_->publish(msg);
  
  return CallbackReturn::SUCCESS;
}


  std::vector<StateInterface> MicroRosInterface::export_state_interfaces()
  {
    std::vector<StateInterface> state_interfaces;
    
    // Export joint state interfaces
    for (size_t i = 0; i < info_.joints.size(); i++) {
      state_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION, &hw_positions_[i]);
      state_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    
    // Export IMU state interfaces
    std::vector<std::string> imu_names = {"linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z", "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};
    for (size_t i = 0; i < imu_data_.size(); i++) {
      state_interfaces.emplace_back(info_.sensors[0].name, imu_names[i], &imu_data_[i]);
    }
    
    
    RCLCPP_INFO(rclcpp::get_logger("MicroRosInterface"), 
                "Exported %zu state interfaces", state_interfaces.size());
    
    return state_interfaces;
  }

  std::vector<CommandInterface> MicroRosInterface::export_command_interfaces()
  {
    std::vector<CommandInterface> command_interfaces;
    
    // Export voltage command interfaces for each wheel
    for (size_t i = 0; i < info_.joints.size(); i++) {
      command_interfaces.emplace_back(info_.joints[i].name, "voltage", &hw_commands_[i]);
    }
    
    RCLCPP_INFO(rclcpp::get_logger("MicroRosInterface"), 
                "Exported %zu command interfaces", command_interfaces.size());
    
    return command_interfaces;
  }



  return_type MicroRosInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // State is updated via callback, no additional reading needed here
    // In a real implementation, you might add timeout checks here
    return return_type::OK;
  }

  return_type MicroRosInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Convert velocity commands to motor values and send to micro-ROS
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(4);
    
    // Convert double commands to float32 (scale factor depends on your motor driver)
    // This is a simple example - adjust scaling as needed
    const double scale_factor = 1.0; // Adjust based on your system
    
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      // Clamp to float32 range
      double scaled_cmd = hw_commands_[i] * scale_factor;
      scaled_cmd = std::max(-24.0, std::min(24.0, scaled_cmd));
      msg.data[i] = static_cast<float>(scaled_cmd);
    }
    
    // Publish the command
    command_pub_->publish(msg);
    
    return return_type::OK;
  }


  void MicroRosInterface::state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Update encoder positions (assuming first 4 values are encoder data)
  
  for (size_t i = 0; i < 4 && i < hw_positions_.size(); i++) {
    // Convert encoder ticks to position (adjust scaling as needed)
    hw_positions_[i] = static_cast<double>(msg->data[i]);
    // You might want to calculate velocity from position changes here
    // set_state(info_.joints[i].name + "/position", hw_positions_[i]);
  }
  

  for (size_t i = 0; i < 4 && i < hw_velocities_.size(); i++) {
    // Convert encoder ticks to position (adjust scaling as needed)
    hw_velocities_[i] = static_cast<double>(msg->data[i+4]);
    // set_state(info_.joints[i].name + "/velocity", hw_velocities_[i]);
    // You might want to calculate velocity from position changes here
  }

}

void MicroRosInterface::imu_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Update IMU data (assuming 6 values: ax, ay, az, gx, gy, gz)
  for (size_t i = 0; i < 6 && i < imu_data_.size(); i++) {
    imu_data_[i] = static_cast<double>(msg->data[i]);
  }
}

}; // namespace mecanum_hardware_interface
PLUGINLIB_EXPORT_CLASS(mecanum_hardware_interface::MicroRosInterface, hardware_interface::SystemInterface)
