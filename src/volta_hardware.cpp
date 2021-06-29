#include <volta_hardware/volta_hardware.h>
#include <boost/assign/list_of.hpp>
#include <urdf/model.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    volta_hardware::voltaHardware,
    hardware_interface::SystemInterface
)
namespace volta_hardware {

voltaHardware::voltaHardware() {

    this->node_ = rclcpp::Node::make_shared("volta_hardware_interface");
    
    this->max_rpm_ = MAX_RPM;
    // Publish values from base_controller to rpm_pub
    this->rpm_pub = this->node_->create_publisher<volta_msgs::msg::RPM>("rpm_pub", 100);
    
    // Client node to read RPM values from server
    this->rpm_query_client = this->node_->create_client<volta_msgs::srv::Rpm>("volta_rpm_service");
    
    while(!rpm_query_client->wait_for_service())
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->node_->get_logger(), "Interrupted while waiting for service. Exiting!!");
        }
        RCLCPP_INFO(this->node_->get_logger(), "Volta RPM service not available, waiting again....");
    }
    
    //Load the hardware 
    RCLCPP_INFO(this->node_->get_logger(), "Loading volta hardware");

}

hardware_interface::return_type voltaHardware::configure(const hardware_interface::HardwareInfo & volta_info)
{
    
    if (configure_default(volta_info) != hardware_interface::return_type::OK)
    {
        return hardware_interface::return_type::ERROR;
    }

    this->motor_ids_.resize(this->info_.joints.size(), std::numeric_limits<uint8_t>::quiet_NaN());
    this->joint_names_.resize(this->info_.joints.size(), std::numeric_limits<std::string>::quiet_NaN());
    this->hw_positions_.resize(this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    this->hw_velocities_.resize(this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    this->hw_commands_.resize(this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (hardware_interface::ComponentInfo &joint : this->info_.joints)
    {
        if (joint.parameters["motor_id"].empty()) {
            RCLCPP_FATAL(this->node_->get_logger(), "Motor id not defined for joint %s", joint.name.c_str());
            return hardware_interface::return_type::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(this->node_->get_logger(), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(this->node_->get_logger(), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(this->node_->get_logger(), "Invalid number of state interfaces (expected: 2)");
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(this->node_->get_logger(), "Invalid joint state interface 0 type (expected: position)");
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(this->node_->get_logger(), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::return_type::ERROR;
        }
    }

    for (size_t i = 0; i < this->info_.joints.size(); i++) 
    {
        this->motor_ids_[i] = (uint8_t)std::stoi(this->info_.joints[i].parameters["motor_id"]);
        RCLCPP_INFO(this->node_->get_logger(), "%s mapped to motor %d", this->info_.joints[i].name.c_str(), this->motor_ids_[i]);
    }

    this->status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;

}

std::vector<hardware_interface::StateInterface> voltaHardware::export_state_interfaces()
{
    RCLCPP_INFO(this->node_->get_logger(), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < this->info_.joints.size(); i++) {

        RCLCPP_INFO(this->node_->get_logger(), "Adding position state interface: %s", this->info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                this->info_.joints[i].name, hardware_interface::HW_IF_POSITION, &(this->hw_positions_[i])
            )
        );

        RCLCPP_INFO(this->node_->get_logger(), "Adding velocity state interface: %s", this->info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                this->info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &(this->hw_velocities_[i])
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> voltaHardware::export_command_interfaces()
{
    RCLCPP_INFO(this->node_->get_logger(), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < this->info_.joints.size(); i++) {
        RCLCPP_INFO(this->node_->get_logger(), "Adding velocity command interface: %s", this->info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                this->info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &(this->hw_commands_[i])
            )
        );
    }
    return command_interfaces;
}

hardware_interface::return_type voltaHardware::start()
{
  RCLCPP_INFO(this->node_->get_logger(), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(this->node_->get_logger(), "Volta Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type voltaHardware::stop()
{
  RCLCPP_INFO(this->node_->get_logger(), "Stopping Volta...please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(this->node_->get_logger(), "Volta successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type voltaHardware::read()
{
    float rpm_left, rpm_right;
    
    auto request = std::make_shared<volta_msgs::srv::Rpm::Request>();

    auto result = this->rpm_query_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result)==rclcpp::FutureReturnCode::SUCCESS)
    {
        //RCLCPP_INFO(this->node_->get_logger(), "RPM queried successfully");
    }

    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Unable to query RPM");
    } 

    rpm_left 	=	(float) result.get()->left;
    rpm_right 	=	(float) result.get()->right;

    double left, right;
    left = this->convert_rpm_to_radians(rpm_left);
    right = this->convert_rpm_to_radians(rpm_right);

    for (size_t i=0; i < this->info_.joints.size(); i++) {
        if (i%2 == 0) {
            this->hw_velocities_[i] = left;
        } else {
            this->hw_velocities_[i] = right;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type voltaHardware::write()
{
   volta_msgs::msg::RPM rpm;
   float rpm_left, rpm_right;

   double left = this->hw_commands_[0];
   double right = this->hw_commands_[1];

   rpm_left = convert_radians_to_rpm(left);
   rpm_right = convert_radians_to_rpm(right);

   this->limit_speeds(rpm_left, rpm_right);

   rpm.right= rpm_right * 24.0;
   rpm.left = rpm_left * 24.0;

   this->rpm_pub->publish(rpm);

   return hardware_interface::return_type::OK;
}

void voltaHardware::set_speeds(float left, float right) {
    float  rpm_left, rpm_right;

    rpm_left = convert_radians_to_rpm(left);
    rpm_right = convert_radians_to_rpm(right);

    this->limit_speeds(rpm_left, rpm_right);
}

void voltaHardware::limit_speeds(float &left, float &right) {
    int16_t temp_max = std::max(std::abs(left), std::abs(right));
    if (temp_max > this->max_rpm_) {
        left *= (float)(this->max_rpm_) / (float)temp_max;
        right *= (float)(this->max_rpm_) / (float)temp_max;
    }
}

double voltaHardware::convert_rpm_to_radians(float rpm) {
    return (double)((rpm*2.0*PI)/(60.0));
}

double voltaHardware::convert_radians_to_rpm(float radians) {
    double ret= (double)(radians*60.0)/(2.0*PI);
    return ret;
}

}
