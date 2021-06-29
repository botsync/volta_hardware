#ifndef VOLTA_BASE_VOLTA_HARDWARE_H
#define VOLTA_BASE_VOLTA_HARDWARE_H

#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include <controller_manager/controller_manager.hpp>


#include <string>
#include "sensor_msgs/msg/joint_state.hpp"

#include <volta_hardware/constants.h>


#include "std_msgs/msg/int16.hpp"
#include "volta_msgs/msg/rpm.hpp"

//Added service header to read encoder readings
#include "volta_msgs/srv/rpm.hpp"


namespace volta_hardware {
class voltaHardware: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>{
public:

    voltaHardware();

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo & volta_info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type start() override;
    hardware_interface::return_type stop() override;
    hardware_interface::return_type read() override;
    hardware_interface::return_type write() override;
    
private:

    const hardware_interface::HardwareInfo volta_info;

    std::vector<uint8_t>motor_ids_;
    std::vector<std::string>joint_names_;
    std::vector<double>hw_positions_;
    std::vector<double>hw_velocities_;
    std::vector<double>hw_commands_;

    rclcpp::Publisher<volta_msgs::msg::RPM>::SharedPtr rpm_pub ;

    // Client to read encoder values
    rclcpp::Client<volta_msgs::srv::Rpm>::SharedPtr rpm_query_client;

    void set_speeds(float left, float right);
    void limit_speeds(float &left,float &right);
    double convert_rpm_to_radians(float rpm);
    double convert_radians_to_rpm(float radians);

    double max_rpm_;

    rclcpp::Node::SharedPtr node_;

};

}

#endif
