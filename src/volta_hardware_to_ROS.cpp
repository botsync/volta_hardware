#include "rclcpp/rclcpp.hpp"
#include "volta_msgs/msg/rpm.hpp"
#include "volta_msgs/srv/rpm.hpp"

static float subMotorRPMROSRight, subMotorRPMROSLeft;

void rpmROSCallback(const volta_msgs::msg::RPM::SharedPtr& rpmTemp)
{
  subMotorRPMROSRight	= (float) rpmTemp->right/24.0;
  subMotorRPMROSLeft	= (float) rpmTemp->left/24.0;
}

void RPM_query(const std::shared_ptr<volta_msgs::srv::Rpm::Request> request, 
                      std::shared_ptr<volta_msgs::srv::Rpm::Response> response)
{
  response->left=subMotorRPMROSLeft;
  response->right=subMotorRPMROSRight;
}

int main(int argc, char *argv[]) 

{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_=rclcpp::Node::make_shared("volta_hardware_to_ROS_node");

    rclcpp::Subscription<volta_msgs::msg::RPM>::SharedPtr rpm_sub_ROS=
    node_->create_subscription<volta_msgs::msg::RPM>("rpm_sub", 100, [](volta_msgs::msg::RPM::SharedPtr msg){rpmROSCallback(msg);});

    rclcpp::Service<volta_msgs::srv::Rpm>::SharedPtr RPM_service = 
    node_->create_service<volta_msgs::srv::Rpm>("volta_rpm_service", &RPM_query);

    rclcpp::spin(node_);
    rclcpp::shutdown();
   
    return 0;

}