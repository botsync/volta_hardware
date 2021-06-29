
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "constants.h"
#include "volta_msgs/msg/table.hpp"
#include "volta_msgs/msg/rpm.hpp"
#include "volta_msgs/msg/bms.hpp"

extern rclcpp::Publisher<volta_msgs::msg::RPM>::SharedPtr rpm_pub;
extern rclcpp::Subscription<volta_msgs::msg::RPM>::SharedPtr rpm_sub;
extern rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub;
extern rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr diagEn_sub;

extern volta_msgs::msg::Table table_msg;
extern volta_msgs::msg::BMS BMS_msg;

extern rclcpp::Publisher<volta_msgs::msg::Table>::SharedPtr table_pub;
extern rclcpp::Subscription<volta_msgs::msg::Table>::SharedPtr  publishTable;

extern rclcpp::Publisher<volta_msgs::msg::BMS>::SharedPtr bms_pub;


void rpmCallback(const volta_msgs::msg::RPM::SharedPtr& rpmTemp);
void estopCallback(const std_msgs::msg::Bool::SharedPtr& e_stop_msg);
void diagEnCallback(const std_msgs::msg::Bool& diagEn_msg);
void rosTopicInit(std::shared_ptr<rclcpp::Node> node);
void volta_update_RPM(float left, float right);
void publish_table(void);
void publish_bms(void);