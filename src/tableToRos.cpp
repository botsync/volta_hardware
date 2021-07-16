#include <rclcpp/rclcpp.hpp>
#include "volta_hardware/tableToRos.h"
#include "volta_hardware/conversion.h"
#include <iostream>
#include <fstream>
#include "volta_hardware/voltaDataStruct.h"

uint8_t rpmData=false;
uint8_t estopData=false;
uint8_t estopStatus=false;
uint8_t diagEnData=false;
uint8_t diagEnStatus= false;

extern float subMotorRPMRight;
extern float subMotorRPMLeft;
extern uint8_t rpmAvailable;

rclcpp::Publisher<volta_msgs::msg::RPM>::SharedPtr rpm_pub;
rclcpp::Subscription<volta_msgs::msg::RPM>::SharedPtr rpm_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr diagEn_sub;
volta_msgs::msg::Table table_msg;
volta_msgs::msg::BMS BMS_msg;

rclcpp::Publisher<volta_msgs::msg::Table>::SharedPtr table_pub;
rclcpp::Subscription<volta_msgs::msg::Table>::SharedPtr  publishTable;

rclcpp::Publisher<volta_msgs::msg::BMS>::SharedPtr bms_pub;

void rpmCallback(const volta_msgs::msg::RPM::SharedPtr& rpmTemp) {
	subMotorRPMLeft= rpmTemp->left;
	subMotorRPMRight= rpmTemp->right;
	rpmAvailable = true;

}
void estopCallback(const std_msgs::msg::Bool::SharedPtr& e_stop_msg)
{
	RCLCPP_INFO(rclcpp::get_logger("tableTORos"), "Estop called");
	estopData=true;
	estopStatus = e_stop_msg->data;
	volta_update_table(PRIORITY_DIAG,DIAG_ROS_ESTOP_STATE,&estopStatus,1);
}
void diagEnCallback(const std_msgs::msg::Bool::SharedPtr& diagEn_msg)
{
	RCLCPP_INFO(rclcpp::get_logger("tableTORos"), "Diagnostics enabled");
	if(diagEn_msg->data == true)
	{
		diagEnStatus = 1;
	}
	else
	{
		diagEnStatus = 0;
	}
	volta_update_table(PRIORITY_DIAG,DIAG_EN,&diagEnStatus,1);
	RCLCPP_INFO(rclcpp::get_logger("tableTORos"), "diag_status.en =%x",diag_status.en);

}


void rosTopicInit(std::shared_ptr<rclcpp::Node>node)
{
	rpm_sub=node->create_subscription<volta_msgs::msg::RPM>("rpm_pub", 100, [](volta_msgs::msg::RPM::SharedPtr msg){rpmCallback(msg);});

	rpm_pub=node->create_publisher<volta_msgs::msg::RPM>("rpm_sub", 100);

	estop_sub=node->create_subscription<std_msgs::msg::Bool>("e_stop_sw_enable", 100, [](std_msgs::msg::Bool::SharedPtr msg){estopCallback(msg);});

	diagEn_sub=node->create_subscription<std_msgs::msg::Bool>("diag_enable", 100, [](std_msgs::msg::Bool::SharedPtr msg){diagEnCallback(msg);});

	table_pub=node->create_publisher<volta_msgs::msg::Table>("diag", 100);

	bms_pub=node->create_publisher<volta_msgs::msg::BMS>("BMS_volta",100);
}

void volta_update_RPM(float left, float right)
{
	volta_msgs::msg::RPM rpm;
	rpm.left = left;
	rpm.right = right;
	rpm_pub->publish(rpm);
}

void publish_table(void)
{

	table_msg.soc = diag_status.soc;
	table_msg.soh = diag_status.soh;
	table_msg.current= diag_status.current;
	table_msg.voltage= diag_status.voltage;
	table_msg.system_status= diag_status.system_status;
	table_msg.relay_temp= diag_status.relay_temp;
	table_msg.powe_in_temp= diag_status.powe_in_temp;

	table_msg.motor_volt= diag_status.motor_volt;
	table_msg.battery_volt= diag_status.battery_volt;
	table_msg.hw_estop_volt= diag_status.hw_estop_volt;
	table_msg.motor_alarm= diag_status.motorAlarm;
	table_msg.diag_en= diag_status.en;
	table_msg.hw_estop_state= diag_status.hw_Estop_state;
	table_msg.sw_estop_state= diag_status.sw_Estop_state;
	table_msg.ros_estop_state= diag_status.ros_Estop_state;
	table_pub->publish(table_msg);

}
void publish_bms(void)
{
	BMS_msg.soc = bms_status.soc ;
	BMS_msg.soh= bms_status.soh;
	BMS_msg.current= bms_status.current;
	BMS_msg.voltage= bms_status.voltage;
	bms_pub->publish(BMS_msg);
}
