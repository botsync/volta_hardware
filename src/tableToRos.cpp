#include <ros/ros.h>
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

extern int16_t subMotorRPMRight;
extern int16_t subMotorRPMLeft;
extern uint8_t rpmAvailable;

ros::Publisher rpm_pub;
ros::Subscriber rpm_sub;
ros::Subscriber estop_sub;
ros::Subscriber diagEn_sub;
ros::Publisher table_pub;
ros::Subscriber   publishTable;
volta_msgs::Table table_msg;

bool enable_publish = false;

void rpmCallback(const volta_msgs::RPM::ConstPtr& rpmTemp) {
	subMotorRPMLeft= rpmTemp->left;
	subMotorRPMRight= rpmTemp->right;
	rpmAvailable = true;

}
void estopCallback(const std_msgs::Bool& e_stop_msg)
{
	ROS_INFO("estopCallback \n");
	estopData=true;
	estopStatus = e_stop_msg.data;
	volta_update_table(PRIORITY_DIAG,DIAG_ROS_ESTOP_STATE,&estopStatus,1);
}
void diagEnCallback(const std_msgs::Bool& diagEn_msg)
{
	ROS_INFO("diagEnCallback \n");
	diagEnData=true;
	diagEnStatus =  diagEn_msg.data;
	enable_publish=diagEn_msg.data;
	volta_update_table(PRIORITY_DIAG,DIAG_EN,&diagEnStatus,1);
}


void rosTopicInit(void)
{
	ros::NodeHandle nh;
	rpm_sub 		= nh.subscribe("rpm_pub",100,&rpmCallback);
	rpm_pub 		= nh.advertise<volta_msgs::RPM>("rpm_sub", 100);
	estop_sub 		= nh.subscribe("e_stop_sw_enable",100,&estopCallback);
	diagEn_sub		= nh.subscribe("diag_enable",100,&diagEnCallback);
	table_pub 		= nh.advertise<volta_msgs::Table>("diag", 100);
}

void volta_update_RPM(int16_t left, int16_t right)
{
	volta_msgs::RPM rpm;
	rpm.left = left;
	rpm.right = right;
	rpm_pub.publish(rpm);
}

void publish_table(void)
{

	if(enable_publish)
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
		table_msg.motorAlarm= diag_status.motorAlarm;
		table_msg.diag_en= diag_status.en;
		table_msg.hw_Estop_state= diag_status.hw_Estop_state;
		table_msg.sw_Estop_state= diag_status.sw_Estop_state;
		table_msg.ros_Estop_state= diag_status.ros_Estop_state;
		table_pub.publish(table_msg);
	}

}
