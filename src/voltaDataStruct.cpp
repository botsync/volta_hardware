/*
 * dataStruct.c
 *
 *  Created on: 19-May-2020
 *      Author: james
 */

#include <ros/ros.h>
#include <string.h>
#include "volta_base/voltaDataStruct.h"
#include "volta_base/conversion.h"
#include "stdint.h"
#include "volta_base/queue.h"
#include "volta_base/tableToRos.h"

rpm_status_typedef rpm_status;
rpm_status_typedef rpm_FB_status;
bms_status_typedef bms_status;
diag_status_typedef diag_status;
void volta_default(void)
{
	rpm_status.left = 0;
	rpm_status.right = 0;
	rpm_status.en =0;
}
void volta_update_table(uint8_t table_id,uint8_t msg_id,uint8_t aData[],uint8_t size)
{
	uint8_t msgTemp[50];

		msgTemp[0] = table_id;
		msgTemp[1] = 0x00;
		msgTemp[2] = size+1;
		msgTemp[3] = msg_id;
		for(int i = 0 ; i < size ; i++)
		{
			msgTemp[4+i] = aData[i];
		}
		volta_data_callBack(msgTemp);
		queue_insert(msgTemp,1);
}
void volta_data_callBack(uint8_t* data_in)
{
		
	switch(data_in[0])
	{
	case PRIORITY_RPM:
		rpmDataHandler(data_in);
		break;
	case PRIORITY_RPM_FB:
		/*char tempBuff[20];	
		char prinfbuff[1000]; 
		memset(prinfbuff,0,1000);
		for(int i=0;i< (data_in[2]+3) ;i++)
		{
			sprintf(tempBuff," %x",data_in[i]);
			strcat(prinfbuff,tempBuff);
		}
		strcat(prinfbuff,"\n");
		ROS_ERROR("Received message: %s", prinfbuff);
		*/
		rpmFBDataHandler(data_in);
		volta_update_RPM(rpm_FB_status.left,rpm_FB_status.right);
		break;
	case PRIORITY_BMS:
		bmsDataHandler(data_in);
		break;
	case PRIORITY_DIAG:
		diagDataHandler(data_in);
		publish_table();
		break;
	}
}

void rpmDataHandler(uint8_t * data)
{
	bytes2short((data+4),&rpm_status.left);
	bytes2short((data+6),&rpm_status.right);
	rpm_status.en = 1;
}
void rpmFBDataHandler(uint8_t * data)
{
	bytes2short((data+4),&rpm_FB_status.left);
	bytes2short((data+6),&rpm_FB_status.right);
	rpm_FB_status.en = 1;
}
void bmsDataHandler(uint8_t * data)
{
	switch(data[3])
	{
	case BMS_SOC:
		bytes2Float((data+4),&bms_status.soc);
		break;
	case BMS_SOH:
		bytes2Float((data+4),&bms_status.soh);
		break;
	case BMS_CURR:
		bytes2Float((data+4),&bms_status.current);
		break;
	case BMS_VOLT:
		bytes2Float((data+4),&bms_status.voltage);
		break;
	}
}
void diagDataHandler(uint8_t * data)
{
	switch(data[3])
	{
	case DIAG_SOC:
		bytes2Float((data+4),&diag_status.soc);
		break;
	case DIAG_SOH:
		bytes2Float((data+4),&diag_status.soh);
		break;
	case DIAG_CURR:
		bytes2Float((data+4),&diag_status.current);
		break;
	case DIAG_VOLT:
		bytes2Float((data+4),&diag_status.voltage);
		break;
	case DIAG_SYS_STATUS:
		diag_status.system_status = data[4];
		break;
	case DIAG_REL_TEMP:
		bytes2Float((data+4),&diag_status.relay_temp);
		break;
	case DIAG_PWR_TEMP:
		bytes2Float((data+4),&diag_status.powe_in_temp);
		break;
	case DIAG_MOTOR_VOLT:
		bytes2Float((data+4),&diag_status.motor_volt);
		break;
	case DIAG_BAT_VOLT:
		bytes2Float((data+4),&diag_status.battery_volt);
		break;
	case DIAG_HW_ESTOP_VOLT:
		bytes2Float((data+4),&diag_status.hw_estop_volt);
		break;
	case DIAG_MOTOR_ALARM:
		diag_status.motorAlarm = data[4];
		break;
	case DIAG_EN:
		diag_status.en = data[4];
		break;
	case DIAG_HW_ESTOP_STATE:
		diag_status.hw_Estop_state = data[4];
		break;
	case DIAG_SW_ESTOP_STATE:
		diag_status.sw_Estop_state = data[4];
		break;
	case DIAG_ROS_ESTOP_STATE:
		diag_status.ros_Estop_state = data[4];
		break;
	}
}
