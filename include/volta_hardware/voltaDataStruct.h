#ifndef INC_DATASTRUCT_H
#define INC_DATASTRUCT_H
#include "stdint.h"
typedef struct
{
	float left;
	float right;
	int16_t en;
} rpm_status_typedef;

typedef struct
{
	float soc;
	float soh;
	float current;
	float voltage;
} bms_status_typedef;
typedef struct
{
	float soc;
	float soh;
	float current;
	float voltage;
	uint8_t system_status;
	float relay_temp;
	float powe_in_temp;

	float motor_volt;
	float battery_volt;
	float hw_estop_volt;
	uint8_t motorAlarm;
	uint8_t en;
	uint8_t hw_Estop_state;
	uint8_t sw_Estop_state;
	uint8_t ros_Estop_state;
} diag_status_typedef;

typedef struct
{
 	float Kp1;
 	float Ki1;
 	float Kd1;
 	float Kp2;
 	float Ki2;
	float Kd2;
 	uint8_t set;
 	uint8_t save;
	uint8_t reset;
}speed_control_typedef;

extern rpm_status_typedef rpm_status;
extern rpm_status_typedef rpm_FB_status;
extern bms_status_typedef bms_status;
extern diag_status_typedef diag_status;
extern speed_control_typedef speed_control;

#define PRIORITY_RPM 				0x01
#define PRIORITY_RPM_FB 			0x02
#define PRIORITY_BMS	 			0x03
#define PRIORITY_DIAG	 			0x04
#define PRIORITY_SP				0x05

#define BMS_SOC 				0x01
#define BMS_SOH 				0x02
#define BMS_CURR 				0x03
#define BMS_VOLT 				0x04


#define DIAG_SOC 				0x01
#define DIAG_SOH 				0x02
#define DIAG_CURR 				0x03
#define DIAG_VOLT 				0x04
#define DIAG_SYS_STATUS				0x05
#define DIAG_REL_TEMP 				0x06
#define DIAG_PWR_TEMP				0x07
#define DIAG_MOTOR_VOLT 			0x08
#define DIAG_BAT_VOLT				0x09
#define DIAG_HW_ESTOP_VOLT			0x0A
#define DIAG_MOTOR_ALARM			0x0B
#define DIAG_EN					0x0C
#define DIAG_HW_ESTOP_STATE			0x0D
#define DIAG_SW_ESTOP_STATE			0x0E
#define DIAG_ROS_ESTOP_STATE			0x0F

#define SP_KP1 					0x01
#define SP_KI1 					0x02
#define SP_KD1 					0x03
#define SP_KP2 					0x04
#define SP_KI2 					0x05
#define SP_KD2 					0x06
#define SP_SET 					0x07
#define SP_SAVE					0x08
#define SP_RESET				0x09


void volta_default(void);
void volta_update_table(uint8_t table_id,uint8_t msg_id,uint8_t aData[],uint8_t size);
void volta_data_callBack(uint8_t *data_in);
void rpmDataHandler(uint8_t * data);
void rpmFBDataHandler(uint8_t * data);
void bmsDataHandler(uint8_t * data);
void diagDataHandler(uint8_t * data);
void speedControlHandler(uint8_t *data);
#endif
