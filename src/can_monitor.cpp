#include "volta_base/can_monitor.h"
#include "stdio.h"
#include "volta_base/voltaDataStruct.h"



void CAN_set_default(void)
{


/*
	WatchDog_status.Power_Board= HAL_GetTick();
	WatchDog_status.PC= HAL_GetTick();
	WatchDog_status.Corner_Proximity_1= HAL_GetTick();
	WatchDog_status.Corner_Proximity_2= HAL_GetTick();
	WatchDog_status.Corner_Proximity_3= HAL_GetTick();
	WatchDog_status.Corner_Proximity_4= HAL_GetTick();
	WatchDog_status.BMS= HAL_GetTick();
	WatchDog_status.IO= HAL_GetTick();
	WatchDog_status.LED= HAL_GetTick();
	WatchDog_status.Lifting_Contoller= HAL_GetTick();
	WatchDog_status.Conveyor_Controller= HAL_GetTick();
	WatchDog_status.SONAR_F= HAL_GetTick();
	WatchDog_status.SONAR_R= HAL_GetTick();
	WatchDog_status.Guide_Sensor= HAL_GetTick();
*/


}
void Can_update_table(uint8_t table_id,uint8_t msg_id,uint8_t aData[],uint8_t size)
{
	volta_update_table(table_id,msg_id,aData,size);	
}



