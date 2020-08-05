/*
to use this library:
declare which node is this with CAN_monitor_register_self() at the start
put CAN_monitor_self_handle() in while 1
publish the status of self to other can node using CAN_monitor_pub();
put CAN_monitor_call_back to can receive call back
 */
#ifndef CAN_MONITOR_H_
#define CAN_MONITOR_H_


#include <stdint.h>
#include <stdbool.h>
#include "constants.h"
#include "can_priority.h"

#include "queue.h"
#include "tableToRos.h"


extern uint8_t can_monitor_data_out[8];

void CAN_set_default(void);
bool CAN_watchDog_check(void);
void Can_update_table(uint8_t table_id,uint8_t msg_id,uint8_t aData[],uint8_t size);
void CAN_monitor_call_back(uint8_t topic_id, uint8_t aData[]);


#endif /* CAN_MONITOR_H_ */
