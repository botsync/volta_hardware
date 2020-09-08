#ifndef INC_CONVERSION_H
#define INC_CONVERSION_H
#include "stdint.h"
void float2Bytes(uint8_t bytes_temp[4],float float_variable);
void int2Bytes(uint8_t bytes_temp[4],int int_variable);
void unsignedint2Bytes(uint8_t bytes_temp[4],uint32_t int_variable);
void short2Bytes(uint8_t bytes_temp[2],int16_t variable);
void unsignedshort2Bytes(uint8_t bytes_temp[2],uint16_t variable);
void bytes2Float(uint8_t bytes_temp[4],float *float_variable);
void bytes2int(uint8_t bytes_temp[4],int *variable);
void bytes2unsignedint(uint8_t bytes_temp[4],uint32_t *variable);
void bytes2short(uint8_t bytes_temp[2],int16_t *variable);
void bytes2unsignedshort(uint8_t bytes_temp[2],uint16_t *variable);




#endif
