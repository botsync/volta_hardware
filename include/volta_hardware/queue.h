#ifndef QUEUE_H
#define QUEUE_H
#include <stdint.h>
#include <stdbool.h>

int queue_insert(uint8_t*,uint8_t);
int queue_insertWithCount(uint8_t *,int , int );
int queue_delete(uint8_t);
int8_t queue_Count(uint8_t );
int queue_dataGet(uint8_t *,uint8_t );
void copyString(uint8_t * ,uint8_t *);
void copyStringWithCount(uint8_t * ,uint8_t *,int );
void printQueue();
void clearBuff(uint8_t * ,uint8_t );
void sort(uint8_t);
int compare(uint8_t string1[], uint8_t string2[],uint8_t size);
#endif
