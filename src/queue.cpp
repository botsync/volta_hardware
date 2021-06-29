/*
 * queue.cpp
 *
 *  Created on: 30-Mar-2020
 *      Author: james
 */
#include "volta_hardware/queue.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#define QUEUE_MAX 10
uint8_t queueOne[QUEUE_MAX][50];
uint8_t queueTwo[QUEUE_MAX][50];
int8_t queueCountOne=0;
int8_t queueCountTwo=0;

int queue_insert(uint8_t *data, uint8_t queueId)
{
	if(queueId == 1)
	{
		if(queueCountOne >= QUEUE_MAX)
		{
			return -1;
		}
		copyString(data,queueOne[queueCountOne]);
		queueCountOne++;
		return queueCountOne;
	}
	else
	{
		if(queueCountTwo >= QUEUE_MAX)
		{
			return -1;
		}
		copyString(data,queueTwo[queueCountTwo]);
		queueCountTwo++;
		return queueCountTwo;
	}
	return -1;
}
int queue_insertWithCount(uint8_t *data,int count, int queueId)
{
	if(queueId == 1)
	{
		if(queueCountOne >= QUEUE_MAX)
		{
			return -1;
		}
		copyStringWithCount(data,queueOne[queueCountOne],count);
		queueCountOne++;	
		return queueCountOne;
	}
	else
	{
		if(queueCountTwo >= QUEUE_MAX)
		{
			return -1;
		}
		copyStringWithCount(data,queueTwo[queueCountTwo],count);
		queueCountTwo++;
		return queueCountTwo;
	}
}
int queue_delete(uint8_t queueId)
{
	if(queueId == 1)
	{
		if(queueCountOne == 0)
		{
			return -1;
		}
		for(int i=0 ; i <=queueCountOne ; i++)
		{
			clearBuff(queueOne[i],50);
			copyString(queueOne[i+1] ,queueOne[i]);
		}
		queueCountOne--;
		return queueCountOne;
	}
	else
	{
		if(queueCountTwo == 0)
		{
			return -1;
		}
		for(int i=0 ; i <=queueCountTwo ; i++)
		{
			clearBuff(queueTwo[i],50);
			copyString(queueTwo[i+1] ,queueTwo[i]);
		}
		queueCountTwo--;
		return queueCountTwo;
	}
}

int8_t queue_Count(uint8_t queueId)
{
	if(queueId == 1)
	{
		return queueCountOne;
	}
	else
	{
		return queueCountTwo;
	}

}

int queue_dataGet(uint8_t *data,uint8_t queueId)
{
	if(queueId == 1)
	{
		if(queueCountOne == 0)
		{
			return -1;
		}
		copyString(queueOne[0],data);
		queue_delete(1);
		return 1;
	}
	else
	{
		if(queueCountTwo == 0)
		{
			return -1;
		}
		copyString(queueTwo[0],data);
		queue_delete(2);
		return 1;
	}

}

void copyString(uint8_t *source ,uint8_t *destination)
{
	int i =0;
	for( i=0; i<50 ; i++)
	{
		destination[i] = source[i];
	}
}
void copyStringWithCount(uint8_t *source ,uint8_t *destination,int n)
{
	int i =0;
	for( i=0; i<n ; i++)
	{
		destination[i] = source[i];
	}
}

void printQueue(void)
{
	char buff[20];

	for(int i=0; i< queueCountOne ;i++)
	{
		sprintf(buff,"\n%d ) ",i);
	}
}
void clearBuff(uint8_t * data,uint8_t count)
{
	for(int i=0 ;i < count ;i++)
	{
		data[i] = 0;
	}
}
void sort(uint8_t queueId)
{
	uint8_t s[50];
	if(queueId == 1)
	{
		for(int i=0;i<queueCountOne;i++)
		{
			for(int j=i+1;j<queueCountOne;j++)
			{
				if(compare(queueOne[i],queueOne[j],50)>0)
				{
					copyStringWithCount(s,queueOne[i],50);
					copyStringWithCount(queueOne[i],queueOne[j],50);
					copyStringWithCount(queueOne[j],s,50);
				}
			} 
		}
	}
	else
	{
		for(int i=0;i<queueCountTwo;i++)
		{
			for(int j=i+1;j<queueCountTwo;j++)
			{
				if(compare(queueTwo[i],queueTwo[j],50)>0)
				{
					copyStringWithCount(s,queueTwo[i],50);
					copyStringWithCount(queueTwo[i],queueTwo[j],50);
					copyStringWithCount(queueTwo[j],s,50);
				} 
			}
		}
	}
}
 
int compare(uint8_t string1[],uint8_t string2[],uint8_t size)
{
	for(int i = 0; i < size ; i++)
	{
		if(string1[i] != string2[i])
		{
			return string1[i] < string2[i] ? -1 : 1;
		}
	}
	return 0;
}

