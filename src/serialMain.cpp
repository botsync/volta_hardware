#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
// C library headers
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <thread>
#include "stdlib.h"
#include "stdbool.h"
#include "time.h"
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()#include "std_msgs/Int16.h"

#include "volta_hardware/queue.h"
#include "volta_hardware/tableToRos.h"
#include "volta_hardware/constants.h"
#include "volta_hardware/voltaDataStruct.h"
#include "volta_hardware/conversion.h"

const uint8_t initSequence[5]={'F','F','F','F','F'};
const uint8_t endSequence[5]={'E','E','E','E','E'};

extern bool estopData;
extern bool estopStatus;
bool wifiData=false;
bool wifiStatus= false;

uint8_t incomplete[200];
uint8_t incompleteSize=0;
bool foundIncomplete=false;

uint8_t checkSum(char *buff);
void findData(uint8_t *data , uint8_t size);
uint8_t findsequence(uint8_t* data,uint8_t size);
void dataClear(uint8_t * data,uint8_t size);

float subMotorRPMRight=0;
float subMotorRPMLeft=0;
uint8_t rpmAvailable = false;

/*

void controlsCallback(volta_hardware::controlsConfig &config, uint32_t level)
{
  uint8_t update;
  uint8_t data[10]="";
  float gain=0.0;
  ROS_INFO("controls config updated");
  if(config.Kp1 != speed_control.Kp1)
  {
    speed_control.Kp1 = config.Kp1;
    float2Bytes(data,speed_control.Kp1);
    volta_update_table(PRIORITY_SP,SP_KP1,data,4);
  }
  if(config.Ki1 != speed_control.Ki1)
  {
    speed_control.Ki1 = config.Ki1;
    float2Bytes(data,speed_control.Ki1);
    volta_update_table(PRIORITY_SP,SP_KI1,data,4);
  }
 if(config.Kd1 != speed_control.Kd1)
  {
    speed_control.Kd1 = config.Kd1;
    float2Bytes(data,speed_control.Kd1);
    volta_update_table(PRIORITY_SP,SP_KD1,data,4);
  }
  if(config.Kp2 != speed_control.Kp2)
  {
    speed_control.Kp2 = config.Kp2;
    float2Bytes(data,speed_control.Kp2);
    volta_update_table(PRIORITY_SP,SP_KP2,data,4);
  }
  if(config.Ki2 != speed_control.Ki2)
  {
    speed_control.Ki2 = config.Ki2;
    float2Bytes(data,speed_control.Ki2);
    volta_update_table(PRIORITY_SP,SP_KI2,data,4);
  }
 if(config.Kd2 != speed_control.Kd2)
  {
    speed_control.Kd2 = config.Kd2;
    float2Bytes(data,speed_control.Kd2);
    volta_update_table(PRIORITY_SP,SP_KD2,data,4);
  }

  data[0] = config.write_controls;
  volta_update_table(PRIORITY_SP,SP_SET,data,1);

 data[0] = config.Save_to_eeprom;
  volta_update_table(PRIORITY_SP,SP_SAVE,data,1);

 data[0] = config.reset_pid;
  volta_update_table(PRIORITY_SP,SP_RESET,data,1);
}

*/

int main(int argc, char *argv[])
{
	rclcpp::init(argc,argv);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("volta_control");
	RCLCPP_DEBUG(node->get_logger(), "START\n");

	rosTopicInit(node);

	/*
	dynamic_reconfigure::Server<volta_hardware::controlsConfig> server;
  	dynamic_reconfigure::Server<volta_hardware::controlsConfig>::CallbackType f;
  	dynamic_reconfigure::ReconfigureRequest srv_req;
  	dynamic_reconfigure::ReconfigureResponse srv_resp;
  	dynamic_reconfigure::IntParameter double_param;
 	dynamic_reconfigure::Config conf;
 	f = boost::bind(&controlsCallback, _1, _2);
 	server.setCallback(f);
	 */

	std::chrono::microseconds us{100};
	int serial_port = open("/dev/mcu", O_RDWR|O_NONBLOCK);

	// Create new termios struc, we call it 'tty' for convention
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port, &tty) != 0)
	{
    		RCLCPP_ERROR(node->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 9600
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
	{
    		RCLCPP_ERROR(node->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}
	//sleep(2); //required to make flush work, for some reason
  	tcflush(serial_port,TCIOFLUSH);
	uint8_t msg[100] ="";
	uint8_t buf[50]="";
	uint8_t msgSize =0;
	char front;
	char back;
	char dataIn[200]="";
	int data1;
	int data2;
	uint8_t read_buf[200]="";
	uint8_t readSum=0;
	while(rclcpp::ok())
	{
		rclcpp::spin_some(node);

		std::this_thread::sleep_for(us);
		// Write to serial port
		// Allocate memory for read buffer, set size according to your needs
		if(rpmAvailable == true)
		{
			msgSize=10;
			uint8_t tempCount=0;
			rpmAvailable = false;
			uint8_t tempData[50] = "";
			tempData[0] = PRIORITY_RPM;
			tempData[1] = 0x00;
			tempData[2] = 9;
			tempData[3] = 0x00;

			float2Bytes(tempData+4,subMotorRPMLeft);
			float2Bytes(tempData+8,subMotorRPMRight);
			msgSize+=(tempData[2]+3);
			for(tempCount=0;tempCount<5; tempCount++)
			{
				msg[tempCount]	='F';
			}
			for(int i=0 ;i < (tempData[2]+3) ; i++ , tempCount++)
			{
				msg[tempCount] = tempData[i];
			}
			for(int i=0 ;i < 5 ; i++,tempCount++)
			{
				msg[tempCount]	='E';
			}
			write(serial_port,msg, msgSize);
		}
		if(queue_Count(1) >0)
		{
			msgSize=10;
			uint8_t tempCount=0;

			queue_dataGet(buf,1);
			msgSize+=(buf[2]+3);
			for(tempCount=0;tempCount<5; tempCount++)
			{
				msg[tempCount]	='F';
			}
			for(int i=0 ;i < (buf[2]+3) ; i++ , tempCount++)
			{
				msg[tempCount] = buf[i];
			}
			for(int i=0 ;i < 5 ; i++,tempCount++)
			{
				msg[tempCount]	='E';
			}

			write(serial_port,msg, msgSize);

		}
		memset(&read_buf, '\0', sizeof(read_buf));

		// Read bytes. The behaviour of read() (e.g. does it block?,
		// how long does it block for?) depends on the configuration
		// settings above, specifically VMIN and VTIME
		int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

		// n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
		if (num_bytes < 0)
		{
    			RCLCPP_ERROR(node->get_logger(), "Error reading: %s", strerror(errno));
		}

		// Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
		// print it to the screen like this!)
		if(num_bytes > 0)
		{
			findData(read_buf,num_bytes);
		}

	}
}
uint8_t checkSum(char *buff)
{
	uint8_t len = strlen(buff);
	uint8_t sum =0;
	for(int i =0 ;i < len ; i++)
	{
		sum +=buff[i];
	}
	return sum;
}
void findData(uint8_t *data , uint8_t size)
{
	uint8_t tempData[200];
	if(foundIncomplete == false)
	{
		for(int i=0; i< size ; i++)
		{
			tempData[i] = data[i];
		}
	}
	else
	{
		foundIncomplete = false;
		uint8_t count =0;
		for(count = 0; count <incompleteSize;count++ )
		{
			tempData[count] = incomplete[count];
		}
		for(int i=0;i<size ; i++,count++)
		{
			tempData[count] = data[i];
		}
		size+=incompleteSize;
	}

	uint8_t  dataOccurance=1;
	while(dataOccurance !=0)
	{
		dataOccurance = findsequence(tempData,size);
		if(dataOccurance >0)
		{
			uint16_t topicId = ((uint16_t)(tempData[dataOccurance]&0x00ff) << 8);
			topicId |= tempData[dataOccurance+1];
			volta_data_callBack(tempData+dataOccurance);
			dataClear(tempData,dataOccurance+10+3+tempData[dataOccurance+2]-5);

		}
		else
		{
			uint8_t dataLocation = 0;
			bool dataFound=false;
			incompleteSize=0;
			for(int i=0;i< size ;i++)
			{

				if(tempData[i] > 0)
				{
					dataFound =true;
					dataLocation=i;
					break;
				}
			}
			if(dataFound >0)
			{
				foundIncomplete = true;
				for(int i=0;i<size-dataLocation;i++)
				{
					incomplete[i] = tempData[i+dataLocation];
					incompleteSize++;
				}
			}

		}
	}
}
uint8_t findsequence(uint8_t* data,uint8_t size)
{
	uint8_t occurance = 0;
	for(int i =0  ; i < size ; i++)
	{
		if(data[i] == 'F')
		{
			occurance = i+5;
			uint8_t tempCount =0;
			for(int j=0;j<5;j++,i++)
			{
				if(data[i] == 'F')
				{
					tempCount++;
				}
			}
			if(tempCount == 5)
			{
				i+= data[i+2]+3;
				for(int j=0;j<5;j++,i++)
				{
					if(data[i] == 'E')
					{
					tempCount++;
					}
				}
				if(tempCount == 10)
				{
					return occurance;
				}
				else
				{
					i = occurance;
					occurance = 0;
				}
			}
			else
			{
					occurance = 0;
			}
		}
	}
	return occurance;
}
void dataClear(uint8_t * data,uint8_t size)
{
	for(int i=0;i<size;i++)
	{
		data[i]=0;
	}
}
