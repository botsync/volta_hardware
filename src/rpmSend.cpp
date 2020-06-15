#include <ros/ros.h>
#include "volta_msgs/RPM.h"
#include "std_msgs/Bool.h"
// C library headers
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "stdbool.h" 
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()#include "std_msgs/Int16.h"


#define ROS_ESTOP "ROS_ES"
#define DIAG_ENABLE "DIAG_EN"

bool rpmData=false;
bool estopData=false;
bool estopStatus=false;
bool diagEnData=false;
bool diagEnStatus= false;

int16_t subMotorRPMRight=0;
int16_t subMotorRPMLeft=0;

uint8_t checkSum(char *buff);

void rpmCallback(const volta_msgs::RPM::ConstPtr& rpmTemp) {
  //ROS_INFO("data received \n");
  subMotorRPMLeft= rpmTemp->left;
  subMotorRPMRight= rpmTemp->right;
}
void estopCallback(const std_msgs::Bool& e_stop_msg)
{
	estopData=true;
	estopStatus = e_stop_msg.data;
}
void diagEnCallback(const std_msgs::Bool& diagEn_msg)
{
	diagEnData=true;
	diagEnStatus =  diagEn_msg.data;
}
int main(int argc, char *argv[]) 
{
	ros::init(argc,argv,"rpm_test");
	ROS_ERROR("START\n");
	ros::NodeHandle nh;
	ros::Subscriber rpm_sub = nh.subscribe("RPM_PUB",100,&rpmCallback);
	ros::Publisher rpm_pub = nh.advertise<volta_msgs::RPM>("RPM_SUB", 100);
	ros::Subscriber estop_sub = nh.subscribe("e_stop_sw_enable",100,&estopCallback);
	ros::Subscriber diagEn_sub=nh.subscribe("volta_diag_enable",100,&diagEnCallback);
	ros::Rate rate(20);
	volta_msgs::RPM rpm;
	int serial_port = open("/dev/mcu", O_RDWR);

	// Create new termios struc, we call it 'tty' for convention
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port, &tty) != 0) 
	{
    		ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
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

	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 9600
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
	{
    		ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}
	//sleep(2); //required to make flush work, for some reason
  	tcflush(serial_port,TCIOFLUSH);
	char msg[100] ="";
	char buf[100]="";
	char front;
	char back;
	char dataIn[200]="";
	int data1;
	int data2;
	char read_buf[100]="";
	uint8_t readSum=0;

	while(ros::ok())
	{
		ros::spinOnce();
		//ROS_ERROR("WORKING HERE");
		//ROS_INFO_STREAM("rpms :" << "left " << subMotorRPMLeft << "right" << subMotorRPMRight);
		rate.sleep();
		// Write to serial port
		if(estopData)
		{
			estopData = false;
			sprintf(msg,"$ %s %d %d",ROS_ESTOP,estopStatus == true ? 1 : 0,0);
			uint8_t sum = checkSum(msg);
			sprintf(msg,"$ %s %d %d %hhu \n",ROS_ESTOP,estopStatus == true ? 1 : 0,0,sum);
			write(serial_port,msg, strlen(msg));
		}
		else if(diagEnData)
		{
			diagEnData=false;
			sprintf(msg,"$ %s %d %d",DIAG_ENABLE,diagEnStatus == true ? 1 : 0,0);
			uint8_t sum = checkSum(msg);
			sprintf(msg,"$ %s %d %d %hhu \n",DIAG_ENABLE,diagEnStatus == true ? 1 : 0,0,sum);
			write(serial_port,msg, strlen(msg));
		}
		else
		{
//			ROS_ERROR("SENDING");
			sprintf(msg,"$ RPM %d %d",subMotorRPMLeft,subMotorRPMRight);
			uint8_t sum = checkSum(msg);
			sprintf(msg,"$ RPM %d %d %hhu \n",subMotorRPMLeft,subMotorRPMRight,sum);
			write(serial_port,msg, strlen(msg));
		}

		// Allocate memory for read buffer, set size according to your needs
		memset(&read_buf, '\0', sizeof(read_buf));

		// Read bytes. The behaviour of read() (e.g. does it block?,
		// how long does it block for?) depends on the configuration
		// settings above, specifically VMIN and VTIME
		int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

		// n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
		if (num_bytes < 0) 
		{
    			ROS_ERROR("Error reading: %s", strerror(errno));
		}

		// Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
		// print it to the screen like this!)
		if(num_bytes > 0)
		{
//			ROS_ERROR("Read %i bytes. Received message: %s", num_bytes, read_buf);
			if(strchr(read_buf,'$') != NULL && strchr(read_buf,'\n') != NULL)
			{
				sscanf(read_buf,"%c %s %d %d %hhu %c",&front,dataIn,&data1,&data2,&readSum,&back);
				sprintf(buf,"%c %s %d %d",front,dataIn,data1,data2);
				if(checkSum(buf) == readSum)
				{
					if(strcmp(dataIn,"RPM") == 0)
					{
						rpm.left = data1;
						rpm.right = data2;
						rpm_pub.publish(rpm);
					}
				}
				else
				{
				//ROS_ERROR("checksum failed %s",read_buf);
				}
			}
			else
			{
				ROS_ERROR("Read %i bytes. Received message: %s", num_bytes, read_buf);
			}
			num_bytes =0;
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
