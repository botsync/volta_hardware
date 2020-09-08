
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "constants.h"
#include "can_monitor.h"
#include "volta_msgs/table.h"
#include "volta_msgs/RPM.h"

extern ros::Publisher rpm_pub;
extern ros::Subscriber rpm_sub;
extern ros::Subscriber estop_sub;
extern ros::Subscriber diagEn_sub;
extern volta_msgs::table table_msg;
extern ros::Publisher table_pub;
extern ros::Subscriber  publishTable;


void rpmCallback(const volta_msgs::RPM::ConstPtr& rpmTemp);
void estopCallback(const std_msgs::Bool& e_stop_msg);
void diagEnCallback(const std_msgs::Bool& diagEn_msg);
void rosTopicInit(void);
void tableCallback(const std_msgs::Bool& table_data);
void volta_update_RPM(int16_t left, int16_t right);
void publish_table(void);
