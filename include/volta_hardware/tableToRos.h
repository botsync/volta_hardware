
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "constants.h"
#include "volta_msgs/Table.h"
#include "volta_msgs/RPM.h"
#include "volta_msgs/BMS.h"

extern ros::Publisher rpm_pub;
extern ros::Subscriber rpm_sub;
extern ros::Subscriber estop_sub;
extern ros::Subscriber diagEn_sub;

extern volta_msgs::Table table_msg;
extern volta_msgs::BMS   BMS_msg;

extern ros::Publisher table_pub;
extern ros::Subscriber  publishTable;

extern ros::Publisher   bms_pub;


void rpmCallback(const volta_msgs::RPM::ConstPtr& rpmTemp);
void estopCallback(const std_msgs::Bool& e_stop_msg);
void diagEnCallback(const std_msgs::Bool& diagEn_msg);
void rosTopicInit(void);
void volta_update_RPM(float left, float right);
void publish_table(void);
void publish_bms(void);