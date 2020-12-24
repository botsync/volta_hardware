#ifndef VOLTA_BASE_ODOM_COMPUTE_H
#define VOLTA_BASE_ODOM_COMPUTE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

float imu_yaw_rate = 0;
float steering_angle_ = 0;
float linear_velocity_x_ = 0;
float linear_velocity_y_ = 0;
float angular_velocity_z_ = 0;
ros::Time last_vel_time_;
float vel_dt_ = 0;
float x_pos_ = 0;
float y_pos_ = 0;
float heading_ = 0;
ros::Publisher odom_publisher_;
tf::TransformBroadcaster *odom_broadcaster_;

#endif
