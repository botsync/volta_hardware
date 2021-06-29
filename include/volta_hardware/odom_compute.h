#ifndef VOLTA_BASE_ODOM_COMPUTE_H
#define VOLTA_BASE_ODOM_COMPUTE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

float imu_yaw_rate = 0;
float steering_angle_ = 0;
float linear_velocity_x_ = 0;
float linear_velocity_y_ = 0;
float angular_velocity_z_ = 0;

rclcpp::Clock::SharedPtr clock_;
rclcpp::Time last_vel_time_;

float vel_dt_ = 0;
float x_pos_ = 0;
float y_pos_ = 0;
float heading_ = 0;

std::shared_ptr<rclcpp::Node>node;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub2;
tf2_ros::TransformBroadcaster *odom_broadcaster_;

tf2::Quaternion quat;

#endif
