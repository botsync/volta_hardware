#include <volta_hardware/odom_compute.h>

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    imu_yaw_rate = imu->angular_velocity.z;
}

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr vel)
{
    geometry_msgs::msg::TransformStamped odom_trans;
    
    rclcpp::Time current_time = clock_->now();

    linear_velocity_x_ = vel->twist.twist.linear.x;
    linear_velocity_y_ = vel->twist.twist.linear.y;
    angular_velocity_z_ = imu_yaw_rate;

    vel_dt_ = (current_time.nanoseconds() - last_vel_time_.nanoseconds());
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //Generate the tf2 Quaternion
    quat.setRPY(0, 0, heading_);

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);


    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;

    odom_broadcaster_->sendTransform(odom_trans);

    nav_msgs::msg::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_publisher_->publish(odom);
}

int main(int argc, char** argv )
{
    rclcpp::init(argc, argv);

    node=rclcpp::Node::make_shared("odom_compute_node");

    odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 1);

    sub1 = node->create_subscription<nav_msgs::msg::Odometry>("odometry/wheel", 1, odomCallback);
    sub2 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, imuCallback);

    last_vel_time_ = clock_->now();

    rclcpp::spin(node);
    return 0;
}
