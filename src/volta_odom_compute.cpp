#include <volta_hardware/odom_compute.h>

void imuCallback(const sensor_msgs::Imu& imu) {
    imu_yaw_rate = imu.angular_velocity.z;
}

void odomCallback(const nav_msgs::Odometry& vel)
{
    geometry_msgs::TransformStamped odom_trans;
    
    ros::Time current_time = ros::Time::now();

    linear_velocity_x_ = vel.twist.twist.linear.x;
    linear_velocity_y_ = vel.twist.twist.linear.y;
    angular_velocity_z_ = imu_yaw_rate;

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);


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

    nav_msgs::Odometry odom;

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

    odom_publisher_.publish(odom);
}

int main(int argc, char** argv )
{
    ros::init(argc, argv, "odom_compute_node");

    ros::NodeHandle nh_;

    odom_broadcaster_ = new tf::TransformBroadcaster();

    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odometry/filtered", 1);
    ros::Subscriber sub1 = nh_.subscribe("odometry/wheel", 1, odomCallback);
    ros::Subscriber sub2 = nh_.subscribe("imu/data", 1, imuCallback);

    last_vel_time_ = ros::Time::now();

    ros::spin();
    return 0;
}
