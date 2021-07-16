#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <cstdlib>

using namespace std::chrono_literals;

class Mover : public rclcpp::Node
{
  public: 
    Mover() : Node("odom_test_node")
    {
      publisher_=this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      subscriber_=this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&Mover::odom_callback, this, std::placeholders::_1));
      timer_=this->create_wall_timer(50ms, std::bind(&Mover::timer_callback, this));

      // Param list is in the order: linear_x, angular_z, x_vel, yaw_vel, start_move
      this->declare_parameter("param_list", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    }

    private:
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
      rclcpp::TimerBase::SharedPtr timer_;
      bool first_move=true;
      double x_odom, y_odom, yaw_odom;
      double orientation_x, orientation_y, orientation_z, orientation_w;

      void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) 
      {
        this->x_odom=msg->pose.pose.position.x;
        this->y_odom=msg->pose.pose.position.y;

        tf2::Quaternion q(msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        this->yaw_odom=yaw;
      }

      int timer_callback()
      {
        geometry_msgs::msg::Twist twist;

        std::vector<double>param_values=this->get_parameter("param_list").as_double_array();

        if (param_values[4])
        {
          RCLCPP_INFO(this->get_logger(), "Starting to move");
          double x, y, yaw;

          twist.linear.x=param_values[2];
          twist.linear.y=0;
          twist.linear.z=0;
          twist.angular.x=0;
          twist.angular.y=0;
          twist.angular.z=param_values[3];

          int condition=(param_values[0]==0.0) ? 1 : 0;
          RCLCPP_INFO(this->get_logger(), "condition: %d", condition);

          if (this->first_move)
          {
            x=this->x_odom;
            y=this->y_odom;
            yaw=this->yaw_odom;
            this->first_move=false;
          }

          if(((!condition)&&(abs(this->x_odom)<(abs(x)+param_values[0]))) || ((condition)&&(abs(this->yaw_odom)<(abs(yaw)+param_values[1]))))
          {
            twist.linear.x=param_values[2];
            twist.linear.y=0;
            twist.linear.z=0;
            twist.angular.x=0;
            twist.angular.y=0;
            twist.angular.z=param_values[3];
            this->publisher_->publish(twist);
            param_values=this->get_parameter("param_list").as_double_array();
            RCLCPP_ERROR(this->get_logger(), "this->x_odom: %f", this->x_odom);
            RCLCPP_INFO(this->get_logger(), "x: %f", x);
            RCLCPP_ERROR(this->get_logger(), "param_values[0]: %f", param_values[0]);
          }

          else
          {
            twist.linear.x=0;
            twist.linear.y=0;
            twist.linear.z=0;
            twist.angular.x=0;
            twist.angular.y=0;
            twist.angular.z=0;

            this->first_move=true;
            this->publisher_->publish(twist);
            this->set_parameter(rclcpp::Parameter("param_list", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0}));
            RCLCPP_INFO(this->get_logger(), "Completed moving");
          }

        }

        return 0;
      }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mover>());
  rclcpp::shutdown();
  return 0;
}