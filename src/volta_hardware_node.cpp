#include <volta_hardware/volta_hardware.h>
#include "rclcpp/rclcpp.hpp"
#include <controller_manager/controller_manager.hpp>
#include <pluginlib/class_loader.hpp>

int main(int argc, char *argv[]) 

{
    rclcpp::init(argc, argv);

    std::string controller_manager_node_name="volta_hardware_node";
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto controller_manager_node = std::make_shared<controller_manager::ControllerManager>(executor, controller_manager_node_name);

    std::thread cm_thread([controller_manager_node]() {
      // load controller_manager update time parameter
      double control_freq=10.0;
      controller_manager_node->declare_parameter("control_frequency", 10.0);
      control_freq=controller_manager_node->get_parameter("control_frequency").as_double();

      if (!control_freq) {
        throw std::runtime_error("control frequency parameter not existing or empty");
      }
      RCLCPP_INFO(controller_manager_node->get_logger(), "control frequency is %f Hz", control_freq);

      while (rclcpp::ok()) {
        std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
        controller_manager_node->read();
        controller_manager_node->update();
        controller_manager_node->write();
        std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
        std::this_thread::sleep_for(
          std::max(
            std::chrono::nanoseconds(0),
            std::chrono::nanoseconds(1000000000 / (int)control_freq) -
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)));
      }
    });

    // Run the node(s)
    executor->add_node(controller_manager_node);
    executor->spin();
    cm_thread.join();
    
    // Exit
    rclcpp::shutdown();
    return 0;

}