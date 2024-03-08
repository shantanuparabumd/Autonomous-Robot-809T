
/**
 * @file ex1_demo.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Code for Exercise 1
 * This file demonstrates the creation of a simple ROS 2 node that publishes
 * a stream of numbers to a topic. The node also subscribes to the same topic
 * and processes the received numbers to print only even numbers.
 * @version 0.1
 * @date 2024-02-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <rclcpp/rclcpp.hpp> ///< Include the ROS2 C++ library
#include "auto_car/demo_1_interface.hpp" ///< Include the Ex1Demo class



/**
 * @brief Main function to initialize and run the Ex1Demo.
 *
 * This is the entry point of the program. It initializes the ROS 2 system,
 * creates a Ex1Demo, and spins it to continuously publish messages until
 * the program is interrupted or terminated.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Execution status code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  ///< Initialize ROS 2.
  auto publisher_node = std::make_shared<Ex1Demo>("demo_1");  ///< Create an instance of Ex1Demo.
  rclcpp::spin(publisher_node);  ///< Enter a loop, pumping callbacks.
  rclcpp::shutdown();            ///< Shutdown ROS 2.
}
