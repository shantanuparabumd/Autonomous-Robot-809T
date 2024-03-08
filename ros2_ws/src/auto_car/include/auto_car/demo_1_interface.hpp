/**
 * @file ex1_interface.hpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Declares the Ex1Demo class for publsihing and subscribing to "even_number" topic
 * messages.
 *
 * This file contains the declaration of the Ex1Demo class, which
 * extends the rclcpp::Node class to create a ROS2 publisher that send 
 * a stream of integers to the "even_number" topic. It also has a subscriber that listens
 * to messages of type std_msgs::msg::Int32 published on the "even_number" topic and prints only the 
 * even numbers.

 * @version 0.1
 * @date 2024-02-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp> // Include the ROS2 C++ library
#include <std_msgs/msg/int32.hpp> // Include the Int32 message type
#include <chrono> // Include the chrono library for time related functions
#include "auto_car/gpio.hpp"
/**
 * @class Ex1Demo
 * @brief A ROS2 pubslisher subscriber node class
 * 
 * Ex1Demo is a class derived from rclcpp::Node designed to publish and subscribe to
 * integer messages on a topic named "even_number" and log only the even number. 
 */

class Ex1Demo : public rclcpp::Node{
 public:
  /**
   * @brief Construct a new Ex1Demo object with a spcific node name
   * 
   * This constructor initializes the node with a specific name and sets up a timer

   * @param node_name Name of the node.
   */
  Ex1Demo(std::string node_name) : Node(node_name){
    RCLCPP_INFO_STREAM(this->get_logger(), node_name <<" has been started.");

    // Setup a timer to call timer_callback every 1000 milliseconds
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0)),
        std::bind(&Ex1Demo::publish_number, this));

    // Initialize the publisher on topic "even_number" with a queue size of 10
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("even_number", 10);

    // Initialize the subscriber on topic "even_number" with a queue size of 10
    subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "even_number", 10,
        std::bind(&Ex1Demo::process_number, this, std::placeholders::_1));

    pin_ = GPIO(25);
    pin_.setMode("out");


  } 

  private:
    
    GPIO pin_;

    rclcpp::TimerBase::SharedPtr timer_;  ///< Timer to trigger publishing.

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  publisher_;  ///< The publisher object.

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_; ///< The subscriber object.
 
    /**
    * @brief Timer callback function that publishes a message.
    *
    * This function constructs a Int32 message and publishes the message.
    */
    void publish_number();

    /**
     * @brief Callback function to process received integer.
     * 
     * This function processes the received integer and logs only the even numbers.
     * @param msg 
     */
    void process_number(const std_msgs::msg::Int32::SharedPtr msg);

    int number_ = 0;

    int flag = 1;

}; 


//=====================================
void Ex1Demo::publish_number() {
  // Initialize the message
  std_msgs::msg::Int32 number = std_msgs::msg::Int32();

  // Set the data by incrementing the number_ variable
  number.data = number_++;

  // Publish the message
  publisher_->publish(number);

  // Log the published number
  RCLCPP_INFO_STREAM(this->get_logger(), "Published number: " << number.data);

  if(flag==1){
    pin_.setValue(flag);
    flag=0;
  }
  else{
    pin_.setValue(flag);
    flag=1;
  }


}

//=====================================
void Ex1Demo::process_number(const std_msgs::msg::Int32::SharedPtr msg) {
  
  // Check if the received number is even
  if (msg->data % 2 == 0) {
    // Log the even number
    RCLCPP_INFO_STREAM(this->get_logger(), "Even number: " << msg->data);
  }
}