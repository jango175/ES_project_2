// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>


#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 8
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_MOVING_SPEED 32
#define ADDR_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyACM0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

std::vector<int> interpolate(int start, int end, int steps) {
    std::vector<int> points;
    for (int i = 0; i <= steps; ++i) {
        int value = start + (end - start) * i / steps;
        points.push_back(value);
    }
    return points;
}

ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
    this->create_subscription<SetPosition>(
    "set_position",
    QOS_RKL10V,
    [this](const SetPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      std::vector<std::vector<int>> goal_positions_0 = {
          {620, 490, 485, 600, 620},
          {600, 486},
          {620, 600, 540, 550, 490, 485},
          {620, 600, 540, 550, 540, 485, 490},
          {620, 550, 540, 600, 485},
          {600, 620, 550, 540, 485, 490},
          {600, 620, 490, 485, 540, 550},
          {620, 600, 485},
          {540, 550, 620, 600, 485, 490, 550},
          {540, 550, 620, 600, 485, 490}
      };

      std::vector<std::vector<int>> goal_positions_1 = {
          {630, 788, 860, 730, 630},
          {730, 860},
          {630, 730, 805, 725, 790, 860},
          {630, 730, 805, 725, 805, 860, 790},
          {630, 725, 805, 730, 860},
          {730, 630, 725, 805, 860, 790},
          {730, 630, 790, 860, 805, 725},
          {630, 730, 860},
          {805, 725, 630, 730, 860, 790, 725},
          {805, 725, 630, 730, 860, 790}
      };


      uint32_t profile_velocity = 120;

      dxl_comm_result = packetHandler->write2ByteTxRx(
          portHandler,
          0,  // Motor ID 0
          ADDR_MOVING_SPEED,
          profile_velocity,
          &dxl_error
      );

      // Set Profile Velocity for Motor ID 1
      dxl_comm_result = packetHandler->write2ByteTxRx(
          portHandler,
          1,  // Motor ID 1
          ADDR_MOVING_SPEED,
          profile_velocity,
          &dxl_error
      );
      


      uint32_t list_index = (unsigned int)msg->position;  // Convert int32 -> uint32
      // Position Value of X series is 4 byte data.

      uint32_t goal_position_0 = static_cast<uint32_t>(goal_positions_0[list_index][0]);
      uint32_t goal_position_1 = static_cast<uint32_t>(goal_positions_1[list_index][0]);

      // Write Goal Position for Motor ID 0
      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        0,  // Motor ID 0
        ADDR_GOAL_POSITION,
        goal_position_0,
        &dxl_error
      );
      // Write Goal Position for Motor ID 1
      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        1,  // Motor ID 1
        ADDR_GOAL_POSITION,
        goal_position_1,
        &dxl_error
      );

      // Delay for 1 second
      std::this_thread::sleep_for(std::chrono::seconds(1));

      // Write Goal Position for Motor ID 2
      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        2,  // Motor ID 1
        ADDR_GOAL_POSITION,
        500,
        &dxl_error
      );

      // Delay for 1 second
      std::this_thread::sleep_for(std::chrono::seconds(1));

      for (std::vector<int>::size_type i = 1; i < goal_positions_0[list_index].size(); ++i) {
          int start_pos_0 = goal_positions_0[list_index][i - 1];
          int end_pos_0 = goal_positions_0[list_index][i];
          int start_pos_1 = goal_positions_1[list_index][i - 1];
          int end_pos_1 = goal_positions_1[list_index][i];

          // Interpolate positions for smooth movement
          std::vector<int> interpolated_0 = interpolate(start_pos_0, end_pos_0, 20);  // 20 intermediate steps
          std::vector<int> interpolated_1 = interpolate(start_pos_1, end_pos_1, 20);



          for (size_t j = 0; j < interpolated_0.size(); ++j) {
              // Write interpolated goal positions
              dxl_comm_result = packetHandler->write2ByteTxRx(
                  portHandler,
                  0,  // Motor ID 0
                  ADDR_GOAL_POSITION,
                  static_cast<uint32_t>(interpolated_0[j]),
                  &dxl_error
              );

              dxl_comm_result = packetHandler->write2ByteTxRx(
                  portHandler,
                  1,  // Motor ID 1
                  ADDR_GOAL_POSITION,
                  static_cast<uint32_t>(interpolated_1[j]),
                  &dxl_error
              );

              // Small delay for smoother motion
              std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
      }


      // Write Goal Position for Motor ID 2
      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        2,  // Motor ID 1
        ADDR_GOAL_POSITION,
        400,
        &dxl_error
      );

      std::this_thread::sleep_for(std::chrono::seconds(1));

      // Write Goal Position for Motor ID 1
      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        0,  // Motor ID 0
        ADDR_GOAL_POSITION,
        500,
        &dxl_error
      );
      // Write Goal Position for Motor ID 1
      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        1,  // Motor ID 1
        ADDR_GOAL_POSITION,
        500,
        &dxl_error
      );

      

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Motor ID 0: %s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "Motor ID 0: %s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set Motors [Goal Position: %d]", goal_position);
      }
    }
    );
}

ReadWriteNode::~ReadWriteNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
