#include <cstdio>
#include <memory>
#include <string>
#include <cmath>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "inpipe_robot_interfaces/msg/set_multi_positions.hpp"
#include "inpipe_robot_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "inpipe_robot/read_write_node.hpp"

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT 126  // 🔄 추가: 현재 전류 주소

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 57600
#define DEVICE_NAME "/dev/ttyUSB0"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;
int32_t present_position = 0;
int16_t present_current = 0;  // 🔄 추가: 현재 전류 변수

ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_multi_position_subscriber_ =
  this->create_subscription<inpipe_robot_interfaces::msg::SetMultiPositions>(
    "set_multi_position",
    QOS_RKL10V,
    [this](const inpipe_robot_interfaces::msg::SetMultiPositions::SharedPtr msg) -> void
    {
      for (size_t i = 0; i < msg->ids.size(); ++i) {
        double cm_input = static_cast<double>(msg->positions[i]);
        double D_p = std::abs(cm_input) * 10.0;

        double L = 136.0;
        double W_R = 113.0;
        double R_w = 58.0;

        double H_w = std::abs(std::sqrt(std::pow(D_p, 2) - std::pow(W_R, 2)));
        double H_shaft = H_w - R_w;

        if (H_shaft < 0 || std::pow(L, 2) < std::pow(H_shaft, 2)) {
          RCLCPP_WARN(this->get_logger(), "Invalid input: out of range for geometry. [D_p=%.2f cm]", cm_input);
          continue;
        }

        double numerator = std::pow(L, 2) - std::pow(H_shaft, 2);
        double denominator = L * std::sqrt(std::pow(L, 2) - std::pow(H_shaft, 2));
        double theta2_rad = 2.0 * std::acos(numerator / denominator);
        double theta2_deg = theta2_rad * 180.0 / M_PI;

        uint32_t delta_position = static_cast<uint32_t>(theta2_deg * 4095.0 / 360.0);
        uint32_t center_position = 2048;

        if (cm_input > 0) {
          goal_position = (center_position + delta_position) % 4096;
        } else if (cm_input < 0) {
          goal_position = (center_position - delta_position + 4096) % 4096;
        } else {
          goal_position = center_position;
        }

        dxl_comm_result = packetHandler->write4ByteTxRx(
          portHandler,
          msg->ids[i],
          ADDR_GOAL_POSITION,
          goal_position,
          &dxl_error
        );

        if (dxl_comm_result != COMM_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
          RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
        } else {
          // 🔄 추가: 현재 전류값 읽기
          dxl_comm_result = packetHandler->read2ByteTxRx(
            portHandler,
            msg->ids[i],
            ADDR_PRESENT_CURRENT,
            reinterpret_cast<uint16_t *>(&present_current),
            &dxl_error
          );

          if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
          } else if (dxl_error != 0) {
            RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
          }

          RCLCPP_INFO(this->get_logger(),
            "Set [ID: %d] [Input Distance: %.2f cm] [θ2: %.2f deg] [Goal Pos: %d] [Current: %d mA]",
            msg->ids[i], cm_input, theta2_deg, goal_position, present_current);
        }
      }
    }
  );

  auto get_present_position =
    [this](
      const std::shared_ptr<inpipe_robot_interfaces::srv::GetPosition::Request> request,
      std::shared_ptr<inpipe_robot_interfaces::srv::GetPosition::Response> response) -> void
    {
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        static_cast<uint8_t>(request->id),
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<inpipe_robot_interfaces::srv::GetPosition>("get_position", get_present_position);
}

ReadWriteNode::~ReadWriteNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{
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

  if (!portHandler->openPort()) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");

  if (!portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);
  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}

