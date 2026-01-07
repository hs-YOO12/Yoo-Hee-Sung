/**
 * Author: Geonwoo Kho, Heeseong Yoo
 */

#include "inpipe_robot/ipir_joint_positions.hpp"

#include <cmath>
#include <cstdio>

#define BAUDRATE    57600
#define DEVICE_NAME "/dev/ttyUSB0"

#define PROTOCOL_VERSION  2.0

#define ADDR_OPERATING_MODE   11
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT  126


IpirJointPositions::IpirJointPositions()
: Node("ipir_joint_positions")
{
  RCLCPP_INFO(this->get_logger(), "Initialize Dynamixel for Joints...");

  portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler_->openPort()) {
    RCLCPP_ERROR(rclcpp::get_logger("ipir_joint_positions"), "Failed to open the port!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ipir_joint_positions"), "Succeeded to open the port.");
  }

  if (!portHandler_->setBaudRate(BAUDRATE)) {
    RCLCPP_ERROR(rclcpp::get_logger("ipir_joint_positions"), "Failed to set the baudrate!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ipir_joint_positions"), "Succeeded to set the baudrate.");
  }

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_multi_positions_sub_ =
    this->create_subscription<SetMultiPositions>(
      "/ipir/tilt_joint_positions", QOS_RKL10V,
      std::bind(&IpirJointPositions::setMultiPositionsCallback, this, std::placeholders::_1));

  get_position_srv_ =
    this->create_service<GetPosition>(
      "get_position",
      std::bind(&IpirJointPositions::getPositionCallback, this, std::placeholders::_1, std::placeholders::_2));
}

IpirJointPositions::~IpirJointPositions()
{
}

void IpirJointPositions::setupDynamixel(uint8_t dxl_id)
{
  dxl_comm_result = packetHandler_->write1ByteTxRx(
    portHandler_, dxl_id, ADDR_OPERATING_MODE, 3, &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to set Position Control Mode: %s {%d}\nDXL_ERROR: %s {%d}",
      packetHandler_->getTxRxResult(dxl_comm_result), dxl_comm_result,
      packetHandler_->getRxPacketError(dxl_error), dxl_error);
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Succeeded to set Position Control Mode.");
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(
    portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to enable torque.");
  }
}

void IpirJointPositions::setMultiPositionsCallback(const SetMultiPositions::SharedPtr msg)
{
  const double L = 136.0;   // Link Length [mm]
  const double W_R = 113.0; // Wheel Separation [mm]
  const double R_w = 58.0;  // Wheel Diameter [mm]

  for (size_t i = 0; i < msg->ids.size(); i++) {
    double command = static_cast<double>(msg->positions[i]);
    double D_P = std::abs(command) * 10.0;  // [cm] to [mm]

    double H_w = std::abs(std::sqrt((D_P * D_P) - (W_R * W_R)));
    double H_shaft = H_w - R_w;

    if (H_shaft < 0 || (L * L) < (H_shaft * H_shaft)) {
      RCLCPP_WARN(this->get_logger(), "Invalid command: out of range for geometry. [D_p=%.2f cm]", command);
      continue;
    }

    double theta2_numerator = (L * L) - (H_shaft * H_shaft);
    double theta2_denominator = L * std::sqrt((L * L) - (H_shaft * H_shaft));
    double theta2_rad = 2.0 * std::acos(theta2_numerator / theta2_denominator);
    double theta2_deg = theta2_rad * 180.0 / M_PI;

    uint32_t delta_position = static_cast<uint32_t>(theta2_deg * 4095.0 / 360.0);
    uint32_t center_position = 2048;

    if (command > 0) {
      goal_position = (center_position + delta_position) % 4096;
    } else if (command < 0) {
      goal_position = (center_position - delta_position + 4096) % 4096;
    } else {
      goal_position = center_position;
    }

    dxl_comm_result = packetHandler_->write4ByteTxRx(
      portHandler_, msg->ids[i], ADDR_GOAL_POSITION, goal_position, &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getRxPacketError(dxl_error));
    } else {
      dxl_comm_result = packetHandler_->read2ByteTxRx(
        portHandler_, msg->ids[i], ADDR_PRESENT_CURRENT, reinterpret_cast<uint16_t*>(&present_current)
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler_->getRxPacketError(dxl_error));
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Set [ID: %d] [Input Distance: %.2f cm] [02: %.2f deg] [Goal pos: %d] [Current: %d mA]",
        msg->ids[i], command, theta2_deg, goal_position, present_current);
    }
  }
}

void IpirJointPositions::getPositionCallback(
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response)
{
  dxl_comm_result = packetHandler_->read4ByteTxRx(
    portHandler_, static_cast<uint8_t>(request->id),
    ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&present_position), &dxl_error
  );

  RCLCPP_INFO(this->get_logger(),
    "Get [ID: %d] [Present Position: %d]", request->id, present_position);

  response->position = present_position;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IpirJointPositions>();
  node->setupDynamixel(BROADCAST_ID);

  rclcpp::spin(node);
  rclcpp::shutdown();

  // Enable Torque
  node->packetHandler_->write1ByteTxRx(node->portHandler_, BROADCAST_ID,
                                ADDR_TORQUE_ENABLE, 0, nullptr);

  return 0;
}