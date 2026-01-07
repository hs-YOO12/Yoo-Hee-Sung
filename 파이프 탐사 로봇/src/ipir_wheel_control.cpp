/**
 * Author: Geonwoo Kho, Heeseung Yoo
 */

#include "inpipe_robot/ipir_wheel_control.hpp"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <map>
#include <termios.h>
#include <fcntl.h>


#define DEVICENAME              "/dev/ttyUSB0"
#define BAUDRATE                57600
#define PROTOCOL_VERSION        2.0

// Control Table Address
#define ADDR_OPERATING_MODE     11
#define VELOCITY_CONTROL_MODE   1

#define ADDR_TORQUE_ENABLE      64
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0

#define ADDR_GOAL_VELOCITY      104
#define ADDR_PRESENT_CURRENT    126
#define VELOCITY_MAX            256
#define VELOCITY_MULTIPLY       50


IpirWheelControlNode::IpirWheelControlNode() : Node("ipir_wheel_control_node")
{
  using namespace dynamixel;

  RCLCPP_INFO(this->get_logger(), "Initialize Dynamixel for Wheels...");

  portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler_->openPort()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to Open the Port.");
    return;
  }

  if (!portHandler_->setBaudRate(BAUDRATE)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to Configure Baudrate");
    return;
  }

  uint8_t dxl_error;

  for (int id : {6, 7, 8, 9}) {
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  }

  RCLCPP_INFO(this->get_logger(), "Dynamixel Velocity Control Node ON.");

  RCLCPP_INFO(this->get_logger(), "Initalize Subscriber...");

  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/ipir/cmd_vel", 10, std::bind(&IpirWheelControlNode::velocityCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Ready. Waiting for Twist messages (/cmd_vel) ...");

}

IpirWheelControlNode::~IpirWheelControlNode()
{
  uint8_t dxl_error;
  for (int id : {6, 7, 8, 9}) {
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  }
  portHandler_->closePort();
}

void IpirWheelControlNode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear = msg->linear.x;
  double angular = msg->angular.x;

  int16_t linear_vel = static_cast<int16_t>(linear * VELOCITY_MULTIPLY);
  int16_t angular_vel = static_cast<int16_t>(linear * VELOCITY_MULTIPLY);

  if (std::abs(linear) > 1e-3) {
    for (int id : {6, 7, 8}) {
      sendVelocity(id, linear_vel);
      // sendVelocity(id, VELOCITY_MAX);
    }
  } else {
    for (int id : {6, 7, 8}) {
      sendVelocity(id, 0);
    }
  }

  if (std::abs(angular) > 1e-3) {
    sendVelocity(9, angular_vel);
    // sendVelocity(9, VELOCITY_MAX);
  } else {
    sendVelocity(9, 0);
  }
}

void IpirWheelControlNode::sendVelocity(int id, int16_t velocity)
{
  uint8_t dxl_error;
  int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to send velocity: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    return;
  }

  int16_t current = 0;
  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_PRESENT_CURRENT, (uint16_t*)&current, &dxl_error);

  auto now = std::chrono::steady_clock::now();
  if (last_log_time.find(id) == last_log_time.end()) {
    last_log_time[id] = now - std::chrono::seconds(1);
  }

  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time[id]).count() >= 1) {
    RCLCPP_INFO(this->get_logger(), "[ID:%d] Send Velocity  : %d", id, velocity);   // TODO: change to [m/s] or [rad/s]
    RCLCPP_INFO(this->get_logger(), "[ID:%d] Current Current: %d [mA]", id, current);
    last_log_time[id] = now;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IpirWheelControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}