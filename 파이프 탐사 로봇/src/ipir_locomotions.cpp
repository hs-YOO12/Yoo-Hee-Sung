/**
 * Author: Geonwoo Kho, Chanwoo Seon
 */

#include "inpipe_robot/ipir_locomotions.hpp"

#define DEVICENAME "/dev/ttyUSB0"
#define BAUDRATE   57600
#define PROTOCOL_VERSION 2.0

// Control Table Address
#define VELOCITY_CONTROL_MODE   1
#define POSITION_CONTROL_MODE   3
#define ADDR_OPERATING_MODE     11
#define PROFILE_VELCOCITY_GAIN  40
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_VELOCITY      104
#define ADDR_PROFILE_VELOCITY   112
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_POSITION   132

#define VELOCITY_MAX            256
#define TWIST_VELOCITY          16

#define FREEZE                  3   // [s]


IpirLocomotionNode::IpirLocomotionNode() : Node("ipir_locotion_node")
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler_->openPort()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open the port!");
    throw std::runtime_error("Failed to open port");
  }
  if (!portHandler_->setBaudRate(BAUDRATE)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to set the baudrate!");
    throw std::runtime_error("Failed to set baudrate");
  }

  wheel_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/ipir/cmd_vel", 10,
    std::bind(&IpirLocomotionNode::velocityCallback, this, std::placeholders::_1));

  position_ids_ = {0, 1, 2, 3, 4};
  velocity_ids_ = {5, 6, 7};
  in_pipe_positions_ = {16.45, -27.82, 22.74, -33.02, -21.65};
  target_positions_  = {-35.94, -65.18, 45.28, -65.18, 43.71};
  move_times_ = {3, 4, 5, 6, 7};

  groupSyncWritePos_ = std::make_unique<dynamixel::GroupSyncWrite>(
    portHandler_, packetHandler_, ADDR_PROFILE_VELOCITY, 8);

  groupSyncWriteVel_ = std::make_unique<dynamixel::GroupSyncWrite>(
    portHandler_, packetHandler_, ADDR_GOAL_VELOCITY, 4);

  // for (int id = 0; id < 5; id++) {
  //   enableTorque(id, POSITION_CONTROL_MODE);
  // }

  for (int id = 0; id < 9; id++) {
    int mode = id < 5 ? POSITION_CONTROL_MODE : VELOCITY_CONTROL_MODE;
    setOperatingMode(id, mode);
    enableTorque(id);
  }

  // control_thread_ = std::thread(&IpirLocomotionNode::controlLoop, this);
  control_thread_ = std::thread(&IpirLocomotionNode::syncControlLoop, this);
}

IpirLocomotionNode::~IpirLocomotionNode()
{
  stop_ = true;

  if (control_thread_.joinable()) {
    control_thread_.join();
  }

  portHandler_->closePort();
}

void IpirLocomotionNode::sendVelocity(int id, int16_t velocity)
{
  uint8_t dxl_error;
  int dxl_comm_result;

  if (velocity > 0) {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_VELOCITY, TWIST_VELOCITY, &dxl_error);
  } else if (velocity < 0) {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_VELOCITY, -TWIST_VELOCITY, &dxl_error);
  } else {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  }

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to send velocity: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    return;
  }
}

void IpirLocomotionNode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear = msg->linear.x;
  double angular = msg->angular.x;

  for (int id : {5, 6, 7}) sendVelocity(id, std::abs(linear) > 1e-3 ? linear : 0);
  sendVelocity(8, std::abs(angular) > 1e-3 ? angular : 0);
}

void IpirLocomotionNode::setOperatingMode(uint8_t id, int mode)
{
  uint8_t dxl_error = 0;
  packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  int result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, mode, &dxl_error);
  if (result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode for ID %d: %s", id, packetHandler_->getTxRxResult(result));
  }
}
void IpirLocomotionNode::enableTorque(uint8_t id)
{
  uint8_t dxl_error = 0;
  int result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID %d: %s", id, packetHandler_->getTxRxResult(result));
  }
}


void IpirLocomotionNode::setGoalPosition(uint8_t id, double degree)
{
  int dxl_position = degreeToRaw(degree);
  uint8_t dxl_error = 0;
  int result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_POSITION, dxl_position, &dxl_error);

  if (result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set position for ID %d: %s", id, packetHandler_->getTxRxResult(result));
  } else {
    RCLCPP_INFO(this->get_logger(), "[ID:%d] → Send position: %.2f [deg]", id, degree);
  }
}

void IpirLocomotionNode::syncWritePosition(
  int id,
  uint32_t profile_velocity,
  double goal_positions)
{
  groupSyncWritePos_->clearParam();

  uint8_t param[8];
  param[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity));
  param[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity));
  param[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity));
  param[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity));

  int32_t pos = degreeToRaw(goal_positions);
  param[4] = DXL_LOBYTE(DXL_LOWORD(pos));
  param[5] = DXL_HIBYTE(DXL_LOWORD(pos));
  param[6] = DXL_LOBYTE(DXL_HIWORD(pos));
  param[7] = DXL_HIBYTE(DXL_HIWORD(pos));

  groupSyncWritePos_->addParam(id, param);
  groupSyncWritePos_->txPacket();
}

void IpirLocomotionNode::syncWriteVelocities(
  const std::vector<int>& ids,
  int16_t velocity)
{
  groupSyncWriteVel_->clearParam();
  for (size_t i = 0; i < ids.size(); ++i) {
    uint8_t param[4];
    // int32_t vel = velocities[i];
    param[0] = DXL_LOBYTE(DXL_LOWORD(velocity));
    param[1] = DXL_HIBYTE(DXL_LOWORD(velocity));
    param[2] = DXL_LOBYTE(DXL_HIWORD(velocity));
    param[3] = DXL_HIBYTE(DXL_HIWORD(velocity));
    groupSyncWriteVel_->addParam(ids[i], param);
  }
  groupSyncWriteVel_->txPacket();
}

void IpirLocomotionNode::controlLoop()
{
  rclcpp::Time tic = this->get_clock()->now();

  std::vector<bool> moved(5, false);
  std::vector<bool> returned(5, false);

  std::vector<int> toc(5);
  for (int i = 0; i < 5; i++) {
    toc[i] = move_times_[i] + FREEZE;
  }

  rclcpp::Rate loop_rate(10); // 10 Hz

  while(rclcpp::ok() && !stop_) {
    int elapsed = (this->get_clock()->now() - tic).seconds();

    for (int id = 0; id < 5; id++) {
      if (!moved[id] && elapsed >= move_times_[id]) {
        setGoalPosition(id, target_positions_[id]);
        moved[id] = true;
      } else if (!returned[id] && elapsed >= toc[id]) {
        setGoalPosition(id, in_pipe_positions_[id]);
        returned[id] = true;
      }
    }

    for (int id = 5; id < 8; id++) {
      sendVelocity(id, -TWIST_VELOCITY);
    }

    if (std::all_of(returned.begin(), returned.end(), [](bool status) { return status; })) {
      RCLCPP_INFO(this->get_logger(), "All motions completed.");
      for (int id = 5; id < 8; id++) {
        sendVelocity(id, 0);
      }
      break;
    }

    loop_rate.sleep();
  }
}

void IpirLocomotionNode::syncControlLoop()
{
  rclcpp::Time tic = this->get_clock()->now();

  std::vector<bool> moved(5, false);
  std::vector<bool> returned(5, false);

  std::vector<int> toc(5);
  for (int i = 0; i < 5; i++) {
    toc[i] = move_times_[i] + FREEZE;
  }

  rclcpp::Rate loop_rate(10); // 10 Hz

  while(rclcpp::ok() && !stop_) {
    int elapsed = (this->get_clock()->now() - tic).seconds();

    for (int id = 0; id < 5; id++) {
      if (!moved[id] && elapsed >= move_times_[id]) {
        syncWritePosition(position_ids_[id], PROFILE_VELCOCITY_GAIN, target_positions_[id]);
        moved[id] = true;
        RCLCPP_INFO(this->get_logger(), "[%d] move at %f deg.", position_ids_[id], target_positions_[id]);
      } else if (!returned[id] && elapsed >= toc[id]) {
        syncWritePosition(position_ids_[id], PROFILE_VELCOCITY_GAIN, in_pipe_positions_[id]);
        returned[id] = true;
        RCLCPP_INFO(this->get_logger(), "[%d] moved %f deg.", position_ids_[id], in_pipe_positions_[id]);
      }
    }

    // syncWriteVelocities(velocity_ids_, -TWIST_VELOCITY);

    if (std::all_of(returned.begin(), returned.end(), [](bool status) { return status; })) {
      RCLCPP_INFO(this->get_logger(), "All motions completed.");
      for (int i = 0; i < 5; i++) {
        syncWritePosition(i, PROFILE_VELCOCITY_GAIN, 0.0);
      }
      // syncWriteVelocities(velocity_ids_, 0);
      break;
    }

    loop_rate.sleep();
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IpirLocomotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}