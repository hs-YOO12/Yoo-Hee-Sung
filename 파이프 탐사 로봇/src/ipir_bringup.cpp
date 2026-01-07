/**
 * Author: Geonwoo Kho
 */
#include "inpipe_robot/ipir_bringup.hpp"

#include <cmath>
#include <chrono>

#define DEVICENAME "/dev/ttyUSB0"
#define BAUDRATE   57600
#define PROTOCOL_VERSION 2.0

// Control Table Address
#define VELOCITY_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 3
#define ADDR_OPERATING_MODE   11
// #define VOLTAGE_CONTROL_MODE  16
#define ADDR_TORQUE_ENABLE    64
#define ADDR_POSITION_P_GAIN  84
// #define ADDR_GOAL_PWM         100
#define ADDR_GOAL_VELOCITY    104
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_LOAD     126
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132

#define WHEEL_VELOCITY        16
// #define LOAD_UPPER            500
// #define LOAD_LOWER            100
// #define PWM_STEP              20


// ========================== Constructor
IpirBringupNode::IpirBringupNode() : Node("ipir_bringup")
{
  RCLCPP_INFO(this->get_logger(), "\033[1;34m#########################################");
  RCLCPP_INFO(this->get_logger(), "\033[1;34m###                Modes              ###");
  RCLCPP_INFO(this->get_logger(), "\033[1;34m###                                   ###");
  RCLCPP_INFO(this->get_logger(), "\033[1;34m###      1: Velocity control mode     ###");
  RCLCPP_INFO(this->get_logger(), "\033[1;34m###      3: Position control mode     ###");
  RCLCPP_INFO(this->get_logger(), "\033[1;34m#########################################\n");

  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
  RCLCPP_INFO(this->get_logger(), "\033[1;32m###     Initialize bringup node...    ###");
  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################\n");

  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
  RCLCPP_INFO(this->get_logger(), "\033[1;32m###     Initialize Dynamixels...      ###");
  portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler_->openPort()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open the port!");
    throw std::runtime_error("Failed to open port");
  } else RCLCPP_INFO(this->get_logger(), "\033[1;32m###     \033[0mPort opened: \033[1;32m%s     ###", DEVICENAME);
  if (!portHandler_->setBaudRate(BAUDRATE)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to set the baudrate!");
    throw std::runtime_error("Failed to set baudrate");
  } else
  RCLCPP_INFO(this->get_logger(), "\033[1;32m###     \033[0mBaudrate set: \033[1;32m%d           ###", BAUDRATE);

  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
  uint8_t dxl_error = 0;
  for (int id = 0; id <= 8; id++) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
    RCLCPP_INFO(this->get_logger(), "\033[1;32m###        [Dynamixel ID: \033[0m%d\033[1;32m]          ###", id);
    int mode = (id >= 5) ? VELOCITY_CONTROL_MODE : POSITION_CONTROL_MODE;
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);  // Torque disable
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, mode, &dxl_error);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m###          - \033[0mOperating mode: \033[1;32m%d      ###", mode);
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m###          - \033[0mTorque enabled.        \033[1;32m###");
  }

  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
  RCLCPP_INFO(this->get_logger(), "\033[1;32m########    KINEMATIC CONSTANTS   #######");
  // Kinematic constants
  D_p_ = 135.0; // TODO
  R_w_ = 58.0;
  W_R_ = 100.0;
  H_w_ = std::sqrt(D_p_ * D_p_ - W_R_ * W_R_) + 10;
  H_shaft_ = H_w_ - R_w_;
  R_H_o_ = D_p_ - 0.5 * (D_p_ - H_w_);
  R_H_i_ = R_H_o_ - H_w_;
  R_p_ = 204.0; // TODO : default 205
  x_vel_ = 0.229 * 30;
  dt_ = 0.0001;
  // dt_ = 0.1;
  L_.push_back(43.0);
  L_.push_back(136.0);
  L_.push_back(136.0);
  L_.push_back(136.0);
  L_.push_back(136.0);
  L_.push_back(93.0);

  RCLCPP_INFO(this->get_logger(), "\033[0;32mD_p_     :\t%f", D_p_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mR_w_     :\t%f", R_w_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mW_R_     :\t%f", W_R_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mH_w_     :\t%f", H_w_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mH_shaft_ :\t%f", H_shaft_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mR_H_o_   :\t%f", R_H_o_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mR_H_i_   :\t%f", R_H_i_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mR_p_     :\t%f", R_p_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mx_vel_   :\t%f [m/s]", x_vel_);
  for (int i = 0; i < (int)L_.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "\033[0;32mL_%d : %f\t", i, L_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");

  //
  // groupSyncWriteCor_ = std::make_unique<dynamixel::GroupSyncWrite>(
  //   portHandler_, packetHandler_, ADDR_GOAL_POSITION, 4);
  groupSyncWritePos_ = std::make_unique<dynamixel::GroupSyncWrite>(
    portHandler_, packetHandler_, ADDR_PROFILE_VELOCITY, 8);
  groupSyncWriteVel_ = std::make_unique<dynamixel::GroupSyncWrite>(
    portHandler_, packetHandler_, ADDR_GOAL_VELOCITY, 4);

  reset_srv_ = this->create_service<std_srvs::srv::Empty>(
    "/ipir/reset",
    std::bind(&IpirBringupNode::handleDxlReset, this, std::placeholders::_1, std::placeholders::_2));

  get_position_srv_ = this->create_service<inpipe_robot_interfaces::srv::GetPosition>(
    "/ipir/get_position",
    std::bind(&IpirBringupNode::getPositionCallback, this, std::placeholders::_1, std::placeholders::_2));

  wheel_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/ipir/cmd_vel", 10,
    std::bind(&IpirBringupNode::velocityCallback, this, std::placeholders::_1));

  set_joint_positions_server_ = rclcpp_action::create_server<inpipe_robot_interfaces::action::SetJointPositions>(
    this,
    "/ipir/set_joint_positions",
    std::bind(&IpirBringupNode::setJointPositionsHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&IpirBringupNode::setJointPositionsHandleCancel, this, std::placeholders::_1),
    std::bind(&IpirBringupNode::setJointPositionsHandleAccepted, this, std::placeholders::_1));

  inpipe_motion_server_ = rclcpp_action::create_server<inpipe_robot_interfaces::action::InpipeMotion>(
    this,
    "/ipir/inpipe_motion",
    std::bind(&IpirBringupNode::inpipeMotionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&IpirBringupNode::inpipeMotionHandleCancel, this, std::placeholders::_1),
    std::bind(&IpirBringupNode::inpipeMotionHandleAccepted, this, std::placeholders::_1));

  corner_motion_server_ = rclcpp_action::create_server<inpipe_robot_interfaces::action::CornerMotion>(
    this,
    "/ipir/corner_motion",
    std::bind(&IpirBringupNode::cornerMotionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&IpirBringupNode::cornerMotionHandleCancel, this, std::placeholders::_1),
    std::bind(&IpirBringupNode::cornerMotionHandleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "\033[1;32mBringup node started, all subscribers and services are ready.\n");
}
// Constructor ==========================


// ========================== Destructor
IpirBringupNode::~IpirBringupNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutdown the node...");
  uint8_t dxl_error;
  for (int id = 0; id <= 8; id++) {
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  }
  portHandler_->closePort();
}

void IpirBringupNode::handleDxlReset(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req, (void)res;
  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
  uint8_t dxl_error = 0;
  RCLCPP_INFO(this->get_logger(), "\033[1;32m##############  Reboot   ################");
  for (int id = 0; id <= 8; id++) {
    int dxl_comm_result = packetHandler_->reboot(portHandler_, id);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
    } else {
      RCLCPP_INFO(this->get_logger(), "\033[1;32m###            Reboot [%d]            ####", id);
    }
  }
  portHandler_->closePort();

  if (!portHandler_->openPort()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open the port!");
    throw std::runtime_error("Failed to open port");
  } else RCLCPP_INFO(this->get_logger(), "\033[1;32m###     \033[0mPort opened: \033[1;32m%s     ###", DEVICENAME);
  if (!portHandler_->setBaudRate(BAUDRATE)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to set the baudrate!");
    throw std::runtime_error("Failed to set baudrate");
  } else
  RCLCPP_INFO(this->get_logger(), "\033[1;32m###     \033[0mBaudrate set: \033[1;32m%d           ###", BAUDRATE);

  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
  for (int id = 0; id <= 8; id++) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
    RCLCPP_INFO(this->get_logger(), "\033[1;32m###        [Dynamixel ID: \033[0m%d\033[1;32m]          ###", id);
    int mode = (id >= 5) ? VELOCITY_CONTROL_MODE : POSITION_CONTROL_MODE;
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);  // Torque disable
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, mode, &dxl_error);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m###          - \033[0mOperating mode: \033[1;32m%d      ###", mode);
    packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m###          - \033[0mTorque enabled.        \033[1;32m###");
  }
  RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");

  // res->
  // for (int id = 0; id <= 8; id++) {
  //   RCLCPP_INFO(this->get_logger(), "\033[1;32m###############  Reset  #################");
  //   RCLCPP_INFO(this->get_logger(), "\033[1;32m###        [Dynamixel ID: \033[0m%d\033[1;32m]          ###", id);
  //   int mode = (id >= 5) ? VELOCITY_CONTROL_MODE : POSITION_CONTROL_MODE;
  //   packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);  // Torque disable
  //   packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, mode, &dxl_error);
  //   RCLCPP_INFO(this->get_logger(), "\033[1;32m###          - \033[0mOperating mode: \033[1;32m%d      ###", mode);
  //   packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  //   RCLCPP_INFO(this->get_logger(), "\033[1;32m###          - \033[0mTorque enabled.        \033[1;32m###");
  // }
  // RCLCPP_INFO(this->get_logger(), "\033[1;32m#########################################");
}

// ===================================================================
std::pair<double, double> IpirBringupNode::o_func(double x) {
  double r = - R_p_ / 1.414;

  if (x < r) {
    double y = - x + 20;
    return {x, y};
  } else if (x < -r) {
    double y = (1.414 * R_p_) - std::sqrt((R_p_ * R_p_) - (x * x));
    return {x, y};
  } else {
    double y = x + 20;
    return {x, y};
  }
}

std::pair<double, double> IpirBringupNode::i_func(double x) {
  double dhr = H_shaft_ - R_p_ + 20;
  double r = dhr / 1.414;

  if (x < r) {
    double y = (1.414 * (H_shaft_ + 20)) - x;
    return {x, y};
  } else if (x < -r) {
    double y = (R_p_ * 1.414) - std::sqrt((dhr * dhr) - (x * x));
    return {x, y};
  } else {
    double y = (1.414 * (H_shaft_ + 20)) + x;
    return {x, y};
  }
}

std::pair<double, double> IpirBringupNode::c_func(double x) {
  double dhr = (((H_shaft_+20) - (2.0 * R_p_)) / 2.0);
  double r = dhr / 1.414;

  if (x < r) {
    double y = ((H_shaft_+20) / 1.414) - x;
    return {x, y};
  } else if (x < -r) {
    double y = (1.414 * R_p_) - std::sqrt((dhr * dhr) - (x * x));
    return {x, y};
  } else {
    double y = ((H_shaft_+20) / 1.414) + x;
    return {x, y};
  }
}

std::pair<double, double> IpirBringupNode::r_func(double x) {
  double dhr = (D_p_ + (H_shaft_+20) - R_w_);
  double r = ((dhr - (2.0 * R_p_)) / (2.0 * 1.414));

  if (x < r) {
    double y = (dhr / 1.414) - x;
    return {x, y};
  } else if (x < -r) {
    double y = (R_p_ * 1.414) - std::sqrt(((dhr - (2.0 * R_p_)) / 2.0) * ((dhr - (2.0 * R_p_)) / 2.0) - (x * x));
    return {x, y};
  } else {
    double y = (dhr / 1.414) + x;
    return {x, y};
  }
}

std::pair<double, double> IpirBringupNode::find_next_point(
  std::pair<double, double> prev,
  std::function<std::pair<double, double>(double)> func,
  double dist, double step
) {
  double x0 = prev.first;
  double y0 = prev.second;
  for (double dx = step; dx < 300.0; dx += step) {
    double x_candidate = x0 - dx;
    double y_candidate = func(x_candidate).second;
    if (std::isnan(y_candidate)) continue;
    double d = std::hypot(x_candidate - x0, y_candidate - y0);
    if (std::abs(d - dist) < 1) {
      return {x_candidate, y_candidate};
    }
  }
  return {NAN, NAN};
}

std::pair<double, double> IpirBringupNode::half_o_func(double x) {

  if (x < 0) {
    double y = 0;
    return {x, y};
  } else if (x >= 0 && x < -(R_p_ / 1.414)) {
    double y = -std::sqrt((R_p_ * R_p_) - (x * x)) + R_p_;
    return {x, y};
  } else {
    double y = x + R_p_ * (1 - 1.414);
    return {x, y};
  }
}

std::pair<double, double> IpirBringupNode::half_c_func(double x) {
  double Half_shaft = H_shaft_ / 2.0;

  if (x < 0) {
    double y = (H_shaft_ / 2.0);
    return {x, y};
  } else if (x >= 0 && x < ((R_p_ - Half_shaft) / 2.0)) {
    double y = -std::sqrt(((R_p_ - Half_shaft) * (R_p_ - Half_shaft)) - (x * x)) + R_p_;
    return {x, y};
  } else {
    double y = x + R_p_ - ((R_p_ - Half_shaft) * 1.414);
    return {x, y};
  }
}

std::pair<double, double> IpirBringupNode::half_i_func(double x) {
  double i = (R_p_ - H_shaft_);

  if (x < 0) {
    double y = H_shaft_;
    return {x, y};
  } else if (x < i / 1.414) {
    double y = -std::sqrt((i * i) - (x * x)) + R_p_;
    return {x, y};
  } else {
    double y = x + R_p_ - (i * 1.414);
    return {x, y};
  }
}

std::pair<double, double> IpirBringupNode::half_r_func(double x) {
  double i = D_p_ + H_shaft_ - R_w_;
  double r = R_p_ - ((D_p_ + H_shaft_ - R_w_) / (2));

  if (x < 0) {
    double y = i / 2.0;
    return {x, y};
  } else if (x >= 0 && x < (r / 1.414)) {
    double y = -std::sqrt((r * r) - (x * x));
    return {x, y};
  } else {
    double y = x + R_p_ - (r * 1.414);
    return {x, y};
  }
}

std::pair<double, double> IpirBringupNode::half_find_next_point(
  std::pair<double, double> prev,
  std::function<std::pair<double, double>(double)> func,
  double dist, double step
) {
  double x0 = prev.first;
  double y0 = prev.second;
  for (double dx = step; dx < 300.0; dx += step) {
    double x_candidate = x0 - dx;
    double y_candidate = func(x_candidate).second;
    if (std::isnan(y_candidate)) continue;
    double d = std::hypot(x_candidate - x0, y_candidate - y0);
    if (std::abs(d - dist) < 1) {
      return {x_candidate, y_candidate};
    }
  }
  return {NAN, NAN};
}
// ===================================================================
double IpirBringupNode::calc_theta(
  std::pair<double, double> p0,
  std::pair<double, double> p1,
  std::pair<double, double> p2
) {
  double ux = p1.first - p0.first;
  double uy = p1.second - p0.second;
  double vx = p2.first - p1.first;
  double vy = p2.second - p1.second;

  double dot = ux * vx + uy * vy;
  double mag_u = std::hypot(ux, uy);
  double mag_v = std::hypot(vx, vy);

  RCLCPP_INFO(this->get_logger(), "\033[1;32m === dot  : %f", dot);
  RCLCPP_INFO(this->get_logger(), "\033[1;32m === mag_u: %f", mag_u);
  RCLCPP_INFO(this->get_logger(), "\033[1;32m === mag_v: %f", mag_v);

  double res = std::acos(dot / (mag_u * mag_v));
  RCLCPP_INFO(this->get_logger(), "\033[1;32m === calc_theta: %f", res);
  // return std::acos(dot / (mag_u * mag_v));
  return res;
}

double IpirBringupNode::calc_signed_theta(
  std::pair<double, double> p0,
  std::pair<double, double> p1,
  std::pair<double, double> p2
) {
  double ux = p1.first - p0.first;
  double uy = p1.second - p0.second;
  double vx = p2.first - p1.first;
  double vy = p2.second - p1.second;

  double dot = ux * vx + uy * vy;
  double cross = ux * vy - uy * vx;
  double mag_u = std::hypot(ux, uy);
  double mag_v = std::hypot(vx, vy);

  if (mag_u == 0 || mag_v == 0) return 0.0;

  double angle = std::atan2(cross, dot);

  return angle;
}

void IpirBringupNode::getPositionCallback(
  const std::shared_ptr<inpipe_robot_interfaces::srv::GetPosition::Request> request,
  std::shared_ptr<inpipe_robot_interfaces::srv::GetPosition::Response> response)
{
  std::lock_guard<std::mutex> lock(dxl_mutex_);

  dxl_comm_result = packetHandler_->read4ByteTxRx(
    portHandler_, static_cast<uint8_t>(request->id),
    ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&present_position), &dxl_error
  );

  RCLCPP_INFO(this->get_logger(),
    "Get [ID: %d] [Present Position: %d]", request->id, present_position);

    response->position = present_position;
}
// Destructor ==========================


// ========================== Topic msg callback
void IpirBringupNode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear = msg->linear.x;
  double angular = msg->angular.x;

  for (int id : {5, 6, 7}) sendVelocity(id, std::abs(linear) > 1e-3 ? linear : 0);
  sendVelocity(8, std::abs(angular) > 1e-3 ? angular : 0);
}

void IpirBringupNode::publishMotorStates()
{
  // std::lock_guard<std::mutex> lock(dxl_mutex_);
  // inpipe_robot_interfaces::msg::MotorStates msg;
  // msg.header.stamp = this->now();

  // std::vector<int> wheel_ids = {5, 6, 7, 8};
  // msg.ids = wheel_ids;

  // for (int id : wheel_ids) {
  //   uint8_t dxl_error = 0;
  //   int dxl_comm_result = 0;
  //   uint32_t position = 0;
  //   int32_t velocity = 0;

  //   dxl_comm_result = packetHandler_->read4ByteTxRx(
  //     portHandler_, id, ADDR_PRESENT_POSITION, &position, &dxl_error);
  //   if (dxl_comm_result != COMM_SUCCESS) position = 0;
  //   msg.present_position.push_back(position);

  //   dxl_comm_result = packetHandler_->read4ByteTxRx(
  //     portHandler_, id, ADDR_PRESENT_VELOCITY, (uint32_t*)&velocity, &dxl_error);
  //   if (dxl_comm_result != COMM_SUCCESS) velocity = 0;
  //   msg.present_velocity.push_back(velocity);

  //   double rpm = (double)velocity * 0.229;
  //   double wheel_rad = 0.058;
  //   double mps = (rpm * 2.0 * M_PI * wheel_rad) / 60.0;
  //   msg.velocity_mps.push_back(mps);
  // }
  // motor_states_pub_->publish(msg);
}
// Topic msg callback ==========================


// ========================== Utilities
void IpirBringupNode::sendVelocity(int id, int16_t velocity)
{
  std::lock_guard<std::mutex> lock(dxl_mutex_);

  uint8_t dxl_error;
  int dxl_comm_result;

  if (velocity > 0) {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_VELOCITY, -WHEEL_VELOCITY, &dxl_error);
  } else if (velocity < 0) {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_VELOCITY, WHEEL_VELOCITY, &dxl_error);
  } else {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  }

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "[ID:%d] Failed to send velocity: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    return;
  }
  int16_t current = 0;

  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_PRESENT_LOAD, (uint16_t*)&current, &dxl_error);

  auto now = std::chrono::steady_clock::now();
  if (last_log_time.find(id) == last_log_time.end()) last_log_time[id] = now - std::chrono::seconds(1);

  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time[id]).count() >= 1) {
    RCLCPP_INFO(this->get_logger(), "[ID:\033[1;34m%d\033[0m] Send Velocity  : \033[1;34m%d\033[0m", id, velocity);
    RCLCPP_INFO(this->get_logger(), "[ID:\033[1;34m%d\033[0m] Current Current: \033[1;34m%d\033[0m [mA]", id, current);
    last_log_time[id] = now;
  }
}

void IpirBringupNode::calcJointPositions(
  const double raw_goal_position,
  std::vector<double>& goal_positions)
{
  const double L = 135.0;     // default: 136.0
  const double L_4 = 47.0;    // default: 64.2
  const double W_R = 100.0;   // default: 113.0 | real robot: 110.0
  const double R_w = 56.0;

  double D_P = std::abs(raw_goal_position);
  RCLCPP_INFO(this->get_logger(), "command[D_P]:\t%f", D_P);
  double H_w = std::abs(std::sqrt((D_P * D_P) - (W_R * W_R)));
  RCLCPP_INFO(this->get_logger(), "H_w:\t\t\t%f", H_w);
  double H_shaft = H_w - R_w;
  RCLCPP_INFO(this->get_logger(), "H_shaft:\t\t%f", H_shaft);
  RCLCPP_INFO(this->get_logger(), "=================================");
  double phi_1_c           = ((D_P + H_w) / 2.0) - R_w;
  double phi_1_numerator   = std::sqrt((L * L) - (phi_1_c * phi_1_c));
  double phi_1_denominator = L;
  double phi_1_rad         = std::acos(phi_1_numerator / phi_1_denominator);
  double theta_1_deg       = std::abs(phi_1_rad) * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(), "phi_1_numerator:\t%f", phi_1_numerator);
  RCLCPP_INFO(this->get_logger(), "phi_1_denominator:\t%f", phi_1_denominator);
  RCLCPP_INFO(this->get_logger(), "phi_1_rad:\t\t%f", phi_1_rad);
  RCLCPP_INFO(this->get_logger(), "theta_1_deg:\t\t%f", theta_1_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");
  double phi_2_numerator   = std::sqrt((L * L) - (H_shaft * H_shaft));
  double phi_2_denominator = L;
  double phi_2_rad         = std::acos(phi_2_numerator / phi_2_denominator);
  double phi_2_deg         = std::abs(phi_2_rad) * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(), "phi_2_numerator:\t%f", phi_2_numerator);
  RCLCPP_INFO(this->get_logger(), "phi_2_denominator:\t%f", phi_2_denominator);
  RCLCPP_INFO(this->get_logger(), "phi_2_rad:\t\t%f", phi_2_rad);
  RCLCPP_INFO(this->get_logger(), "phi_2_deg:\t\t%f", phi_2_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");
  double theta_2_deg = -(theta_1_deg + phi_2_deg);
  double theta_3_deg = (2.0 * phi_2_deg);
  RCLCPP_INFO(this->get_logger(), "theta_2_deg:\t\t%f", theta_2_deg);
  RCLCPP_INFO(this->get_logger(), "theta_3_deg:\t\t%f", theta_3_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");
  double half_H_saft       = H_shaft / 2.0;
  double phi_4_numerator   = std::sqrt((L_4 * L_4) - (half_H_saft * half_H_saft));
  double phi_4_denominator = L_4;
  double phi_4_rad         = std::acos(phi_4_numerator / phi_4_denominator);
  double phi_4_deg         = std::abs(phi_4_rad) * 180.0 / M_PI;
  double theta_4_deg       = -(phi_2_deg + phi_4_deg);
  double theta_5_deg       = -(phi_4_deg);
  RCLCPP_INFO(this->get_logger(), "phi_4_numerator:\t%f", phi_4_numerator);
  RCLCPP_INFO(this->get_logger(), "phi_4_denominator:\t%f", phi_4_denominator);
  RCLCPP_INFO(this->get_logger(), "phi_4_rad:\t\t%f", phi_4_rad);
  RCLCPP_INFO(this->get_logger(), "phi_4_deg:\t\t%f", phi_4_deg);
  RCLCPP_INFO(this->get_logger(), "theta_4_deg:\t\t%f", theta_4_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");

  double angle[5] = {
    -theta_5_deg * (0.75),
    theta_4_deg,
    theta_3_deg * (1.25),
    theta_2_deg * (1.4),
    -theta_1_deg * (0.7)
  };

  for (int i = 0; i < 5; i++) {
    goal_positions.push_back(angle[i]);
  }

  for (int i=0; i<5; i++) {
    RCLCPP_INFO(this->get_logger(), "angles[%d]:\t%f", i, goal_positions[i]);
  }
}

void IpirBringupNode::calcJointPositionsWeight(
  const double raw_goal_position,
  std::vector<double>& goal_positions,
  std::vector<double>& weights)
{
  const double L = 135.0;     // default: 136.0
  const double L_4 = 62.0;    // default: 64.2
  const double W_R = 100.0;   // default: 113.0 | real robot: 110.0
  const double R_w = 56.0;

  double D_P = std::abs(raw_goal_position);
  RCLCPP_INFO(this->get_logger(), "command[D_P]:\t%f", D_P);
  double H_w = std::abs(std::sqrt((D_P * D_P) - (W_R * W_R)));
  RCLCPP_INFO(this->get_logger(), "H_w:\t\t\t%f", H_w);
  double H_shaft = H_w - R_w;
  RCLCPP_INFO(this->get_logger(), "H_shaft:\t\t%f", H_shaft);
  RCLCPP_INFO(this->get_logger(), "=================================");
  double phi_1_c           = ((D_P + H_w) / 2.0) - R_w;
  double phi_1_numerator   = std::sqrt((L * L) - (phi_1_c * phi_1_c));
  double phi_1_denominator = L;
  double phi_1_rad         = std::acos(phi_1_numerator / phi_1_denominator);
  double theta_1_deg       = std::abs(phi_1_rad) * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(), "phi_1_numerator  :\033[1;32m\t%f", phi_1_numerator);
  RCLCPP_INFO(this->get_logger(), "phi_1_denominator:\033[1;32m\t%f", phi_1_denominator);
  RCLCPP_INFO(this->get_logger(), "phi_1_rad        :\033[1;32m\t%f", phi_1_rad);
  RCLCPP_INFO(this->get_logger(), "theta_1_deg      :\033[1;32m\t%f", theta_1_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");
  double phi_2_numerator   = std::sqrt((L * L) - (H_shaft * H_shaft));
  double phi_2_denominator = L;
  double phi_2_rad         = std::acos(phi_2_numerator / phi_2_denominator);
  double phi_2_deg         = std::abs(phi_2_rad) * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(), "phi_2_numerator  :\033[1;32m\t%f", phi_2_numerator);
  RCLCPP_INFO(this->get_logger(), "phi_2_denominator:\033[1;32m\t%f", phi_2_denominator);
  RCLCPP_INFO(this->get_logger(), "phi_2_rad        :\033[1;32m\t%f", phi_2_rad);
  RCLCPP_INFO(this->get_logger(), "phi_2_deg        :\033[1;32m\t%f", phi_2_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");
  double theta_2_deg = -(theta_1_deg + phi_2_deg);
  double theta_3_deg = (2.0 * phi_2_deg);
  RCLCPP_INFO(this->get_logger(), "theta_2_deg      :\033[1;32m\t%f", theta_2_deg);
  RCLCPP_INFO(this->get_logger(), "theta_3_deg      :\033[1;32m\t%f", theta_3_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");
  double half_H_saft       = H_shaft / 2.0;
  double phi_4_numerator   = std::sqrt((L_4 * L_4) - (half_H_saft * half_H_saft));
  double phi_4_denominator = L_4;
  double phi_4_rad         = std::acos(phi_4_numerator / phi_4_denominator);
  double phi_4_deg         = std::abs(phi_4_rad) * 180.0 / M_PI;
  double theta_4_deg       = -(phi_2_deg + phi_4_deg);
  double theta_5_deg       = -(phi_4_deg);
  RCLCPP_INFO(this->get_logger(), "phi_4_numerator  :\033[1;32m\t%f", phi_4_numerator);
  RCLCPP_INFO(this->get_logger(), "phi_4_denominator:\033[1;32m\t%f", phi_4_denominator);
  RCLCPP_INFO(this->get_logger(), "phi_4_rad        :\033[1;32m\t%f", phi_4_rad);
  RCLCPP_INFO(this->get_logger(), "phi_4_deg        :\033[1;32m\t%f", phi_4_deg);
  RCLCPP_INFO(this->get_logger(), "theta_4_deg      :\033[1;32m\t%f", theta_4_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");

  // double angle[5] = {
  //   -theta_5_deg * (0.75),
  //   theta_4_deg,
  //   theta_3_deg * (1.25),
  //   theta_2_deg * (1.4),
  //   -theta_1_deg * (0.7)
  // };

  double angle[5] = {
    -theta_5_deg * weights[0],
    theta_4_deg * weights[1],
    theta_3_deg * weights[2],
    theta_2_deg * weights[3],
    -theta_1_deg * weights[4]
  };

  for (int i = 0; i < 5; i++) {
    goal_positions.push_back(angle[i]);
  }

  for (int i=0; i<5; i++) {
    RCLCPP_INFO(this->get_logger(), "angles[%d]:\t\033[1;32m%f", i, goal_positions[i]);
  }
}

void IpirBringupNode::calcNewJointPositionsWeight(
  const double raw_goal_position,
  std::vector<double>& goal_positions,
  std::vector<double>& weights)
{
  const double L = 135.0;     // default: 136.0
  const double L_4 = 62.0;    // default: 64.2
  const double W_R = 100.0;   // default: 113.0 | real robot: 110.0
  const double R_w = 56.0;

  double D_P = std::abs(raw_goal_position);
  RCLCPP_INFO(this->get_logger(), "command[D_P]:\t%f", D_P);
  double H_w = std::abs(std::sqrt((D_P * D_P) - (W_R * W_R)));
  RCLCPP_INFO(this->get_logger(), "H_w:\t\t\t%f", H_w);
  double H_shaft = H_w - R_w;
  RCLCPP_INFO(this->get_logger(), "H_shaft:\t\t%f", H_shaft);
  RCLCPP_INFO(this->get_logger(), "=================================");

  double phi_1_rad         = std::asin((D_P - H_shaft - R_w) / (2.0 * L));
  // double phi_1_deg         = std::abs(phi_1_rad) * 180.0 / M_PI;
  double phi_1_deg         = phi_1_rad * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(), "phi_1_rad:\t\t%f", phi_1_rad);
  RCLCPP_INFO(this->get_logger(), "phi_1_deg:\t\t%f", phi_1_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");

  double phi_2_rad         = std::asin(H_shaft / L);
  // double phi_2_deg         = std::abs(phi_2_rad) * 180.0 / M_PI;
  double phi_2_deg         = phi_2_rad * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(), "phi_2_rad:\t\t%f", phi_2_rad);
  RCLCPP_INFO(this->get_logger(), "phi_2_deg:\t\t%f", phi_2_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");

  double phi_3_rad         = phi_2_rad;
  double phi_3_deg         = phi_2_deg;
  RCLCPP_INFO(this->get_logger(), "phi_3_rad:\t\t%f", phi_3_rad);
  RCLCPP_INFO(this->get_logger(), "phi_3_deg:\t\t%f", phi_3_deg);
  RCLCPP_INFO(this->get_logger(), "=================================");

  double phi_4_rad         = std::asin(H_shaft / (2.0 * L_4));
  // double phi_4_deg         = std::abs(phi_4_rad) * 180.0 / M_PI;
  double phi_4_deg         = phi_4_rad * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(), "phi_4_rad:\t\t%f", phi_4_rad);
  RCLCPP_INFO(this->get_logger(), "phi_4_deg:\t\t%f", phi_4_deg);

  double theta_1 = std::abs(phi_1_deg);
  double theta_2 = std::abs(phi_2_deg - theta_1);
  double theta_3 = std::abs(2.0 * phi_2_deg);
  double theta_4 = std::abs(phi_2_deg + phi_4_deg);
  double theta_5 = std::abs(phi_4_deg);
  RCLCPP_INFO(this->get_logger(), "theta_1:\t\t\033[1;32m%f [deg]", theta_1);
  RCLCPP_INFO(this->get_logger(), "theta_2:\t\t\033[1;32m%f [deg]", theta_2);
  RCLCPP_INFO(this->get_logger(), "theta_3:\t\t\033[1;32m%f [deg]", theta_3);
  RCLCPP_INFO(this->get_logger(), "theta_4:\t\t\033[1;32m%f [deg]", theta_4);
  RCLCPP_INFO(this->get_logger(), "theta_5:\t\t\033[1;32m%f [deg]", theta_5);
  RCLCPP_INFO(this->get_logger(), "=================================");

  // double angle[5] = {
  //   -theta_5_deg * (0.75),
  //   theta_4_deg,
  //   theta_3_deg * (1.25),
  //   theta_2_deg * (1.4),
  //   -theta_1_deg * (0.7)
  // };

  double angle[5] = {
     theta_5 * weights[0],
    -theta_4 * weights[1],
     theta_3 * weights[2],
    -theta_2 * weights[3],
     theta_1 * weights[4]
  };

  for (int i = 0; i < 5; i++) {
    goal_positions.push_back(angle[i]);
  }

  for (int i=0; i<5; i++) {
    RCLCPP_INFO(this->get_logger(), "Joint angle [%d]:\033[1;32m\t%f", i, goal_positions[i]);
  }
}
// ========================== Utilities


// ========================== Actions
  // ===================== Inpipe Motion
rclcpp_action::GoalResponse IpirBringupNode::inpipeMotionHandleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const inpipe_robot_interfaces::action::InpipeMotion::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Recieved InpipeMotion goal");
  RCLCPP_INFO(this->get_logger(), "Pipe diameter: \033[1;32m%f [mm]", goal->pipe_diameter);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IpirBringupNode::inpipeMotionHandleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::InpipeMotion>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::REJECT;
}

void IpirBringupNode::inpipeMotionHandleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::InpipeMotion>> goal_handle)
{
  std::thread([this, goal_handle]() {
    auto result = std::make_shared<inpipe_robot_interfaces::action::InpipeMotion::Result>();
    bool all_success = true;
    std::ostringstream message_stream;

    groupSyncWritePos_->clearParam();
    groupSyncWriteVel_->clearParam();

    std::vector<double> goal_positions;
    RCLCPP_INFO(this->get_logger(), "Generate goal_positions.");

    std::vector<double> weights;
    for (int i = 0; i < 5; i++) {
      weights.push_back(goal_handle->get_goal()->angle_weights[i]);
    }

    if (goal_handle->get_goal()->pipe_diameter != 0) {
      calcJointPositionsWeight(goal_handle->get_goal()->pipe_diameter, goal_positions, weights);
      // calcNewJointPositionsWeight(goal_handle->get_goal()->pipe_diameter, goal_positions, weights);
      RCLCPP_INFO(this->get_logger(), "Successfully calculated joint angles!");
    } else {
      for (int i = 0; i < 5; i++) {
        goal_positions.push_back(0.0);
      }
      RCLCPP_INFO(this->get_logger(), "Joint angles are set by initial position!");
    }

    const uint32_t p_vel = goal_handle->get_goal()->profile_velocity;
    for (size_t i = 0; i < 5; i++) {
      uint8_t params[8];
      params[0] = DXL_LOBYTE(DXL_LOWORD(p_vel));
      params[1] = DXL_HIBYTE(DXL_LOWORD(p_vel));
      params[2] = DXL_LOBYTE(DXL_HIWORD(p_vel));
      params[3] = DXL_HIBYTE(DXL_HIWORD(p_vel));

      uint32_t goal_position = 0;
      double command = goal_handle->get_goal()->pipe_diameter;
      uint32_t delta_position = static_cast<uint32_t>(goal_positions[i] * 4095.0 / 360.0);
      // uint32_t center_position = 2048;  // TODO: Make it ADAPTABLE. (Using action msg)

      if (command > 0) {
        goal_position = (raw_center_position_ + delta_position) % 4096;
      } else if (command < 0) {
        goal_position = (raw_center_position_ - delta_position + 4096) % 4096;
      } else {
        goal_position = raw_center_position_;
      }

      params[4] = DXL_LOBYTE(DXL_LOWORD(goal_position));
      params[5] = DXL_HIBYTE(DXL_LOWORD(goal_position));
      params[6] = DXL_LOBYTE(DXL_HIWORD(goal_position));
      params[7] = DXL_HIBYTE(DXL_HIWORD(goal_position));

      groupSyncWritePos_->addParam(goal_handle->get_goal()->tilt_ids[i], params);
    }

    if (groupSyncWritePos_->txPacket() != COMM_SUCCESS) {
      message_stream << "[Position SyncWrite] TxRxError: " << packetHandler_->getTxRxResult(COMM_TX_FAIL) << "\n";
      all_success = false;
    }
    int32_t lx_vel = goal_handle->get_goal()->linear_wheel_velocity;
    int32_t ax_vel = goal_handle->get_goal()->angular_wheel_velocity;

    for (size_t i = 0; i < 4; i++) {
      uint8_t params[4];
      int32_t vel = (i < 3 ? ( i == 1 ? -lx_vel : lx_vel) : ax_vel);
      params[0] = DXL_LOBYTE(DXL_LOWORD(vel));
      params[1] = DXL_HIBYTE(DXL_LOWORD(vel));
      params[2] = DXL_LOBYTE(DXL_HIWORD(vel));
      params[3] = DXL_HIBYTE(DXL_HIWORD(vel));

      groupSyncWriteVel_->addParam(goal_handle->get_goal()->wheel_ids[i], params);
    }
    RCLCPP_INFO(this->get_logger(), "Wheel velocity is set by %d", lx_vel);
    RCLCPP_INFO(this->get_logger(), "Roll velocity is set by %d", ax_vel);

    if (groupSyncWriteVel_->txPacket() != COMM_SUCCESS) {
      message_stream << "[Velocity SyncWrite] TxRxError: " << packetHandler_->getTxRxResult(COMM_TX_FAIL) << "\n";
      all_success = false;
    }

    result->success = all_success;
    result->message = message_stream.str();
    if (all_success) {
      RCLCPP_INFO(this->get_logger(), "All dynamixels moved successfully.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Some dynamixels failed:");
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
    }

    goal_handle->succeed(result);
  }).detach();
}
  // Inpipe Motion =====================

  // ===================== SetJointPositions
rclcpp_action::GoalResponse IpirBringupNode::setJointPositionsHandleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const inpipe_robot_interfaces::action::SetJointPositions::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(this->get_logger(), "Recieved SetJointPositions goal");

  if (goal->ids.size() != 5 || goal->positions.size() != 5) {
    RCLCPP_WARN(this->get_logger(), "Goal size not 5! (ids.size: %zu, positions.size: %zu)",
      goal->ids.size(), goal->positions.size());

    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IpirBringupNode::setJointPositionsHandleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::SetJointPositions>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::REJECT;
}

void IpirBringupNode::setJointPositionsHandleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::SetJointPositions>> goal_handle)
{
  std::thread([this, goal_handle]() {
    auto result = std::make_shared<inpipe_robot_interfaces::action::SetJointPositions::Result>();
    bool all_success = true;
    std::ostringstream message_stream;

    std::vector<double> goal_positions;
    calcJointPositions(goal_handle->get_goal()->positions[0], goal_positions);

    for (size_t i = 0; i < 5; i++) {
      uint8_t dxl_error = 0;
      int dxl_comm_result = 0;
      uint32_t goal_position = 0;

      double command = goal_handle->get_goal()->positions[0];
      // uint32_t delta_position = static_cast<uint32_t>(angles[i] * 4095.0 / 360.0);
      uint32_t delta_position = static_cast<uint32_t>(goal_positions[i] * 4095.0 / 360.0);
      uint32_t center_position = 2048;

      if (command > 0) {
        goal_position = (center_position + delta_position) % 4096;
      } else if (command < 0) {
        goal_position = (center_position - delta_position + 4096) % 4096;
      } else {
        goal_position = center_position;
      }

      dxl_comm_result = packetHandler_->write4ByteTxRx(
        portHandler_, goal_handle->get_goal()->ids[i],
        ADDR_GOAL_POSITION, goal_position, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        message_stream << "[ID:" << goal_handle->get_goal()->ids[i]
                        << "] TxRxError: " << packetHandler_->getTxRxResult(dxl_comm_result) << "; ";
        all_success = false;
      } else if (dxl_error != 0) {
        message_stream << "[ID:" << goal_handle->get_goal()->ids[i]
                        << "] RxError: " << packetHandler_->getRxPacketError(dxl_error) << "; ";
        all_success = false;
      }
    }

    result->success = all_success;
    result->message = message_stream.str();
    if (all_success) {
      RCLCPP_INFO(this->get_logger(), "All dynamixels moved successfully.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Some dynamixels failed: %s", result->message.c_str());
    }

    goal_handle->succeed(result);
  }).detach();
}
  // SetJointPositions =====================

  // ===================== Corner Motion
rclcpp_action::GoalResponse IpirBringupNode::cornerMotionHandleGoal( // TODO: FOR ONLY POSITION CONTROL MODE
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const inpipe_robot_interfaces::action::CornerMotion::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "\033[0;32mRecieved CornerMotion goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IpirBringupNode::cornerMotionHandleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::CornerMotion>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::REJECT;
}

// void IpirBringupNode::cornerMotionHandleAccepted(
//   const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::CornerMotion>> goal_handle)
// {
//   std::thread([this, goal_handle]() {
//     auto result = std::make_shared<inpipe_robot_interfaces::action::CornerMotion::Result>();
//     bool all_success = true;
//     std::ostringstream message_stream;

//     std::vector<double> goal_positions;
//     std::vector<double> inpipe_positions;
//     std::vector<double> weights;
//     for (int i = 0; i < 5; i++) {
//       weights.push_back(goal_handle->get_goal()->angle_weights[i]);
//       goal_positions.push_back(corner_angles_[i] * weights[i]);
//     }
//     if (goal_handle->get_goal()->pipe_diameter != 0) {
//       calcJointPositionsWeight(goal_handle->get_goal()->pipe_diameter, inpipe_positions, weights);
//       // calcNewJointPositionsWeight(goal_handle->get_goal()->pipe_diameter, inpipe_positions, weights);
//       RCLCPP_INFO(this->get_logger(), "Successfully calculated joint angles.");
//     } else {
//       for (int i = 0; i < 5; i++) {
//         goal_positions.push_back(0.0);
//       }
//       RCLCPP_INFO(this->get_logger(), "Joint angles are set by initial position!");
//     }

//     // uint32_t p_vel = goal_handle->get_goal()->profile_velocity;
//     auto tilt_ids = goal_handle->get_goal()->tilt_ids;
//     auto wheel_ids = goal_handle->get_goal()->wheel_ids;
//     int32_t linear_vel = goal_handle->get_goal()->linear_wheel_velocity;
//     int32_t angular_vel = goal_handle->get_goal()->angular_wheel_velocity;

//     std::vector<int> move_times(5);
//     std::vector<int> toc(5);
//     for (int i = 0; i < 5; i++) {
//       double delay_time = goal_handle->get_goal()->delay;
//       move_times[i] = i * delay_time;
//       toc[i] = move_times[i] + goal_handle->get_goal()->freeze;
//     }
//     RCLCPP_INFO(this->get_logger(), "Delay time : %f", goal_handle->get_goal()->delay);
//     RCLCPP_INFO(this->get_logger(), "Freeze time: %f", goal_handle->get_goal()->freeze);

//     std::vector<bool> moved(5, false);
//     std::vector<bool> returned(5, false);
//     rclcpp::Time tic = this->get_clock()->now();
//     rclcpp::Rate loop_rate(10); // 10 Hz

//     while(rclcpp::ok()) {
//       int elapsed = (this->get_clock()->now() - tic).seconds();

//       for (int i = 0; i < 5; i++) {
//         if (!moved[i] && elapsed >= move_times[i]) {
//           double p_vel_delta = ((std::abs(inpipe_positions[i] - goal_positions[i])) / (3 * M_PI));
//           if (p_vel_delta <= 5.0) {
//             RCLCPP_INFO(this->get_logger(), "Calculated Profile velocity is lower then \033[1;32m1");
//             RCLCPP_INFO(this->get_logger(), "Set to \033[1;32m1");
//             p_vel_delta = 1.0;
//           }
//           RCLCPP_INFO(this->get_logger(), "[%d] profile_velocity: \033[1;32m%f -> %f [rpm]", tilt_ids[i], p_vel_delta, (p_vel_delta * 0.229));
//           groupSyncWritePos_->clearParam();

//           uint8_t param[8];
//           param[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)p_vel_delta));
//           param[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)p_vel_delta));
//           param[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)p_vel_delta));
//           param[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)p_vel_delta));

//           int32_t pos = degreeToRaw(goal_positions[i]);
//           param[4] = DXL_LOBYTE(DXL_LOWORD(pos));
//           param[5] = DXL_HIBYTE(DXL_LOWORD(pos));
//           param[6] = DXL_LOBYTE(DXL_HIWORD(pos));
//           param[7] = DXL_HIBYTE(DXL_HIWORD(pos));

//           groupSyncWritePos_->addParam(i, param);
//           groupSyncWritePos_->txPacket();

//           moved[i] = true;
//           RCLCPP_INFO(this->get_logger(), "[%d] moved to %.2f deg.", tilt_ids[i], goal_positions[i]);
//         } else if (!returned[i] && elapsed >= toc[i]) {
//           double p_vel_delta = ((std::abs(inpipe_positions[i] - goal_positions[i])) / (3 * M_PI));
//           if (p_vel_delta <= 1.0) {
//             RCLCPP_INFO(this->get_logger(), "Calculated Profile velocity is lower then \033[1;32m1");
//             RCLCPP_INFO(this->get_logger(), "Set to \033[1;32m1");
//             p_vel_delta = 1.0;
//           }
//           RCLCPP_INFO(this->get_logger(), "[%d] profile_velocity: \033[1;32m%f -> %f [rpm]", tilt_ids[i], p_vel_delta, (p_vel_delta * 0.229));
//           groupSyncWritePos_->clearParam();

//           uint8_t param[8];
//           param[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)p_vel_delta));
//           param[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)p_vel_delta));
//           param[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)p_vel_delta));
//           param[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)p_vel_delta));

//           int32_t pos = degreeToRaw(inpipe_positions[i]);
//           param[4] = DXL_LOBYTE(DXL_LOWORD(pos));
//           param[5] = DXL_HIBYTE(DXL_LOWORD(pos));
//           param[6] = DXL_LOBYTE(DXL_HIWORD(pos));
//           param[7] = DXL_HIBYTE(DXL_HIWORD(pos));

//           groupSyncWritePos_->addParam(i, param);
//           groupSyncWritePos_->txPacket();
//           returned[i] = true;
//           RCLCPP_INFO(this->get_logger(), "[%d] moved to \033[1;32m%.2f deg.", tilt_ids[i], inpipe_positions[i]);
//         }
//       }
//       groupSyncWritePos_->txPacket();

//       if (groupSyncWritePos_->txPacket() != COMM_SUCCESS) {
//       message_stream << "[Position SyncWrite] TxRxError: "
//                      << packetHandler_->getTxRxResult(COMM_TX_FAIL) << "\n";
//       all_success = false;
//       }

//       for (size_t i = 0; i < wheel_ids.size(); ++i) {
//         groupSyncWriteVel_->clearParam();
//         uint8_t params[4];
//         int32_t vel = (i < 3) ? ( i == 1 ? -linear_vel : linear_vel) : angular_vel;
//         params[0] = DXL_LOBYTE(DXL_LOWORD(vel));
//         params[1] = DXL_HIBYTE(DXL_LOWORD(vel));
//         params[2] = DXL_LOBYTE(DXL_HIWORD(vel));
//         params[3] = DXL_HIBYTE(DXL_HIWORD(vel));
//         groupSyncWriteVel_->addParam(wheel_ids[i], params);
//       }

//       if (groupSyncWriteVel_->txPacket() != COMM_SUCCESS) {
//         message_stream << "[Velocity SyncWrite] TxRxError: "
//                        << packetHandler_->getTxRxResult(COMM_TX_FAIL) << "\n";
//         all_success = false;
//       }

//       if (std::all_of(returned.begin(), returned.end(), [](bool v){return v;})) {
//         RCLCPP_INFO(this->get_logger(), "Corner motion has completed.");
//         // break;
//         result->success = all_success;
//         result->message= message_stream.str();
//         if (all_success) RCLCPP_INFO(this->get_logger(), "All dynamixels moved successfully.");
//         else RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
//         goal_handle->succeed(result);
//         break;
//       }

//       loop_rate.sleep();
//     }
//   }).detach();
// }
void IpirBringupNode::cornerMotionHandleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::CornerMotion>> goal_handle)
{
  std::thread([this, goal_handle]() {
    RCLCPP_INFO(this->get_logger(), "Corner Motion");
    using clock = std::chrono::high_resolution_clock;
    using namespace std::chrono;

    auto result = std::make_shared<inpipe_robot_interfaces::action::CornerMotion::Result>();
    bool all_success = true;
    std::ostringstream message_stream;

    // D_p_ = goal_handle->get_goal()->pipe_diameter + 1;
    D_p_ = 138.0;
    R_p_ = goal_handle->get_goal()->freeze;
    auto tilt_ids = goal_handle->get_goal()->tilt_ids;
    auto wheel_ids = goal_handle->get_goal()->wheel_ids;
    auto sample_time = goal_handle->get_goal()->delay;
    int32_t angular_vel = goal_handle->get_goal()->angular_wheel_velocity;
    // const double vel_mps = linear_vel * 0.299;
    // const double vel_mmps = vel_mps * 1000.0;
    // const double dx = vel_mmps * dt_;
    // dt_ = 0.000002;

    double x0 = -100; // TODO
    double x_end = 800;
    int steps = static_cast<int>(x_end - x0);
    const double linear_vel = ((double)steps / (goal_handle->get_goal()->linear_wheel_velocity * 0.229));
    // auto delay = steps / sample_time;
    auto delay =  sample_time / steps;
    RCLCPP_INFO(this->get_logger(), "Step time: %3f", delay);
    uint32_t p_vel = goal_handle->get_goal()->profile_velocity;

    // rclcpp::Rate loop_rate(10 / dt_);
    // rclcpp::Rate loop_rate(100000);
    // while(rclcpp::ok() && x0 < 810.0) {
    for (int i = 0; i < steps && rclcpp::ok(); i++) {
      auto start = clock::now();

      groupSyncWritePos_->clearParam();
      std::vector<std::pair<double, double>> points(7);
      std::vector<double> thetas;
      double x_in = x0 / 1.414;
      points[0] = c_func(x_in);  // x0: -100, xend: 800
      points[1] = find_next_point(points[0], [this](double x) { return c_func(x); }, L_[0]);
      points[2] = find_next_point(points[1], [this](double x) { return o_func(x); }, L_[1]);
      points[3] = find_next_point(points[2], [this](double x) { return i_func(x); }, L_[2]);
      points[4] = find_next_point(points[3], [this](double x) { return o_func(x); }, L_[3]);
      points[5] = find_next_point(points[4], [this](double x) { return r_func(x); }, L_[4]);
      points[6] = find_next_point(points[5], [this](double x) { return r_func(x); }, L_[5]);

      // points[0] = half_c_func(x_in);  // x0: -100, xendL
      // points[1] = half_find_next_point(points[0], [this](double x) { return half_c_func(x); }, L_[0]);
      // points[2] = half_find_next_point(points[1], [this](double x) { return half_o_func(x); }, L_[1]);
      // points[3] = half_find_next_point(points[2], [this](double x) { return half_i_func(x); }, L_[2]);
      // points[4] = half_find_next_point(points[3], [this](double x) { return half_o_func(x); }, L_[3]);
      // points[5] = half_find_next_point(points[4], [this](double x) { return half_r_func(x); }, L_[4]);
      // points[6] = half_find_next_point(points[5], [this](double x) { return half_r_func(x); }, L_[5]);

      for (int i = 0; i < 7; i++) {
        RCLCPP_INFO(this->get_logger(), "points[%d]: %f, %f", i, points[i].first, points[i].second);
      }

      if (std::isnan(points[1].first)) {
        RCLCPP_WARN(this->get_logger(), "Corner Motion failed: Nan detected in point trajectory.");
        break;
      }

      for (int i = 0; i < 5; i++) {
        double theta = calc_theta(points[i], points[i + 1], points[i + 2]);
        // double theta = calc_signed_theta(points[i], points[i + 1], points[i + 2]);
        if (!theta) {
          RCLCPP_WARN(this->get_logger(), "Fuck,..: %f", theta);
          break;
        } else {
          thetas.push_back(theta * 180.0 / M_PI);
        }
      }

      for (int i = 0; i < 5; i++) {
        thetas[0]*=-1 * goal_handle->get_goal()->angle_weights[0];
        thetas[1]*=-1 * goal_handle->get_goal()->angle_weights[1];
        thetas[2]*=goal_handle->get_goal()->angle_weights[2];
        thetas[3]*=-1 * goal_handle->get_goal()->angle_weights[3];
        thetas[4]*=-1 * goal_handle->get_goal()->angle_weights[4];
      }

      // for (int i = 0; i < 5; i++) {
      //   thetas[0]*=-1 * goal_handle->get_goal()->angle_weights[0];
      //   thetas[1]*=goal_handle->get_goal()->angle_weights[1];
      //   thetas[2]*=goal_handle->get_goal()->angle_weights[2];
      //   thetas[3]*=goal_handle->get_goal()->angle_weights[3];
      //   thetas[4]*=-1 * goal_handle->get_goal()->angle_weights[4];
      // }

      // for (int i = 0; i < 5; i++) {
      //   thetas[0]*=-1 * goal_handle->get_goal()->angle_weights[0];
      //   thetas[1]*=goal_handle->get_goal()->angle_weights[1];
      //   thetas[2]*=goal_handle->get_goal()->angle_weights[2];
      //   thetas[3]*=goal_handle->get_goal()->angle_weights[3];
      //   thetas[4]*=-1 * goal_handle->get_goal()->angle_weights[4];
      // }


      for (size_t i = 0; i < 5; i++) {
        uint32_t raw_pos = 0;
        raw_pos = degreeToRaw(thetas[i]);
        RCLCPP_INFO(this->get_logger(), "theta[%ld]: %f", i, thetas[i]);

        uint8_t goal_pos[8];
        goal_pos[0] = DXL_LOBYTE(DXL_LOWORD(p_vel));
        goal_pos[1] = DXL_HIBYTE(DXL_LOWORD(p_vel));
        goal_pos[2] = DXL_LOBYTE(DXL_HIWORD(p_vel));
        goal_pos[3] = DXL_HIBYTE(DXL_HIWORD(p_vel));
        goal_pos[4] = DXL_LOBYTE(DXL_LOWORD(raw_pos));
        goal_pos[5] = DXL_HIBYTE(DXL_LOWORD(raw_pos));
        goal_pos[6] = DXL_LOBYTE(DXL_HIWORD(raw_pos));
        goal_pos[7] = DXL_HIBYTE(DXL_HIWORD(raw_pos));
        groupSyncWritePos_->addParam(i, goal_pos);
      }
      groupSyncWritePos_->txPacket();
      // x0 += dx;
      RCLCPP_INFO(this->get_logger(), "x0: %f", x0);
      if (groupSyncWritePos_->txPacket() != COMM_SUCCESS) {
      message_stream << "[Position SyncWrite] TxRxError: "
                      << packetHandler_->getTxRxResult(COMM_TX_FAIL) << "\n";
      all_success = false;
      }

      for (size_t i = 0; i < wheel_ids.size(); ++i) {
        groupSyncWriteVel_->clearParam();
        uint8_t params[4];
        int32_t vel = (i < 3) ? ( i == 1 ? -linear_vel * 0.5 : linear_vel) : angular_vel;
        RCLCPP_INFO(this->get_logger(), "Linear Velocity: %d", vel);
        params[0] = DXL_LOBYTE(DXL_LOWORD(vel));
        params[1] = DXL_HIBYTE(DXL_LOWORD(vel));
        params[2] = DXL_LOBYTE(DXL_HIWORD(vel));
        params[3] = DXL_HIBYTE(DXL_HIWORD(vel));
        groupSyncWriteVel_->addParam(wheel_ids[i], params);
      }

      if (groupSyncWriteVel_->txPacket() != COMM_SUCCESS) {
        message_stream << "[Velocity SyncWrite] TxRxError: "
                        << packetHandler_->getTxRxResult(COMM_TX_FAIL) << "\n";
        all_success = false;
      }

      auto end = clock::now();
      duration<double> dt = end - start;

      RCLCPP_INFO(this->get_logger(), "Step %d | x0 = %.1f | dt = %.4f sec", i, x0, dt.count());

      // loop_rate.sleep();

      auto elapsed_us = duration_cast<microseconds>(end - start).count();
      auto remaining_us = static_cast<int>(delay * 1e6) - elapsed_us;
      if (remaining_us > 0) {
        std::this_thread::sleep_for(microseconds(remaining_us));
        RCLCPP_INFO(this->get_logger(), "remaining: %f", elapsed_us);
      }

      x0 += 1.0;
    }
    if (all_success) {
      RCLCPP_INFO(this->get_logger(), "Corner Motion Action completed.");
      goal_handle->succeed(result);
    } else {
      RCLCPP_WARN(this->get_logger(), "Corner Motion Action failed.");
      goal_handle->abort(result);
    }
  }).detach();
}
  // Corner Motion =====================



// ========================== Main (Run node)
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto bringup = std::make_shared<IpirBringupNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bringup);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}