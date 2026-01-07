#include "inpipe_robot/ipir_odometry.hpp"


IpirOdometryNode::IpirOdometryNode() : Node("ipir_odometry_node")
{
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/ipir/wheel_odom", 10);
  motor_state_subscriber_ = create_subscription<inpipe_robot_interfaces::msg::MotorStates>(
    "/ipir/motor_states", 10,
    std::bind(&IpirOdometryNode::motorCallback, this, std::placeholders::_1));
  imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
    "/camera/imu", 10,
    std::bind(&IpirOdometryNode::imuCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Initalized.");

  x_ = y_ = theta_ = roll_ = 0.0;
  imu_yaw_ = imu_roll_ = 0.0;
  last_time_ = now();
  got_motor_ = got_imu_ = false;

  RCLCPP_INFO(this->get_logger(), "Start !");
}

IpirOdometryNode::~IpirOdometryNode() = default;

void IpirOdometryNode::motorCallback(const inpipe_robot_interfaces::msg::MotorStates::SharedPtr msg)
{

  v_front_ = msg->velocity_mps;
  v_mid_   = msg->velocity_mps;
  v_rear_  = msg->velocity_mps;
  // v_roll_  = msg->velocity_mps;
  RCLCPP_INFO(this->get_logger(), "Recieved velocity: %f [m/s]", msg->velocity_mps);

  got_motor_ = true;

  tryComputeOdom();
}

void IpirOdometryNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->orientation.x, msg->orientation.y,
    msg->orientation.z, msg->orientation.w);

  tf2::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  imu_roll_  = r;
  // imu_pitch_ = p;
  imu_yaw_   = y;
  got_imu_ = true;
  tryComputeOdom();
}

void IpirOdometryNode::tryComputeOdom()
{
  if (!got_motor_ || !got_imu_) return; // Waiting for sensor data

  auto now_time = now();
  double dt = (now_time - last_time_).seconds();
  if (dt < 1e-4) return;

  last_time_ = now_time;

  // 1. Odometry dead-reckoning + IMU yaw
  double vx = (v_front_ - v_mid_ + v_rear_) / 3.0;
  theta_ = imu_yaw_;
  roll_  = imu_roll_;
  // phi_   = imu_pitch_;

  x_ += vx * std::cos(theta_) * dt;
  y_ += vx * std::sin(theta_) * dt;

  // 2. Pub msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "camera_link";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(roll_, 0.0, theta_);
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = 0.0;

  odom_publisher_->publish(odom);
  got_motor_ = got_imu_ = false; // Waiting for next feedback
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IpirOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}