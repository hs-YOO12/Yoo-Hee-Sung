#include "inpipe_robot/ipir_mapper.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

IpirMapperNode::IpirMapperNode() : Node("ipir_mapper")
{
  RCLCPP_INFO(this->get_logger(), "Mapping Node Initializing...");
  d_p_ = 0.0;
  x_ = y_ = z_ = 0.0;
  roll_ = pitch_ = yaw_ = 0.0;
  got_odom_ = false;
  got_d_p_ = false;

  // diameter_sub_ = this->create_subscription<inpipe_robot_interfaces::msg::MotorStates>(
  //   "/ipir/motor_states", 10, std::bind(&IpirMapperNode::recievedDP, this, std::placeholders::_1));

  mapping_config_server_ = rclcpp_action::create_server<inpipe_robot_interfaces::action::MappingConfig>(
    this,
    "/ipir/mapping_config",
    std::bind(&IpirMapperNode::handleMapperGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&IpirMapperNode::handleMapperCancel, this, std::placeholders::_1),
    std::bind(&IpirMapperNode::handleMapperAccepted, this, std::placeholders::_1));


  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10, std::bind(&IpirMapperNode::odomCallback, this, std::placeholders::_1));

  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pipe_map", 1);
  RCLCPP_INFO(this->get_logger(), "\033[1;32mReady to mapping.");
}

// void IpirMapperNode::recievedDP(const inpipe_robot_interfaces::msg::MotorStates::SharedPtr msg)
// {
//   d_p_ = msg->d_p;

//   if (msg->d_p <= 0.0) {
//     RCLCPP_WARN(this->get_logger(), "Received invalid diameter: %.3f", d_p_);
//     got_d_p_ = false;
//     return;
//   }

//   got_d_p_ = true;
// }

rclcpp_action::GoalResponse IpirMapperNode::handleMapperGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const inpipe_robot_interfaces::action::MappingConfig::Goal> goal)
{
  (void)uuid;
  if (goal->diameter < 0.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid diameter: %.3f", goal->diameter);
    return rclcpp_action::GoalResponse::REJECT;
  }
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    mapping_active_ = false;
  }
  got_d_p_ = true;
  RCLCPP_INFO(this->get_logger(), "Recieved goal. Diameter:[%.3f]", goal->diameter);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

  rclcpp_action::CancelResponse IpirMapperNode::handleMapperCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::MappingConfig>> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Recieved request to cancle mapping.");
    (void)goal;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      mapping_active_ = false;
      saveMap("/home/go/ipir/pipemap/map.pcd");
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

// void IpirMapperNode::handleMapperAccepted(
//   const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::MappingConfig>> goal)
// {
//   std::thread([this, goal]() {
//     if (got_d_p_ && got_odom_) {
//       auto feedback = std::make_shared<inpipe_robot_interfaces::action::MappingConfig::Feedback>();
//       auto result = std::make_shared<inpipe_robot_interfaces::action::MappingConfig::Result>();

//       std::vector<geometry_msgs::msg::Point> pts;
//       d_p_ = goal->get_goal()->diameter;
//       addCircle(d_p_);

//       for (size_t i = 0; i < pts.size(); i++) {
//         if (goal->is_canceling()) {
//           result->success = false;
//           result->message = "Mapping canceled by client.";
//           goal->canceled(result);

//           return;
//         }
//         if (i % 10 == 0) {
//           feedback->points_mapped = all_points_.size();
//           goal->publish_feedback(feedback);
//         }
//       }

//       result->success = true;
//       result->message = "Mapping finished.";
//       goal->succeed(result);

//       // publishCloud();
//     }
//   }).detach();
// }

void IpirMapperNode::handleMapperAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::MappingConfig>> goal)
{
  std::thread([this, goal]() {
    if (!(got_d_p_ && got_odom_)) {
      auto result = std::make_shared<inpipe_robot_interfaces::action::MappingConfig::Result>();
      result->success = false;
      result->message = "Missing diameter or Odometry data.";
      goal->abort(result);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      current_mapping_handle_ = goal;
      mapping_active_ = true;
      d_p_ = goal->get_goal()->diameter / 1000.0;
    }
    auto feedback = std::make_shared<inpipe_robot_interfaces::action::MappingConfig::Feedback>();
    auto result = std::make_shared<inpipe_robot_interfaces::action::MappingConfig::Result>();

    size_t last_feedback_points = 0;

    while(rclcpp::ok()) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (!mapping_active_) {
        result->success = false;
        result->message = "Mapping stopped.";
        goal->canceled(result);
        return;
      }

      if (all_points_.size() > last_feedback_points) {
        feedback->points_mapped = all_points_.size();
        goal->publish_feedback(feedback);
        last_feedback_points = all_points_.size();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Feedback rate
    }

    result->success = true;
    result->message = "Mapping finished.";
    goal->succeed(result);
  }).detach();
}

void IpirMapperNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  z_ = msg->pose.pose.position.z;

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  got_odom_ = true;

  // if (got_d_p_)
  //   addCircle(d_p_);

  if (mapping_active_)
    addCircle(d_p_);
}

void IpirMapperNode::addCircle(double diameter)
{
  if (!got_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry yet, circle ignored.");
    return;
  }
  std::vector<geometry_msgs::msg::Point> circle_pts;
  mapper(diameter, circle_pts);
  all_points_.insert(all_points_.end(), circle_pts.begin(), circle_pts.end());
  publishCloud();
  RCLCPP_INFO(this->get_logger(),
    "Mapped circle at (%.3f, %.3f, %.3f), rpy(%.3f, %.3f, %.3f), diameter=%.3f",
    x_, y_, z_, roll_, pitch_, yaw_, d_p_);
}

void IpirMapperNode::mapper(double diameter, std::vector<geometry_msgs::msg::Point>& circle_pts)
{
  double radius = diameter / 2.0;
  int N = 100;

  tf2::Quaternion q;
  q.setRPY(roll_, pitch_, yaw_);
  tf2::Matrix3x3 R(q);

  tf2::Vector3 normal = R * tf2::Vector3(1, 0, 0);

  tf2::Vector3 ref(0, 0, 1);
  if (fabs(normal.dot(ref)) > 0.99)
    ref = tf2::Vector3(0, 1, 0);
  tf2::Vector3 v1 = normal.cross(ref).normalized(); // normal ⨯ ref
  tf2::Vector3 v2 = normal.cross(v1).normalized();  // normal ⨯ v1

  for (int i = 0; i < N; ++i) {
    double angle = 2 * M_PI * i / N;
    tf2::Vector3 pt = tf2::Vector3(x_, y_, z_) + radius * (cos(angle) * v1 + sin(angle) * v2);

    geometry_msgs::msg::Point msg_pt;
    msg_pt.x = pt.x();
    msg_pt.y = pt.y();
    msg_pt.z = pt.z();
    circle_pts.push_back(msg_pt);
  }
}

void IpirMapperNode::publishCloud()
{
  if (all_points_.empty()) return;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = this->now();
  cloud.header.frame_id = "odom";

  cloud.height = 1;
  cloud.width = all_points_.size();
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.point_step = 12; // 3*float32
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.fields.resize(3);

  cloud.fields[0].name = "x"; cloud.fields[0].offset = 0; cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[0].count = 1;
  cloud.fields[1].name = "y"; cloud.fields[1].offset = 4; cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[1].count = 1;
  cloud.fields[2].name = "z"; cloud.fields[2].offset = 8; cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[2].count = 1;

  cloud.data.resize(cloud.row_step);
  for (size_t i = 0; i < all_points_.size(); ++i) {
    float* data_ptr = reinterpret_cast<float*>(&cloud.data[i * 12]);
    data_ptr[0] = all_points_[i].x;
    data_ptr[1] = all_points_[i].y;
    data_ptr[2] = all_points_[i].z;
  }
  pc_pub_->publish(cloud);
}

void IpirMapperNode::saveMap(const std::string& filename)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = all_points_.size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(all_points_.size());

  for (size_t i = 0; i < all_points_.size(); i++) {
    cloud.points[i].x = all_points_[i].x;
    cloud.points[i].y = all_points_[i].y;
    cloud.points[i].z = all_points_[i].z;
  }

  pcl::io::savePCDFileBinary(filename, cloud);
  RCLCPP_INFO(this->get_logger(), "Map saved to %s", filename.c_str());
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IpirMapperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}