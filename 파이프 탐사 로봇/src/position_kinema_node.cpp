/**
 * Geonwoo Kho, Chanwoo Seon
 */

#include "inpipe_robot/position_kinema_node.hpp"

#define WIDTH 1200
#define HEIGHT 800
#define SCALE 1.0
#define OFFSET_X 600
#define OFFSET_Y 200


PositionKinema::PositionKinema() : Node("position_kinema")
{
  RCLCPP_INFO(this->get_logger(), "\033[0;32m====== IPIR IK SOLVER ======");
  RCLCPP_INFO(this->get_logger(), "\033[0;32m=== Initializing... ===");

  D_p_ = 135.0;
  R_w_ = 58.0;
  W_R_ = 100.0;
  H_w_ = std::sqrt(D_p_ * D_p_ - W_R_ * W_R_);
  H_shaft_ = H_w_ - R_w_;
  R_H_o_ = D_p_ - 0.5 * (D_p_ - H_w_);
  R_H_i_ = R_H_o_ - H_w_;
  R_p_ = 205.0;
  x_vel_ = 0.229 * 30;
  L_.push_back(43.0);
  L_.push_back(136.0);
  L_.push_back(136.0);
  L_.push_back(136.0);
  L_.push_back(136.0);
  L_.push_back(93.0);

  RCLCPP_INFO(this->get_logger(), "\033[0;32mD_p_   :\t%f", D_p_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mR_w_   :\t%f", R_w_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mW_R_   :\t%f", W_R_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mH_w_   :\t%f", H_w_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mH_shaft_:\t%f", H_shaft_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mR_H_o_ :\t%f", R_H_o_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mR_H_i_ :\t%f", R_H_i_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mR_p_ :\t%f", R_p_);
  RCLCPP_INFO(this->get_logger(), "\033[0;32mx_vel_ :\t%f [m/s]", x_vel_);
  for (int i = 0; i < (int)L_.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "\033[0;32mL_%d : %f\t", i, L_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "\033[0;32m=== Done ===");

  // for (int i=0; i<6; i++) {
  //   newLoopControl();
  // }

  animateMotion();

}

// ===================================================================
std::pair<double, double> PositionKinema::new_o_func(double x) {
  double r = - R_p_ / 1.414;

  if (x < r) {
    double y = -x;
    return {x, y};
  } else if (x < -r) {
    double y = (1.414 * R_p_) - std::sqrt((R_p_ * R_p_) - (x * x));
    return {x, y};
  } else {
    double y = x;
    return {x, y};
  }
}

std::pair<double, double> PositionKinema::new_i_func(double x) {
  double dhr = H_shaft_ - R_p_;
  double r = dhr / 1.414;

  if (x < r) {
    double y = (1.414 * H_shaft_) - x;
    return {x, y};
  } else if (x < -r) {
    double y = (R_p_ * 1.414) - std::sqrt((dhr * dhr) - (x * x));
    return {x, y};
  } else {
    double y = (1.414 * H_shaft_) + x;
    return {x, y};
  }
}

std::pair<double, double> PositionKinema::new_c_func(double x) {
  double dhr = ((H_shaft_ - (2.0 * R_p_)) / 2.0);
  double r = dhr / 1.414;

  if (x < r) {
    double y = (H_shaft_ / 1.414) - x;
    return {x, y};
  } else if (x < -r) {
    double y = (1.414 * R_p_) - std::sqrt((dhr * dhr) - (x * x));
    return {x, y};
  } else {
    double y = (H_shaft_ / 1.414) + x;
    return {x, y};
  }
}

std::pair<double, double> PositionKinema::new_r_func(double x) {
  double dhr = (D_p_ + H_shaft_ - R_w_);
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

std::pair<double, double> PositionKinema::new_find_next_point(
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
    // RCLCPP_INFO(this->get_logger(), "\033[0;32mtrying dx=%.2f, dist=%.2f, d=%.2f", dx, dist, d);
    if (std::abs(d - dist) < 1) {
      return {x_candidate, y_candidate};
    }
  }
  return {NAN, NAN};
}
// ===================================================================
double PositionKinema::calc_theta(
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

  return std::acos(dot / (mag_u * mag_v));
}

double PositionKinema::calc_signed_theta(
  std::pair<double, double> p0,
  std::pair<double, double> p1,
  std::pair<double, double> p2
) {
  double ux = p1.first - p0.first;
  double uy = p1.second - p0.second;
  double vx = p2.first - p1.first;
  double vy = p2.second - p1.second;

  double dot = ux * vx + uy * vy;              // 내적 (cosθ 관련)
  double cross = ux * vy - uy * vx;            // 외적 (sinθ 관련)
  double mag_u = std::hypot(ux, uy);
  double mag_v = std::hypot(vx, vy);

  if (mag_u == 0 || mag_v == 0) return 0.0;     // 크기 0인 벡터 예외 처리

  double angle = std::atan2(cross, dot);        // atan2(sinθ, cosθ)

  return angle;  // 라디안 반환, 부호 있음
}

void PositionKinema::newLoopControl()
{
  RCLCPP_INFO(this->get_logger(), "\033[0;31m=== MAIN CALC ===");

  std::vector<std::pair<double, double>> points(7);
  std::vector<std::pair<double, double>> temp_points;
  std::vector<double> temp_theta;
  double x0;

  std::cout << "x0 입력: ";
  std::cin >> x0;
  x0 /= 1.414;
  points[0] = new_c_func(x0);
  points[1] = new_find_next_point(points[0], [this](double x) { return new_c_func(x); }, L_[0]);
  points[2] = new_find_next_point(points[1], [this](double x) { return new_o_func(x); }, L_[1]);
  points[3] = new_find_next_point(points[2], [this](double x) { return new_i_func(x); }, L_[2]);
  points[4] = new_find_next_point(points[3], [this](double x) { return new_o_func(x); }, L_[3]);
  points[5] = new_find_next_point(points[4], [this](double x) { return new_r_func(x); }, L_[4]);
  points[6] = new_find_next_point(points[5], [this](double x) { return new_r_func(x); }, L_[5]);

  // prev_points_.clear();
  for (int i = 0; i < (int)points.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "[p_%d]: \033[0;32m(%f, %f)", i, points[i].first, points[i].second);
    RCLCPP_INFO(this->get_logger(),
      "[different_%d]: \033[0;34m(%f, %f)", i,
      points[i].first - prev_points_[i].first,
      points[i].second - prev_points_[i].second);

    temp_points.push_back(points[i]);
    prev_points_[i] = points[i];
  }
  RCLCPP_INFO(this->get_logger(), "\033[0;32m----------------------------------------");
  // prev_theta_.clear();
  for (int i = 0; i < 5; ++i) {
    // double theta = calc_theta(points[i], points[i+1], points[i+2]);
    double theta = calc_signed_theta(points[i], points[i+1], points[i+2]);
    RCLCPP_INFO(this->get_logger(), "[theta_%d]: \033[0;32m%f [deg]", i, deg(theta));
    RCLCPP_INFO(this->get_logger(), "[different_%d]: \033[0;34m%f [deg]", i, deg(prev_theta_[i]) - deg(theta));

    temp_theta.push_back(theta);
    prev_theta_[i] = theta;
  }
  drawPose(temp_points, temp_theta);
}
// ===================================================================
void PositionKinema::drawPose(const std::vector<std::pair<double, double>>& points, const std::vector<double>& thetas) {
  cv::Mat canvas(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

  auto scale = [](double val) { return int(val * SCALE); };

  auto drawDashedLine = [&](std::function<std::pair<double, double>(double)> func, double x_start, double x_end, cv::Scalar color) {
    const double step = 1.0;
    const int dash_length = 5;
    const int gap_length = 5;
    int count = 0;
    cv::Point last_pt;
    bool has_last = false;

    for (double x = x_start; x <= x_end; x += step) {
      auto pt_pair = func(x);
      if (std::isnan(pt_pair.second)) continue;

      cv::Point pt(scale(pt_pair.first) + OFFSET_X, HEIGHT - (scale(pt_pair.second) + OFFSET_Y));
      if (has_last && (count / dash_length) % 2 == 0) {
        cv::line(canvas, last_pt, pt, color, 1);
      }
      last_pt = pt;
      has_last = true;
      count++;
    }
  };

  // Draw background curves (as dashed lines)
  drawDashedLine([this](double x) { return new_c_func(x); }, -150, 300, cv::Scalar(200, 200, 200)); // gray
  drawDashedLine([this](double x) { return new_o_func(x); }, -150, 300, cv::Scalar(200, 0, 200));   // purple
  drawDashedLine([this](double x) { return new_i_func(x); }, -150, 300, cv::Scalar(0, 200, 200));   // cyan
  drawDashedLine([this](double x) { return new_r_func(x); }, -150, 300, cv::Scalar(100, 100, 0));   // brown-ish


  RCLCPP_INFO(this->get_logger(), "points: %ld", points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    auto p = points[i];
    cv::Point pt(scale(p.first) + OFFSET_X, HEIGHT - (scale(p.second) + OFFSET_Y));

    cv::circle(canvas, pt, 5, cv::Scalar(0, 0, 255), -1);
    cv::putText(canvas, "p" + std::to_string(i), pt + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0));
    if (i > 0) {
      auto prev = points[i - 1];
      cv::Point pt_prev(scale(prev.first) + OFFSET_X, HEIGHT - (scale(prev.second) + OFFSET_Y));
      cv::line(canvas, pt_prev, pt, cv::Scalar(255, 0, 0), 2);

      double dist = std::hypot(p.first - prev.first, p.second - prev.second);
      cv::Point mid_pt = (pt + pt_prev) * 0.5;

      std::string dist_text = std::to_string(static_cast<int>(dist));
      cv::putText(canvas, dist_text + " mm", mid_pt + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 0, 128));
    }
  }

  for (size_t i = 0; i + 2 < points.size(); ++i) {
    auto p = points[i + 1];
    cv::Point pt(scale(p.first) + OFFSET_X, HEIGHT - (scale(p.second) + OFFSET_Y));
    std::string text = std::to_string(int(thetas[i])) + " deg";
    cv::putText(canvas, text, pt + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 128, 0));
  }

  cv::imshow("Kinematic Chain", canvas);
  cv::waitKey(0);
}

void PositionKinema::drawPoseOnCanvas(cv::Mat& canvas,
    const std::vector<std::pair<double, double>>& points,
    const std::vector<double>& thetas)
{
  auto scale = [](double val) { return int(val * SCALE); };

  auto drawDashedLine = [&](std::function<std::pair<double, double>(double)> func, double x_start, double x_end, cv::Scalar color) {
    const double step = 1.0;
    const int dash_length = 5;
    const int gap_length = 5;
    int count = 0;
    cv::Point last_pt;
    bool has_last = false;

    for (double x = x_start; x <= x_end; x += step) {
      auto pt_pair = func(x);
      if (std::isnan(pt_pair.second)) continue;

      cv::Point pt(scale(pt_pair.first) + OFFSET_X, HEIGHT - (scale(pt_pair.second) + OFFSET_Y));
      if (has_last && (count / dash_length) % 2 == 0) {
        cv::line(canvas, last_pt, pt, color, 1);
      }
      last_pt = pt;
      has_last = true;
      count++;
    }
  };

  // Draw background curves (as dashed lines)
  drawDashedLine([this](double x) { return new_c_func(x); }, -1000, 1000, cv::Scalar(200, 200, 200)); // gray
  drawDashedLine([this](double x) { return new_o_func(x); }, -1000, 1000, cv::Scalar(200, 0, 200));   // purple
  drawDashedLine([this](double x) { return new_i_func(x); }, -1000, 1000, cv::Scalar(0, 200, 200));   // cyan
  drawDashedLine([this](double x) { return new_r_func(x); }, -1000, 1000, cv::Scalar(100, 100, 0));   // brown-ish

  for (size_t i = 0; i < points.size(); ++i) {
    auto p = points[i];
    cv::Point pt(scale(p.first) + OFFSET_X, HEIGHT - (scale(p.second) + OFFSET_Y));

    cv::circle(canvas, pt, 5, cv::Scalar(0, 0, 255), -1);
    cv::putText(canvas, "p" + std::to_string(i), pt + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0));

    if (i > 0) {
      auto prev = points[i - 1];
      cv::Point pt_prev(scale(prev.first) + OFFSET_X, HEIGHT - (scale(prev.second) + OFFSET_Y));
      cv::line(canvas, pt_prev, pt, cv::Scalar(255, 0, 0), 2);

      // 벡터 길이 표시
      double length = std::hypot(p.first - prev.first, p.second - prev.second);
      cv::Point mid = (pt + pt_prev) * 0.5;
      cv::putText(canvas, std::to_string((int)length) + "mm", mid, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 0, 128));
    }
  }

  // 각도 표시
  for (size_t i = 0; i + 2 < points.size(); ++i) {
    auto p = points[i + 1];
    cv::Point pt(scale(p.first) + OFFSET_X, HEIGHT - (scale(p.second) + OFFSET_Y));
    std::string text = std::to_string(int(thetas[i])) + " deg";
    cv::putText(canvas, text, pt + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 128, 0));
  }
}

void PositionKinema::animateMotion()
{
  double x0 = -100.0;
  double vel = x_vel_ * 1000.0; // [mm/s]
  double dt = 0.001;
  double dx = vel * dt;

  auto start_time = std::chrono::high_resolution_clock::now();

  while (rclcpp::ok() && x0 <= 805) {
    std::vector<std::pair<double, double>> points(7);
    std::vector<double> thetas;

    points[0] = new_c_func(x0);
    points[1] = new_find_next_point(points[0], [this](double x) { return new_c_func(x); }, L_[0]);
    points[2] = new_find_next_point(points[1], [this](double x) { return new_o_func(x); }, L_[1]);
    points[3] = new_find_next_point(points[2], [this](double x) { return new_i_func(x); }, L_[2]);
    points[4] = new_find_next_point(points[3], [this](double x) { return new_o_func(x); }, L_[3]);
    points[5] = new_find_next_point(points[4], [this](double x) { return new_r_func(x); }, L_[4]);
    points[6] = new_find_next_point(points[5], [this](double x) { return new_r_func(x); }, L_[5]);

    if (std::isnan(points[1].first)) break;

    for (int i = 0; i < 5; ++i) {
      thetas.push_back(calc_theta(points[i], points[i + 1], points[i + 2]));
      RCLCPP_INFO(this->get_logger(), "[theta_%d]: \033[0;32m%f [deg]", i, deg(thetas[i]));
    }

    cv::Mat canvas(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

    drawPoseOnCanvas(canvas, points, thetas);

    std::string text = "x = " + std::to_string((int)x0) + " mm | v = " +
                       std::to_string(x_vel_) + " m/s | t = " + std::to_string((int)(x0 / dx * dt)) + " s";
    cv::putText(canvas, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(50, 50, 50), 2);

    cv::imshow("Kinematic Animation", canvas);
    int key = cv::waitKey(300);
    if (key == 27 || key == 'q') break;

    x0 += dx;
    // dt += dt;
  }

  auto end_time = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  RCLCPP_INFO(this->get_logger(), "\033[0;35m[Animation Duration] %ld ms (%.2f sec)", duration, duration / 1000.0);
}

// ===================================================================
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto kinema = std::make_shared<PositionKinema>();
  rclcpp::spin(kinema);
  rclcpp::shutdown();
  return 0;
}

// x_0가 640~680정도 되어야 통과