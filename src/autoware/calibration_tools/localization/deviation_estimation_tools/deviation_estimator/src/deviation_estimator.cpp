// Copyright 2018-2019 Autoware Foundation
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

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "deviation_estimator/deviation_estimator.hpp"
#include "rclcpp/logging.hpp"

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on
using std::placeholders::_1;

double double_round(const double x, const int n)
{
  return std::round(x * pow(10, n)) / pow(10, n);
}

template<typename T>
double calculateMean(const std::vector<T> & v)
{
  if (v.size() == 0) {return 0;}

  double mean = 0;
  for (const T & t : v) {
    mean += t;
  }
  mean /= v.size();
  return mean;
}

template<typename T>
double calculateStd(const std::vector<T> & v)
{
  if (v.size() == 0) {return 0;}

  double mean = calculateMean(v);
  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

template<typename T>
double calculateStdMeanConst(const std::vector<T> & v, const double mean)
{
  if (v.size() == 0) {return 0;}

  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size() );
}

struct CompareMsgTimestamp
{
  template<typename T1>
  bool operator()(T1 const & t1, double const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < t2;
  }

  template<typename T2>
  bool operator()(double const & t1, T2 const & t2) const
  {
    return t1 < rclcpp::Time(t2.header.stamp).seconds();
  }

  template<typename T1, typename T2>
  bool operator()(T1 const & t1, T2 const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < rclcpp::Time(t2.header.stamp).seconds();
  }

  template<typename T1>
  bool operator()(T1 const & t1, rclcpp::Time const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < t2.seconds();
  }

  template<typename T2>
  bool operator()(rclcpp::Time const & t1, T2 const & t2) const
  {
    return t1.seconds() < rclcpp::Time(t2.header.stamp).seconds();
  }
};

template<typename T>
std::vector<T> extractSubTrajectory(
  const std::vector<T> & msg_list,
  const rclcpp::Time t0, const rclcpp::Time t1
) {
  auto start_iter = std::lower_bound(
    msg_list.begin(), msg_list.end(),
    t0, CompareMsgTimestamp());
  auto end_iter = std::lower_bound(
    msg_list.begin(), msg_list.end(),
    t1, CompareMsgTimestamp());
  std::vector<T> msg_list_sub(start_iter, end_iter);
  return msg_list_sub;
}

DeviationEstimator::DeviationEstimator(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  show_debug_info_ = declare_parameter("show_debug_info", false);
  dt_design_ = declare_parameter("dt_design", 10.0);
  dx_design_ = declare_parameter("dx_design", 30.0);
  vx_threshold_ = declare_parameter("vx_threshold", 1.5);
  wz_threshold_ = declare_parameter("wz_threshold", 0.01);
  estimation_freq_ = declare_parameter("estimation_freq", 0.5);
  use_pose_with_covariance_ =
    declare_parameter("use_pose_with_covariance", true);
  use_twist_with_covariance_ =
    declare_parameter("use_twist_with_covariance", true);
  use_predefined_coef_vx_ =
    declare_parameter("use_predefined_coef_vx", false);
  predefined_coef_vx_ = declare_parameter("predefined_coef_vx", 1.0);
  results_path_ = declare_parameter("results_path", "test");

  auto timer_control_callback = std::bind(&DeviationEstimator::timerCallback, this);
  auto period_control = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / estimation_freq_));
  timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_control_callback)>>(
    this->get_clock(), period_control, std::move(timer_control_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_control_, nullptr);

  sub_pose_with_cov_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1,
    std::bind(&DeviationEstimator::callbackPoseWithCovariance, this, _1));
  sub_twist_with_cov_ =
    create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance",
    1,
    std::bind(&DeviationEstimator::callbackTwistWithCovariance, this, _1));
  sub_pose_ =
    create_subscription<geometry_msgs::msg::PoseStamped>(
    "in_pose", 1,
    std::bind(&DeviationEstimator::callbackPose, this, _1));
  sub_twist_ =
    create_subscription<geometry_msgs::msg::TwistStamped>(
    "in_twist",
    1,
    std::bind(&DeviationEstimator::callbackTwist, this, _1));

  pub_coef_vx_ =
    create_publisher<std_msgs::msg::Float64>("estimated_coef_vx", 1);
  pub_bias_wz_ =
    create_publisher<std_msgs::msg::Float64>("estimated_bias_wz", 1);
  pub_stddev_vx_ =
    create_publisher<std_msgs::msg::Float64>("estimated_stddev_vx", 1);
  pub_stddev_wz_ =
    create_publisher<std_msgs::msg::Float64>("estimated_stddev_wz", 1);

  DEBUG_INFO(
    this->get_logger(),
    "[Deviation Estimator] launch success");
}

void DeviationEstimator::callbackPoseWithCovariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // push pose_msg to queue
  if (use_pose_with_covariance_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    pose_buf_.push_back(pose);
    pose_all_.push_back(pose);
  }
}

void DeviationEstimator::callbackTwistWithCovariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // push twist_msg to queue
  if (use_twist_with_covariance_ == true) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header = msg->header;
    twist.twist = msg->twist.twist;

    if (use_predefined_coef_vx_) {
      twist.twist.linear.x *= predefined_coef_vx_;
    }

    twist_all_.push_back(twist);
  }
}

void DeviationEstimator::callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // push pose_msg to queue
  if (!use_pose_with_covariance_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose;

    pose_buf_.push_back(pose);
    pose_all_.push_back(pose);
  }
}

void DeviationEstimator::callbackTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // push twist_msg to queue
  if (!use_twist_with_covariance_) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header = msg->header;
    twist.twist = msg->twist;

    if (use_predefined_coef_vx_) {
      twist.twist.linear.x *= predefined_coef_vx_;
    }
    twist_all_.push_back(twist);
  }
}

void DeviationEstimator::timerCallback()
{
  // Update bias
  if (pose_buf_.size() != 0) {
    updateBias();
  }

  // Publish vx bias
  if (coef_vx_.second != 0) {
    std_msgs::msg::Float64 coef_vx_msg;
    coef_vx_msg.data = coef_vx_.first / coef_vx_.second;
    pub_coef_vx_->publish(coef_vx_msg);
  }

  // Publish wz bias
  if (bias_wz_.second != 0) {
    std_msgs::msg::Float64 bias_wz_msg;
    bias_wz_msg.data = bias_wz_.first / bias_wz_.second;
    pub_bias_wz_->publish(bias_wz_msg);
  }

  // Estimate stddev when ready
  if ((coef_vx_.second != 0) & (bias_wz_.second != 0)) {
    estimateStddev();
    estimateStddevPrime();
  }
}

void DeviationEstimator::updateBias()
{
  if(twist_all_.empty()) return;
  rclcpp::Time t0_rclcpp_time = rclcpp::Time(pose_buf_.front().header.stamp);
  rclcpp::Time t1_rclcpp_time = rclcpp::Time(pose_buf_.back().header.stamp);
  if (t1_rclcpp_time <= t0_rclcpp_time) return;

  std::vector<geometry_msgs::msg::TwistStamped> twist_buf =
    extractSubTrajectory(twist_all_, t0_rclcpp_time, t1_rclcpp_time);

  double t0 = t0_rclcpp_time.seconds();
  double t1 = t1_rclcpp_time.seconds();

  // Calculate coef_vx only when the velocity is higher than the threshold
  double mean_abs_vx = 0;
  for (const auto & msg : twist_buf) {
    mean_abs_vx += abs(msg.twist.linear.x);
  }
  mean_abs_vx /= twist_buf.size();
  if (mean_abs_vx > vx_threshold_) {
    std::pair<double, double> d_pos = calculateErrorPos(pose_buf_, twist_buf, false);
    double dx = pose_buf_.back().pose.position.x - pose_buf_.front().pose.position.x;
    double dy = pose_buf_.back().pose.position.y - pose_buf_.front().pose.position.y;
    double d_coef_vx = (d_pos.first * dx + d_pos.second * dy) /
      (d_pos.first * d_pos.first + d_pos.second * d_pos.second);

    double time_factor = (
      rclcpp::Time(twist_buf.back().header.stamp).seconds() -
      rclcpp::Time(twist_buf.front().header.stamp).seconds()
      ) / (t1 - t0);
    coef_vx_.first += d_coef_vx * time_factor;
    coef_vx_.second += 1;
    coef_vx_list_.push_back(d_coef_vx);
  } else {
    DEBUG_INFO(
      this->get_logger(), "[Deviation Estimator] vx excitation not enough.");
  }

  double error_yaw = calculateErrorYaw(pose_buf_, twist_buf, false);
  bias_wz_.first += (t1 - t0) * error_yaw;
  bias_wz_.second += (t1 - t0) * (t1 - t0);
  bias_wz_list_.push_back(error_yaw / (t1 - t0));

  pose_buf_.clear();
}

void DeviationEstimator::estimateStddev()
{
  double est_window_duration = 4.0; // Hard coded

  auto duration = rclcpp::Duration(
    int(est_window_duration / 1),
    int((est_window_duration - est_window_duration / 1) * 1e9)
  );

  std::vector<double> error_x_list;
  std::vector<double> error_yaw_lists;

  rclcpp::Time t0_rclcpp_time, t1_rclcpp_time;
  t0_rclcpp_time = rclcpp::Time(pose_all_.front().header.stamp);
  t1_rclcpp_time = t0_rclcpp_time + duration;

  // Iterate over the whole sub_trajectory every time. Calculation cost ~ O(T^2)
  while (t1_rclcpp_time < rclcpp::Time(pose_all_.back().header.stamp)) {

    std::vector<geometry_msgs::msg::PoseStamped> pose_sub_traj =
      extractSubTrajectory(pose_all_, t0_rclcpp_time, t1_rclcpp_time);

    if (rclcpp::Time(pose_sub_traj.back().header.stamp) > rclcpp::Time(pose_sub_traj.front().header.stamp)) {
      std::vector<geometry_msgs::msg::TwistStamped> twist_sub_traj = extractSubTrajectory(
        twist_all_,
        rclcpp::Time(pose_sub_traj.front().header.stamp),
        rclcpp::Time(pose_sub_traj.back().header.stamp)
      );

      // calculate Error theta only if the vehicle is moving
      double x0 = pose_sub_traj.front().pose.position.x;
      double y0 = pose_sub_traj.front().pose.position.y;
      double x1 = pose_sub_traj.back().pose.position.x;
      double y1 = pose_sub_traj.back().pose.position.y;
      double distance = std::sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
      if (distance > est_window_duration * vx_threshold_) {
        double error_yaw = calculateErrorYaw(pose_sub_traj, twist_sub_traj);
        error_yaw_lists.push_back(error_yaw);
      }

      // calculate Error x only if the vehicle is not curving
      double mean_abs_wz = 0;
      for (auto msg : twist_sub_traj) {
        mean_abs_wz += abs(msg.twist.angular.z);
      }
      mean_abs_wz /= twist_sub_traj.size();
      if (mean_abs_wz < wz_threshold_) {
        std::pair<double, double> d_pos = calculateErrorPos(pose_sub_traj, twist_sub_traj);
        double distance_from_twist = std::sqrt(d_pos.first * d_pos.first + d_pos.second * d_pos.second);
        error_x_list.push_back(distance - distance_from_twist);
      }
    }

    t0_rclcpp_time += duration;
    t1_rclcpp_time += duration;
  }

  stddev_vx_ = calculateStd(error_x_list) / std::sqrt(est_window_duration);
  stddev_wz_ = calculateStd(error_yaw_lists) / std::sqrt(est_window_duration);
}

std::pair<double, double> DeviationEstimator::calculateErrorPos(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list,
  const bool enable_bias)
{
  double t_prev = rclcpp::Time(twist_list.front().header.stamp).seconds();
  std::pair<double, double> d_pos;
  double yaw = getYawFromQuat(pose_list.front().pose.orientation);
  for (std::size_t i = 0; i < twist_list.size() - 1; ++i) {
    double t_cur = rclcpp::Time(twist_list[i + 1].header.stamp).seconds();
    yaw += twist_list[i].twist.angular.z * (t_cur - t_prev);
    if (enable_bias) {
      d_pos.first += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::cos(yaw) *
        coef_vx_.first / coef_vx_.second;
      d_pos.second += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::sin(yaw) *
        coef_vx_.first / coef_vx_.second;
    } else {
      d_pos.first += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::cos(yaw);
      d_pos.second += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::sin(yaw);
    }
    t_prev = t_cur;
  }
  return d_pos;
}

double DeviationEstimator::calculateErrorYaw(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list,
  const bool enable_bias)
{
  double yaw_0 = getYawFromQuat(pose_list.front().pose.orientation);
  double yaw_1 = getYawFromQuat(pose_list.back().pose.orientation);

  double d_yaw = 0;
  double t_prev = rclcpp::Time(twist_list.front().header.stamp).seconds();
  for (std::size_t i = 0; i < twist_list.size() - 1; ++i) {
    double t_cur = rclcpp::Time(twist_list[i + 1].header.stamp).seconds();
    if (enable_bias) {
      d_yaw += (t_cur - t_prev) * (twist_list[i].twist.angular.z -
        bias_wz_.first / bias_wz_.second);
    } else {
      d_yaw += (t_cur - t_prev) * twist_list[i].twist.angular.z;
    }
    t_prev = t_cur;
  }
  double error_yaw = clipRadian(- yaw_1 + yaw_0 + d_yaw);
  return error_yaw;
}

void DeviationEstimator::estimateStddevPrime()
{
  double stddev_coef_vx = calculateStd(coef_vx_list_);
  stddev_vx_prime_ = std::sqrt(
    pow(stddev_vx_, 2) +
    pow(stddev_coef_vx, 2) * pow(dx_design_, 2) / dt_design_);

  double stddev_bias_wz = calculateStdMeanConst(bias_wz_list_, bias_wz_.first / bias_wz_.second);
  stddev_wz_prime_ = std::sqrt(pow(stddev_wz_, 2) + dt_design_ * pow(stddev_bias_wz, 2));

  std_msgs::msg::Float64 stddev_vx_msg;
  stddev_vx_msg.data = stddev_vx_prime_;
  pub_stddev_vx_->publish(stddev_vx_msg);

  std_msgs::msg::Float64 stddev_wz_msg;
  stddev_wz_msg.data = stddev_wz_prime_;
  pub_stddev_wz_->publish(stddev_wz_msg);

  if (results_path_.size() > 0) {
    saveEstimatedParameters(
      stddev_vx_prime_,
      stddev_wz_prime_,
      coef_vx_.first / coef_vx_.second,
      bias_wz_.first / bias_wz_.second
    );
  }
}

double DeviationEstimator::getYawFromQuat(const geometry_msgs::msg::Quaternion quat_msg)
{
  double r, p, y;
  tf2::Quaternion quat_t(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  tf2::Matrix3x3(quat_t).getRPY(r, p, y);
  return y;
}

double DeviationEstimator::clipRadian(const double rad)
{
  if (rad < -M_PI) {
    return rad + 2 * M_PI;
  } else if (rad >= M_PI) {
    return rad - 2 * M_PI;
  } else {
    return rad;
  }
}

void DeviationEstimator::saveEstimatedParameters(
  const double stddev_vx,
  const double stddev_wz,
  const double coef_vx,
  const double bias_wz)
{
  std::ofstream file(results_path_);
  file << "stddev_vx: " << double_round(stddev_vx, 5) << std::endl;
  file << "stddev_wz: " << double_round(stddev_wz, 5) << std::endl;
  file << "coef_vx: " << double_round(coef_vx, 5) << std::endl;
  file << "bias_wz: " << double_round(bias_wz, 6) << std::endl;
  file.close();
}
