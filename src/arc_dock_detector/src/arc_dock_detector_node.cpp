#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
struct RearPoint
{
  double rel_angle;
  double x;
  double y;
};
}  // namespace

class ArcDockDetectorNode : public rclcpp::Node
{
public:
  ArcDockDetectorNode()
  : Node("arc_dock_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/detected_dock_pose");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");
    min_range_ = this->declare_parameter<double>("min_range", 0.22);
    max_range_ = this->declare_parameter<double>("max_range", 1.40);
    rear_half_angle_ = this->declare_parameter<double>("rear_half_angle_rad", 1.05);
    cluster_gap_ = this->declare_parameter<double>("cluster_gap_m", 0.06);
    min_points_ = this->declare_parameter<int>("min_cluster_points", 8);
    target_chord_ = this->declare_parameter<double>("target_arc_chord_m", 0.32);
    min_chord_ = this->declare_parameter<double>("min_chord_m", 0.22);
    max_chord_ = this->declare_parameter<double>("max_chord_m", 0.42);
    dock_depth_offset_ = this->declare_parameter<double>("dock_depth_offset_m", 0.18);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ArcDockDetectorNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "C++ ArcDockDetector started: %s -> %s (%s), target_chord=%.2fm",
      scan_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(), target_chord_);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double eff_min = std::max(min_range_, static_cast<double>(msg->range_min));
    const double eff_max = std::min(max_range_, static_cast<double>(msg->range_max));

    std::vector<RearPoint> pts;
    pts.reserve(128);
    const size_t n = msg->ranges.size();
    for (size_t i = 0; i < n; ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < eff_min || r > eff_max) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const double rel_rear = std::atan2(std::sin(angle - M_PI), std::cos(angle - M_PI));
      if (std::abs(rel_rear) > rear_half_angle_) {
        continue;
      }
      pts.push_back({rel_rear, r * std::cos(angle), r * std::sin(angle)});
    }

    if (static_cast<int>(pts.size()) < min_points_) {
      return;
    }

    std::sort(pts.begin(), pts.end(), [](const RearPoint & a, const RearPoint & b) {
      return a.rel_angle < b.rel_angle;
    });

    using Cluster = std::vector<std::pair<double, double>>;
    std::vector<Cluster> clusters;
    Cluster current;
    current.emplace_back(pts[0].x, pts[0].y);

    for (size_t i = 1; i < pts.size(); ++i) {
      const double dx = pts[i].x - current.back().first;
      const double dy = pts[i].y - current.back().second;
      if (std::hypot(dx, dy) <= cluster_gap_) {
        current.emplace_back(pts[i].x, pts[i].y);
      } else {
        if (static_cast<int>(current.size()) >= min_points_) {
          clusters.push_back(std::move(current));
        }
        current.clear();
        current.emplace_back(pts[i].x, pts[i].y);
      }
    }
    if (static_cast<int>(current.size()) >= min_points_) {
      clusters.push_back(std::move(current));
    }

    if (clusters.empty()) {
      return;
    }

    const Cluster * best_cluster = nullptr;
    double best_err = std::numeric_limits<double>::infinity();
    for (const auto & c : clusters) {
      const double chord = std::hypot(c.back().first - c.front().first, c.back().second - c.front().second);
      if (chord >= min_chord_ && chord <= max_chord_) {
        const double err = std::abs(chord - target_chord_);
        if (err < best_err) {
          best_err = err;
          best_cluster = &c;
        }
      }
    }

    if (!best_cluster) {
      return;
    }

    double cx = 0.0, cy = 0.0;
    for (const auto & p : *best_cluster) {
      cx += p.first;
      cy += p.second;
    }
    cx /= static_cast<double>(best_cluster->size());
    cy /= static_cast<double>(best_cluster->size());

    const double dx = best_cluster->back().first - best_cluster->front().first;
    const double dy = best_cluster->back().second - best_cluster->front().second;
    const double chord_len = std::hypot(dx, dy);
    if (chord_len < 1e-4) {
      return;
    }

    double nx = -dy / chord_len;
    double ny = dx / chord_len;
    if ((nx * (-cx) + ny * (-cy)) < 0.0) {
      nx = -nx;
      ny = -ny;
    }

    const double target_x = cx + nx * dock_depth_offset_;
    const double target_y = cy + ny * dock_depth_offset_;
    const double yaw_laser = std::atan2(ny, nx);

    geometry_msgs::msg::PoseStamped pose_laser;
    pose_laser.header = msg->header;
    pose_laser.pose.position.x = target_x;
    pose_laser.pose.position.y = target_y;
    pose_laser.pose.position.z = 0.0;
    pose_laser.pose.orientation.z = std::sin(yaw_laser * 0.5);
    pose_laser.pose.orientation.w = std::cos(yaw_laser * 0.5);

    if (msg->header.frame_id == target_frame_) {
      pose_pub_->publish(pose_laser);
      return;
    }

    try {
      const auto tf = tf_buffer_.lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.05));
      const double tx = tf.transform.translation.x;
      const double ty = tf.transform.translation.y;
      const auto & q = tf.transform.rotation;
      const double tf_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

      const double cos_t = std::cos(tf_yaw);
      const double sin_t = std::sin(tf_yaw);

      geometry_msgs::msg::PoseStamped pose_odom;
      pose_odom.header.stamp = msg->header.stamp;
      pose_odom.header.frame_id = target_frame_;
      pose_odom.pose.position.x = tx + cos_t * target_x - sin_t * target_y;
      pose_odom.pose.position.y = ty + sin_t * target_x + cos_t * target_y;
      pose_odom.pose.position.z = 0.0;
      const double out_yaw = tf_yaw + yaw_laser;
      pose_odom.pose.orientation.z = std::sin(out_yaw * 0.5);
      pose_odom.pose.orientation.w = std::cos(out_yaw * 0.5);

      pose_pub_->publish(pose_odom);
    } catch (const tf2::TransformException &) {
    }
  }

  std::string scan_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double min_range_;
  double max_range_;
  double rear_half_angle_;
  double cluster_gap_;
  int min_points_;
  double target_chord_;
  double min_chord_;
  double max_chord_;
  double dock_depth_offset_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArcDockDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
