#include "elevation_costmap/elevation_costmap_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace elevation_costmap
{

namespace
{
struct CloudFieldOffsets
{
  int x{-1};
  int y{-1};
  int z{-1};
  int point_step{0};
};

bool findXyzOffsets(const sensor_msgs::msg::PointCloud2 & cloud, CloudFieldOffsets & out)
{
  out.point_step = static_cast<int>(cloud.point_step);
  for (const auto & f : cloud.fields) {
    if (f.name == "x") {
      out.x = static_cast<int>(f.offset);
    } else if (f.name == "y") {
      out.y = static_cast<int>(f.offset);
    } else if (f.name == "z") {
      out.z = static_cast<int>(f.offset);
    }
  }
  return out.x >= 0 && out.y >= 0 && out.z >= 0;
}

inline float readFloat(const uint8_t * ptr)
{
  float v;
  std::memcpy(&v, ptr, sizeof(float));
  return v;
}

tf2::Transform toTf2(const geometry_msgs::msg::Transform & t)
{
  tf2::Transform tf;
  tf2::fromMsg(t, tf);
  return tf;
}
}  // namespace

ElevationCostmapNode::ElevationCostmapNode()
: Node("elevation_costmap_node"),
  grid_(grid_config_)
{
  declareParameters();
  loadParameters();
  grid_.setConfig(grid_config_);

  const double angle_increment = angle_increment_deg_ * M_PI / 180.0;
  const int n_beams = std::max(
    1, static_cast<int>(std::lround((angle_max_ - angle_min_) / angle_increment)));
  scan_ranges_.assign(static_cast<std::size_t>(n_beams), 0.f);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::QoS(1));

  const auto sensor_qos = rclcpp::SensorDataQoS();
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, sensor_qos,
    std::bind(&ElevationCostmapNode::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "elevation_costmap ready: cloud='%s' -> scan='%s' frame='%s' "
    "grid=%.2fm @ %.2fm (%dx%d) beams=%d angle_inc=%.2fdeg",
    cloud_topic_.c_str(), scan_topic_.c_str(), target_frame_.c_str(),
    grid_config_.map_size, grid_config_.sub_resolution,
    grid_.subWidth(), grid_.subHeight(),
    n_beams, angle_increment_deg_);
}

void ElevationCostmapNode::declareParameters()
{
  declare_parameter<std::string>("target_frame", "base_link");
  declare_parameter<std::string>("cloud_topic", "/livox/lidar");
  declare_parameter<std::string>("scan_topic", "/scan");

  declare_parameter<double>("map_size", 4.0);
  declare_parameter<double>("sub_resolution", 0.05);
  declare_parameter<double>("plan_resolution", 0.15);
  declare_parameter<double>("delta_h_min", 0.03);
  declare_parameter<double>("delta_h_max", 0.15);
  declare_parameter<double>("cost_gain", 99.0);
  declare_parameter<int>("lethal_cost", 100);
  declare_parameter<double>("temporal_decay", 0.85);
  declare_parameter<double>("unobserved_margin_cost", 50.0);
  declare_parameter<bool>("enable_slope_correction", true);
  declare_parameter<double>("slope_allow_deg", 12.0);
  declare_parameter<int>("min_points_per_cell", 3);

  declare_parameter<double>("scan.angle_min", -M_PI);
  declare_parameter<double>("scan.angle_max", M_PI);
  declare_parameter<double>("scan.angle_increment_deg", 1.0);
  declare_parameter<double>("scan.range_min", 0.1);
  declare_parameter<double>("scan.range_max", 2.83);

  declare_parameter<double>("roi.x_min", -2.0);
  declare_parameter<double>("roi.x_max", 2.0);
  declare_parameter<double>("roi.y_min", -2.0);
  declare_parameter<double>("roi.y_max", 2.0);
  declare_parameter<double>("roi.z_ground_min", -0.5);
  declare_parameter<double>("roi.z_robot_height", 1.5);

  declare_parameter<bool>("self_filter.enable", true);
  declare_parameter<double>("self_filter.x_min", -0.35);
  declare_parameter<double>("self_filter.x_max", 0.35);
  declare_parameter<double>("self_filter.y_min", -0.25);
  declare_parameter<double>("self_filter.y_max", 0.25);
  declare_parameter<double>("self_filter.z_min", -0.1);
  declare_parameter<double>("self_filter.z_max", 0.6);

  declare_parameter<double>("tf_timeout_sec", 0.05);
}

void ElevationCostmapNode::loadParameters()
{
  target_frame_ = get_parameter("target_frame").as_string();
  cloud_topic_ = get_parameter("cloud_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();

  grid_config_.map_size = get_parameter("map_size").as_double();
  grid_config_.sub_resolution = get_parameter("sub_resolution").as_double();
  grid_config_.plan_resolution = get_parameter("plan_resolution").as_double();
  grid_config_.delta_h_min = get_parameter("delta_h_min").as_double();
  grid_config_.delta_h_max = get_parameter("delta_h_max").as_double();
  grid_config_.cost_gain = get_parameter("cost_gain").as_double();
  grid_config_.lethal_cost =
    static_cast<int8_t>(get_parameter("lethal_cost").as_int());
  grid_config_.temporal_decay = get_parameter("temporal_decay").as_double();
  grid_config_.unobserved_margin_cost =
    get_parameter("unobserved_margin_cost").as_double();
  grid_config_.enable_slope_correction =
    get_parameter("enable_slope_correction").as_bool();
  grid_config_.slope_allow_deg = get_parameter("slope_allow_deg").as_double();
  grid_config_.min_points_per_cell =
    static_cast<int>(get_parameter("min_points_per_cell").as_int());

  angle_min_ = get_parameter("scan.angle_min").as_double();
  angle_max_ = get_parameter("scan.angle_max").as_double();
  angle_increment_deg_ = get_parameter("scan.angle_increment_deg").as_double();
  range_min_ = get_parameter("scan.range_min").as_double();
  range_max_ = get_parameter("scan.range_max").as_double();

  roi_x_min_ = get_parameter("roi.x_min").as_double();
  roi_x_max_ = get_parameter("roi.x_max").as_double();
  roi_y_min_ = get_parameter("roi.y_min").as_double();
  roi_y_max_ = get_parameter("roi.y_max").as_double();
  z_ground_min_ = get_parameter("roi.z_ground_min").as_double();
  z_robot_height_ = get_parameter("roi.z_robot_height").as_double();

  enable_self_filter_ = get_parameter("self_filter.enable").as_bool();
  self_x_min_ = get_parameter("self_filter.x_min").as_double();
  self_x_max_ = get_parameter("self_filter.x_max").as_double();
  self_y_min_ = get_parameter("self_filter.y_min").as_double();
  self_y_max_ = get_parameter("self_filter.y_max").as_double();
  self_z_min_ = get_parameter("self_filter.z_min").as_double();
  self_z_max_ = get_parameter("self_filter.z_max").as_double();

  tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
}

void ElevationCostmapNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto t0 = std::chrono::steady_clock::now();

  std::vector<PointXYZ> points;
  if (!transformCloud(*msg, points)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_.update(points);
    publishScan(msg->header.stamp);
    ++processed_clouds_;
  }

  const auto dt_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  if (dt_ms > 15.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "scan update took %.1f ms (> 15 ms target), points=%zu",
      dt_ms, points.size());
  } else {
    RCLCPP_DEBUG(
      get_logger(), "scan update %.1f ms, points=%zu", dt_ms, points.size());
  }
}

bool ElevationCostmapNode::transformCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::vector<PointXYZ> & out_points)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      target_frame_, cloud.header.frame_id, cloud.header.stamp,
      rclcpp::Duration::from_seconds(tf_timeout_sec_));
  } catch (const tf2::TransformException & ex) {
    // Fallback: latest transform
    try {
      tf = tf_buffer_->lookupTransform(
        target_frame_, cloud.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s <- %s failed: %s", target_frame_.c_str(),
        cloud.header.frame_id.c_str(), ex2.what());
      return false;
    }
  }

  filterAndExtract(cloud, tf, out_points);
  return true;
}

void ElevationCostmapNode::filterAndExtract(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const geometry_msgs::msg::TransformStamped & tf_msg,
  std::vector<PointXYZ> & out_points)
{
  CloudFieldOffsets off;
  if (!findXyzOffsets(cloud, off)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "PointCloud2 missing x/y/z fields");
    return;
  }

  const tf2::Transform tf = toTf2(tf_msg.transform);
  const std::size_t n = static_cast<std::size_t>(cloud.width) * cloud.height;
  out_points.clear();
  out_points.reserve(n / 4);

  for (std::size_t i = 0; i < n; ++i) {
    const uint8_t * ptr = &cloud.data[i * static_cast<std::size_t>(off.point_step)];
    const float x = readFloat(ptr + off.x);
    const float y = readFloat(ptr + off.y);
    const float z = readFloat(ptr + off.z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    const tf2::Vector3 p_out = tf * tf2::Vector3(x, y, z);
    const float ox = static_cast<float>(p_out.x());
    const float oy = static_cast<float>(p_out.y());
    const float oz = static_cast<float>(p_out.z());

    if (ox < roi_x_min_ || ox > roi_x_max_ ||
      oy < roi_y_min_ || oy > roi_y_max_ ||
      oz < z_ground_min_ || oz > z_robot_height_)
    {
      continue;
    }

    if (enable_self_filter_ &&
      ox >= self_x_min_ && ox <= self_x_max_ &&
      oy >= self_y_min_ && oy <= self_y_max_ &&
      oz >= self_z_min_ && oz <= self_z_max_)
    {
      continue;
    }

    out_points.push_back(PointXYZ{ox, oy, oz});
  }
}

void ElevationCostmapNode::publishScan(const rclcpp::Time & stamp)
{
  const double angle_increment = angle_increment_deg_ * M_PI / 180.0;

  grid_.fillLaserScanRanges(
    scan_ranges_, angle_min_, angle_increment, range_min_, range_max_);

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = target_frame_;
  scan.angle_min = static_cast<float>(angle_min_);
  scan.angle_max = static_cast<float>(angle_max_);
  scan.angle_increment = static_cast<float>(angle_increment);
  scan.time_increment = 0.f;
  scan.scan_time = 0.f;
  scan.range_min = static_cast<float>(range_min_);
  scan.range_max = static_cast<float>(range_max_);
  scan.ranges = scan_ranges_;
  scan_pub_->publish(scan);
}

}  // namespace elevation_costmap
