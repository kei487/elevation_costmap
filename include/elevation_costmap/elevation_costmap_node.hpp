#ifndef ELEVATION_COSTMAP__ELEVATION_COSTMAP_NODE_HPP_
#define ELEVATION_COSTMAP__ELEVATION_COSTMAP_NODE_HPP_

#include <memory>
#include <mutex>
#include <cmath>
#include <string>
#include <vector>

#include "elevation_costmap/elevation_grid.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace elevation_costmap
{

class ElevationCostmapNode : public rclcpp::Node
{
public:
  ElevationCostmapNode();

private:
  void declareParameters();
  void loadParameters();
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool transformCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    std::vector<PointXYZ> & out_points);
  void filterAndExtract(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const geometry_msgs::msg::TransformStamped & tf,
    std::vector<PointXYZ> & out_points);
  void publishScan(const rclcpp::Time & stamp);

  GridConfig grid_config_;
  ElevationGrid grid_;

  std::string target_frame_;
  std::string cloud_topic_;
  std::string scan_topic_;

  // LaserScan (req. §4.1)
  double angle_min_{-M_PI};
  double angle_max_{M_PI};
  double angle_increment_deg_{1.0};
  double range_min_{0.1};
  double range_max_{2.83};

  // ROI [m]
  double roi_x_min_{-2.0};
  double roi_x_max_{2.0};
  double roi_y_min_{-2.0};
  double roi_y_max_{2.0};
  double z_ground_min_{-0.5};
  double z_robot_height_{1.5};

  // Self-filter AABB in target frame [m]
  bool enable_self_filter_{true};
  double self_x_min_{-0.35};
  double self_x_max_{0.35};
  double self_y_min_{-0.25};
  double self_y_max_{0.25};
  double self_z_min_{-0.1};
  double self_z_max_{0.6};

  double tf_timeout_sec_{0.05};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  std::mutex mutex_;
  std::size_t processed_clouds_{0};
  std::vector<float> scan_ranges_;
};

}  // namespace elevation_costmap

#endif  // ELEVATION_COSTMAP__ELEVATION_COSTMAP_NODE_HPP_
