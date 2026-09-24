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

/** Axis-aligned self-filter box in frame_id [m]. */
struct SelfFilterBox
{
  bool enable{true};
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};
  double z_min{0.0};
  double z_max{0.0};

  bool contains(float x, float y, float z) const
  {
    return enable &&
           x >= x_min && x <= x_max &&
           y >= y_min && y <= y_max &&
           z >= z_min && z <= z_max;
  }
};

class ElevationCostmapNode : public rclcpp::Node
{
public:
  ElevationCostmapNode();

private:
  void declareParameters();
  void loadParameters();
  void declareSelfFilter(const std::string & prefix, const SelfFilterBox & defaults);
  void loadSelfFilter(const std::string & prefix, SelfFilterBox & out);
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

  std::string frame_id_;
  std::string pointcloud_topic_;
  std::string scan_topic_;

  // LaserScan
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
  double roi_z_min_{-0.5};
  double roi_z_max_{1.5};

  // Self-filter AABBs in frame_id [m]
  SelfFilterBox robot_filter_;
  SelfFilterBox operator_filter_;

  double tf_timeout_{0.05};

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
