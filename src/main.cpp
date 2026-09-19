#include "elevation_costmap/elevation_costmap_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elevation_costmap::ElevationCostmapNode>());
  rclcpp::shutdown();
  return 0;
}
