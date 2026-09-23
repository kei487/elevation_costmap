#ifndef ELEVATION_COSTMAP__ELEVATION_GRID_HPP_
#define ELEVATION_COSTMAP__ELEVATION_GRID_HPP_

#include <cstdint>
#include <vector>

namespace elevation_costmap
{

struct GridConfig
{
  double map_size{4.0};                 // [m] square side length
  double sub_resolution{0.05};          // [m] internal aggregation
  double plan_resolution{0.15};         // [m] planning output
  double delta_h_min{0.03};             // [m] noise floor
  double delta_h_max{0.15};             // [m] lethal threshold
  double cost_gain{99.0};               // linear gain for OccupancyGrid 1..99
  int8_t lethal_cost{100};              // lethal cell value
  int8_t unknown_cost{-1};              // unobserved / blind-spot margin
  double temporal_decay{0.85};          // retain previous cost when cell empty
  double unobserved_margin_cost{50.0};  // safety cost for never-seen cells
  bool enable_slope_correction{true};
  double slope_allow_deg{12.0};         // continuous slope below this is ignored
  int min_points_per_cell{2};
};

struct PointXYZ
{
  float x{0.f};
  float y{0.f};
  float z{0.f};
};

/**
 * @brief Dual-resolution elevation costmap (0.05 m sub-grid -> 0.15 m plan-grid).
 *
 * Robot-centered rolling grid in a levelled frame (base_link / odom).
 */
class ElevationGrid
{
public:
  explicit ElevationGrid(const GridConfig & config = GridConfig{});

  void setConfig(const GridConfig & config);
  const GridConfig & config() const { return config_; }

  int subWidth() const { return sub_width_; }
  int subHeight() const { return sub_height_; }
  int planWidth() const { return plan_width_; }
  int planHeight() const { return plan_height_; }

  /** Origin of both grids relative to robot center (lower-left corner). */
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

  /**
   * @brief Aggregate points, compute delta-h costs, downsample, apply temporal fill.
   * @param points Points already transformed into the map frame (robot-centered).
   * @return Planning-grid costs (OccupancyGrid row-major: index = y * width + x).
   */
  const std::vector<int8_t> & update(const std::vector<PointXYZ> & points);

  const std::vector<int8_t> & planCosts() const { return plan_costs_; }

  /**
   * @brief Build LaserScan ranges from obstacle cells (Δh > delta_h_max).
   *
   * Obstacle cell centers are projected to polar (r, φ); each beam keeps the
   * nearest range. Unhit beams are left as range_max + 1.0.
   */
  void fillLaserScanRanges(
    std::vector<float> & ranges,
    double angle_min,
    double angle_increment,
    double range_min,
    double range_max) const;

private:
  void rebuildGeometry();
  void resetSubAccumulators();
  void accumulatePoints(const std::vector<PointXYZ> & points);
  void computeSubCosts();
  void downsampleToPlan();
  void applyTemporalFill();

  GridConfig config_;
  int sub_width_{0};
  int sub_height_{0};
  int plan_width_{0};
  int plan_height_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int pool_{3};  // plan_resolution / sub_resolution

  std::vector<float> z_min_;
  std::vector<float> z_max_;
  std::vector<int> counts_;
  std::vector<float> sub_delta_h_;
  std::vector<float> sub_cost_;
  std::vector<uint8_t> sub_observed_;
  std::vector<float> plan_cost_f_;
  std::vector<uint8_t> plan_observed_;
  std::vector<int8_t> plan_costs_;
  bool has_history_{false};
};

}  // namespace elevation_costmap

#endif  // ELEVATION_COSTMAP__ELEVATION_GRID_HPP_
