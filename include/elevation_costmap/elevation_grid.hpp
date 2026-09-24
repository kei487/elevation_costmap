#ifndef ELEVATION_COSTMAP__ELEVATION_GRID_HPP_
#define ELEVATION_COSTMAP__ELEVATION_GRID_HPP_

#include <cstdint>
#include <vector>

namespace elevation_costmap
{

struct GridConfig
{
  // Exposed via params.yaml
  double grid_size{4.0};                // [m] square side length
  double resolution{0.05};              // [m] cell size
  double max_delta_h{0.15};             // [m] obstacle threshold
  bool enable_slope_correction{true};
  double max_slope_deg{12.0};           // continuous slope below this is ignored
  int min_points{3};                    // min points per cell

  // Internal (legacy OccupancyGrid path; not parameterized)
  double plan_resolution{0.15};
  double delta_h_min{0.03};
  double cost_gain{99.0};
  int8_t lethal_cost{100};
  int8_t unknown_cost{-1};
  double temporal_decay{0.85};
  double unobserved_margin_cost{50.0};
};

struct PointXYZ
{
  float x{0.f};
  float y{0.f};
  float z{0.f};
};

/**
 * @brief Elevation grid for Δh obstacle detection and LaserScan projection.
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
   * @brief Aggregate points and compute per-cell Δh.
   * @param points Points already transformed into the map frame (robot-centered).
   */
  const std::vector<int8_t> & update(const std::vector<PointXYZ> & points);

  const std::vector<int8_t> & planCosts() const { return plan_costs_; }

  /**
   * @brief Build LaserScan ranges from obstacle cells (Δh > max_delta_h).
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
  int pool_{3};  // plan_resolution / resolution

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
