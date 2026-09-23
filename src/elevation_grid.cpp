#include "elevation_costmap/elevation_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace elevation_costmap
{

namespace
{
constexpr float kInfZ = std::numeric_limits<float>::infinity();

inline int clampi(int v, int lo, int hi)
{
  return std::max(lo, std::min(v, hi));
}

inline float clampf(float v, float lo, float hi)
{
  return std::max(lo, std::min(v, hi));
}
}  // namespace

ElevationGrid::ElevationGrid(const GridConfig & config)
{
  setConfig(config);
}

void ElevationGrid::setConfig(const GridConfig & config)
{
  config_ = config;
  rebuildGeometry();
}

void ElevationGrid::rebuildGeometry()
{
  sub_width_ = static_cast<int>(std::lround(config_.map_size / config_.sub_resolution));
  sub_height_ = sub_width_;
  plan_width_ = static_cast<int>(std::lround(config_.map_size / config_.plan_resolution));
  // Prefer odd planning size so the robot sits near the map center (27 for 4.05 m).
  if (plan_width_ % 2 == 0) {
    ++plan_width_;
  }
  plan_height_ = plan_width_;

  pool_ = std::max(
    1, static_cast<int>(std::lround(config_.plan_resolution / config_.sub_resolution)));

  const double plan_extent = plan_width_ * config_.plan_resolution;
  origin_x_ = -0.5 * plan_extent;
  origin_y_ = -0.5 * plan_extent;

  // Align sub-grid origin to the same lower-left; extend coverage to cover plan extent.
  const int needed_sub =
    static_cast<int>(std::ceil(plan_extent / config_.sub_resolution));
  sub_width_ = std::max(sub_width_, needed_sub);
  sub_height_ = sub_width_;

  const std::size_t sub_n = static_cast<std::size_t>(sub_width_ * sub_height_);
  const std::size_t plan_n = static_cast<std::size_t>(plan_width_ * plan_height_);

  z_min_.assign(sub_n, kInfZ);
  z_max_.assign(sub_n, -kInfZ);
  counts_.assign(sub_n, 0);
  sub_delta_h_.assign(sub_n, 0.f);
  sub_cost_.assign(sub_n, 0.f);
  sub_observed_.assign(sub_n, 0);
  plan_cost_f_.assign(plan_n, 0.f);
  plan_observed_.assign(plan_n, 0);
  plan_costs_.assign(plan_n, config_.unknown_cost);
  has_history_ = false;
}

void ElevationGrid::resetSubAccumulators()
{
  const int n = static_cast<int>(z_min_.size());
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int i = 0; i < n; ++i) {
    const std::size_t idx = static_cast<std::size_t>(i);
    z_min_[idx] = kInfZ;
    z_max_[idx] = -kInfZ;
    counts_[idx] = 0;
    sub_observed_[idx] = 0;
  }
}

void ElevationGrid::accumulatePoints(const std::vector<PointXYZ> & points)
{
  const float res = static_cast<float>(config_.sub_resolution);
  const float ox = static_cast<float>(origin_x_);
  const float oy = static_cast<float>(origin_y_);
  const float x_max = ox + static_cast<float>(sub_width_) * res;
  const float y_max = oy + static_cast<float>(sub_height_) * res;

  for (const auto & p : points) {
    if (p.x < ox || p.y < oy || p.x >= x_max || p.y >= y_max) {
      continue;
    }
    const int u = static_cast<int>(std::floor((p.x - ox) / res));
    const int v = static_cast<int>(std::floor((p.y - oy) / res));
    if (u < 0 || v < 0 || u >= sub_width_ || v >= sub_height_) {
      continue;
    }
    const std::size_t idx = static_cast<std::size_t>(v * sub_width_ + u);
    ++counts_[idx];
    if (p.z < z_min_[idx]) {
      z_min_[idx] = p.z;
    }
    if (p.z > z_max_[idx]) {
      z_max_[idx] = p.z;
    }
  }
}

void ElevationGrid::computeSubCosts()
{
  const float cell = static_cast<float>(config_.sub_resolution);
  const float allow_rad = static_cast<float>(config_.slope_allow_deg * M_PI / 180.0);
  const float tan_allow = std::tan(allow_rad);
  const float dh_min = static_cast<float>(config_.delta_h_min);
  const float dh_max = static_cast<float>(config_.delta_h_max);
  const float gain = static_cast<float>(config_.cost_gain);
  const float lethal = static_cast<float>(config_.lethal_cost);
  const int min_pts = config_.min_points_per_cell;

  std::vector<float> mean_z(static_cast<std::size_t>(sub_width_ * sub_height_), 0.f);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int v = 0; v < sub_height_; ++v) {
    for (int u = 0; u < sub_width_; ++u) {
      const std::size_t idx = static_cast<std::size_t>(v * sub_width_ + u);
      if (counts_[idx] < min_pts) {
        sub_delta_h_[idx] = 0.f;
        sub_cost_[idx] = 0.f;
        sub_observed_[idx] = 0;
        continue;
      }
      sub_observed_[idx] = 1;
      const float dz = z_max_[idx] - z_min_[idx];
      mean_z[idx] = 0.5f * (z_max_[idx] + z_min_[idx]);
      sub_delta_h_[idx] = dz;
    }
  }

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int v = 0; v < sub_height_; ++v) {
    for (int u = 0; u < sub_width_; ++u) {
      const std::size_t idx = static_cast<std::size_t>(v * sub_width_ + u);
      if (!sub_observed_[idx]) {
        continue;
      }

      float delta_h = sub_delta_h_[idx];

      if (config_.enable_slope_correction) {
        float slope_tan = 0.f;
        int neigh = 0;
        const int uu[4] = {u - 1, u + 1, u, u};
        const int vv[4] = {v, v, v - 1, v + 1};
        for (int k = 0; k < 4; ++k) {
          const int nu = uu[k];
          const int nv = vv[k];
          if (nu < 0 || nv < 0 || nu >= sub_width_ || nv >= sub_height_) {
            continue;
          }
          const std::size_t nidx = static_cast<std::size_t>(nv * sub_width_ + nu);
          if (!sub_observed_[nidx]) {
            continue;
          }
          const float dh = std::fabs(mean_z[idx] - mean_z[nidx]);
          slope_tan = std::max(slope_tan, dh / cell);
          ++neigh;
        }
        if (neigh > 0 && slope_tan <= tan_allow) {
          delta_h = std::max(0.f, delta_h - cell * slope_tan);
        }
      }

      sub_delta_h_[idx] = delta_h;

      float cost = 0.f;
      if (delta_h < dh_min) {
        cost = 0.f;
      } else if (delta_h > dh_max) {
        cost = lethal;
      } else {
        cost = gain * (delta_h - dh_min) / (dh_max - dh_min);
        cost = clampf(cost, 1.f, gain);
      }
      sub_cost_[idx] = cost;
    }
  }
}

void ElevationGrid::downsampleToPlan()
{
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int pv = 0; pv < plan_height_; ++pv) {
    for (int pu = 0; pu < plan_width_; ++pu) {
      const std::size_t pidx = static_cast<std::size_t>(pv * plan_width_ + pu);
      float max_cost = 0.f;
      bool any = false;

      const int u0 = pu * pool_;
      const int v0 = pv * pool_;
      for (int dv = 0; dv < pool_; ++dv) {
        for (int du = 0; du < pool_; ++du) {
          const int u = u0 + du;
          const int v = v0 + dv;
          if (u >= sub_width_ || v >= sub_height_) {
            continue;
          }
          const std::size_t sidx = static_cast<std::size_t>(v * sub_width_ + u);
          if (!sub_observed_[sidx]) {
            continue;
          }
          any = true;
          max_cost = std::max(max_cost, sub_cost_[sidx]);
        }
      }

      plan_observed_[pidx] = any ? 1 : 0;
      if (any) {
        plan_cost_f_[pidx] = max_cost;
      }
    }
  }
}

void ElevationGrid::applyTemporalFill()
{
  const float decay = static_cast<float>(config_.temporal_decay);
  const float margin = static_cast<float>(config_.unobserved_margin_cost);
  const float lethal = static_cast<float>(config_.lethal_cost);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int i = 0; i < plan_width_ * plan_height_; ++i) {
    const std::size_t idx = static_cast<std::size_t>(i);

    if (plan_observed_[idx]) {
      // keep fresh observation in plan_cost_f_
    } else if (has_history_) {
      plan_cost_f_[idx] = std::min(lethal, plan_cost_f_[idx] * decay);
      float sum = 0.f;
      int n = 0;
      const int u = i % plan_width_;
      const int v = i / plan_width_;
      for (int dv = -1; dv <= 1; ++dv) {
        for (int du = -1; du <= 1; ++du) {
          if (du == 0 && dv == 0) {
            continue;
          }
          const int nu = u + du;
          const int nv = v + dv;
          if (nu < 0 || nv < 0 || nu >= plan_width_ || nv >= plan_height_) {
            continue;
          }
          const std::size_t nidx = static_cast<std::size_t>(nv * plan_width_ + nu);
          if (plan_observed_[nidx]) {
            sum += plan_cost_f_[nidx];
            ++n;
          }
        }
      }
      if (n > 0) {
        const float avg = sum / static_cast<float>(n);
        plan_cost_f_[idx] = std::max(plan_cost_f_[idx], avg * decay);
      }
    } else {
      plan_cost_f_[idx] = margin;
    }

    if (!plan_observed_[idx] && !has_history_) {
      plan_costs_[idx] = config_.unknown_cost;
    } else if (plan_cost_f_[idx] >= lethal - 0.5f) {
      plan_costs_[idx] = config_.lethal_cost;
    } else if (plan_cost_f_[idx] < 0.5f) {
      plan_costs_[idx] = 0;
    } else {
      plan_costs_[idx] = static_cast<int8_t>(
        clampi(static_cast<int>(std::lround(plan_cost_f_[idx])), 1, 99));
    }
  }

  has_history_ = true;
}

const std::vector<int8_t> & ElevationGrid::update(const std::vector<PointXYZ> & points)
{
  resetSubAccumulators();
  accumulatePoints(points);
  computeSubCosts();
  downsampleToPlan();
  applyTemporalFill();
  return plan_costs_;
}

void ElevationGrid::fillLaserScanRanges(
  std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double range_min,
  double range_max) const
{
  if (ranges.empty() || angle_increment <= 0.0) {
    return;
  }

  const float init_r = static_cast<float>(range_max + 1.0);
  std::fill(ranges.begin(), ranges.end(), init_r);

  const float dh_max = static_cast<float>(config_.delta_h_max);
  const float res = static_cast<float>(config_.sub_resolution);
  const float ox = static_cast<float>(origin_x_);
  const float oy = static_cast<float>(origin_y_);
  const int n_beams = static_cast<int>(ranges.size());
  const float r_min = static_cast<float>(range_min);
  const float r_max = static_cast<float>(range_max);

  for (int v = 0; v < sub_height_; ++v) {
    for (int u = 0; u < sub_width_; ++u) {
      const std::size_t idx = static_cast<std::size_t>(v * sub_width_ + u);
      if (!sub_observed_[idx] || sub_delta_h_[idx] <= dh_max) {
        continue;
      }

      const float xc = ox + (static_cast<float>(u) + 0.5f) * res;
      const float yc = oy + (static_cast<float>(v) + 0.5f) * res;
      const float r = std::sqrt(xc * xc + yc * yc);
      if (r < r_min || r > r_max) {
        continue;
      }

      const float phi = std::atan2(yc, xc);
      int k = static_cast<int>(std::floor((phi - angle_min) / angle_increment));
      if (k < 0 || k >= n_beams) {
        // Wrap φ == angle_max onto the last bin when spanning a full circle.
        if (k == n_beams && std::fabs(phi - (angle_min + n_beams * angle_increment)) < 1e-5) {
          k = 0;
        } else {
          continue;
        }
      }
      ranges[static_cast<std::size_t>(k)] = std::min(ranges[static_cast<std::size_t>(k)], r);
    }
  }
}

}  // namespace elevation_costmap
