# elevation_costmap

Livox Mid-360 点群から高低差コストマップ（2.5D Elevation Costmap）を生成し、価値反復プランナ向けに `nav_msgs/OccupancyGrid` を配信する ROS 2（Humble）C++ パッケージです。

詳細仕様は [`requirement.txt`](requirement.txt) を参照してください。

## 機能

- ROI / 自己干渉除去 / TF 同期変換
- 内部 0.05 m サブグリッドで Δh 算出（勾配補正あり）
- 最大コストプーリングで 0.15 m（27×27）計画グリッドへダウンサンプル
- 欠損セルの時系列減衰 + 隣接補間
- OpenMP によるセル並列処理

## ビルド

```bash
cd /path/to/ws
colcon build --packages-select elevation_costmap
source install/setup.bash
```

## 起動

```bash
ros2 launch elevation_costmap elevation_costmap.launch.py
```

パラメータは `config/params.yaml` で調整できます。

## 入出力

| 種別 | 名前 | 型 |
|------|------|-----|
| Subscribe | `/livox/lidar`（変更可） | `sensor_msgs/PointCloud2` |
| Publish | `/local_costmap/costmap_raw` | `nav_msgs/OccupancyGrid` |
| TF | `target_frame` ← cloud frame | 水平補正済 `base_link` 推奨 |

### OccupancyGrid セル値

- `0` … 平坦
- `1`–`99` … 段差コスト（線形）
- `100` … Lethal（走破不能）
- `-1` … 未観測（初回のみ）

## 座標系

点群は水平基準フレーム（`base_link` または `odom`）へ変換してから処理します。マップは自機中心のローリンググリッド（約 4 m × 4 m）です。
