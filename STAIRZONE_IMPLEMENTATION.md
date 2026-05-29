# Stair Zone Costmap Filter — Implementation Plan

## Problem

The Unitree Go2 Pro navigates a known 2D map using Nav2. When the robot encounters stairs, the LIDAR sees stair edges as obstacles, and the static map marks them as occupied (black). The robot's obstacle avoidance fires and refuses to proceed.

## Solution

A custom Nav2 costmap filter plugin called `StairZoneFilter` that clears all cost (static and dynamic) in designated stair zones. This is the inverse of the existing `KeepoutFilter` — instead of adding cost, it removes cost. The plugin fits into Nav2's costmap filter architecture, reusing the same `CostmapFilterInfo` + `MapServer` mask pipeline.

## Architecture Overview

```
 Nav2 Costmap2D Update Cycle (per tick)
 ───────────────────────────────────────
 1. static_layer    → loads map, marks walls/stairs as occupied (100)
 2. obstacle_layer  → processes /scan, marks LIDAR hits as obstacles
 3. inflation_layer → inflates obstacle costs outward
 4. [filters]       → costmap filters run AFTER all layers
    ├── keepout_filter      → ADDS cost in keepout zones (existing, disabled)
    └── stair_zone_filter   → CLEARS cost in stair zones (NEW)
```

The filter runs last, so it can override costs written by any layer — static, obstacle, or inflation.

## Project Structure After Implementation

```
go2_ros2_sdk/
├── go2_stair_zone_filter/              # NEW C++ package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── stair_zone_filter_plugin.xml    # pluginlib manifest
│   ├── include/
│   │   └── stair_zone_filter.hpp
│   └── src/
│       └── stair_zone_filter.cpp
│
├── go2_robot_sdk/
│   ├── config/
│   │   ├── params_navigation.yaml      # MODIFIED — add stair_zone_filter
│   │   ├── params_keepout.yaml         # existing keepout server params
│   │   └── params_stair_zone.yaml      # NEW — stair zone filter server params
│   ├── launch/
│   │   └── cyclonedds_navigation.launch.py  # MODIFIED — add stair zone nodes
│   └── maps/
│       ├── Studio.yaml
│       ├── Studio_stair_mask.yaml      # NEW — stair zone mask metadata
│       ├── Studio_stair_mask.pgm       # NEW — stair zone mask image
│       ├── ... (one mask per navigable map)
│       └── ...
```

---

## Phase 1: Create the C++ Plugin Package

This phase creates a new `ament_cmake` package with a Nav2 costmap filter plugin. The plugin is modeled after Nav2's own `KeepoutFilter` but with inverted semantics: where the mask is occupied, we clear cost instead of adding it.

> **Nav2 API reference:** This plan is based on the actual Nav2 source code at
> `ros-navigation/navigation2` `main` branch. Key files:
> - `nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/costmap_filter.hpp`
> - `nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/keepout_filter.hpp`
> - `nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp`
> - `nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp`

### 1.1 Key Nav2 CostmapFilter API Facts

Before writing code, understand the actual base class API:

```
CostmapFilter (inherits from Layer)
├── final methods (you CANNOT override):
│   ├── onInitialize()   — reads params: enabled, filter_info_topic, transform_tolerance
│   ├── updateCosts()    — calls your process() then setCurrent(true)
│   ├── activate()       — calls your initializeFilter()
│   ├── deactivate()     — calls your resetFilter()
│   ├── reset()          — calls resetFilter() then initializeFilter() then setCurrent(false)
│   └── isClearable()    — returns false
│
├── pure virtual methods (you MUST implement):
│   ├── initializeFilter(const std::string & filter_info_topic)
│   ├── process(Costmap2D&, int min_i, int min_j, int max_i, int max_j,
│   │            const geometry_msgs::msg::Pose & pose)   ← Pose, not Pose2D
│   └── resetFilter()
│
├── overridable methods:
│   └── updateBounds()    — base stores robot pose in latest_pose_
│
├── protected helpers available to subclasses:
│   ├── transformPose(global_frame, global_pose, mask_frame, mask_pose)
│   ├── getMaskData(OccupancyGrid::ConstSharedPtr, mx, my) → int8_t
│   └── getMaskCost(OccupancyGrid::ConstSharedPtr, mx, my) → unsigned char
│       (converts OccupancyGrid [-1..100] to costmap [0..254 or 255 NO_INFORMATION])
│
├── protected members:
│   ├── filter_info_topic_  (std::string)
│   ├── mask_topic_         (std::string)
│   └── transform_tolerance_ (tf2::Duration)
│
└── inherited from Layer:
    ├── layered_costmap_    → getGlobalFrameID(), getSizeInMetersX/Y(), etc.
    ├── name_               → the filter's YAML key name
    ├── tf_                 → TF buffer
    ├── logger_             → RCLCPP logging
    ├── node_               → weak_ptr to LifecycleNode (use node_.lock())
    ├── enabled_            → bool, toggleable via <name>/toggle_filter service
    └── getMutex()          → std::recursive_mutex* for thread safety
```

**Plugin export:** Register as `nav2_costmap_2d::Layer` (not `CostmapFilter`). This is how Nav2's own KeepoutFilter does it.

**Mask subscription:** The base class does NOT subscribe to the mask for you. Your `initializeFilter()` must manually subscribe to the `CostmapFilterInfo` topic, extract the mask topic name from it, then subscribe to the mask `OccupancyGrid`. This is the same pattern `KeepoutFilter` uses.

**Mask access:** Store the mask as `nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask_` in your class. Use `nav2_util::worldToMap(filter_mask_, wx, wy, mx, my)` to convert world coords to mask cell coords. Use `getMaskCost(filter_mask_, mx, my)` to read mask values as costmap costs.

### 1.2 Package scaffold — `go2_stair_zone_filter/`

**`package.xml`**
```xml
<?xml version="1.0"?>
<package format="3">
  <name>go2_stair_zone_filter</name>
  <version>0.1.0</version>
  <description>Nav2 costmap filter that clears obstacles in designated stair zones</description>
  <maintainer email="your@email.com">Ibrahim Fadhil Djauhari</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_util</depend>
  <depend>nav2_msgs</depend>
  <depend>pluginlib</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_costmap_2d plugin="${prefix}/stair_zone_filter_plugin.xml" />
  </export>
</package>
```

**`CMakeLists.txt`**
```cmake
cmake_minimum_required(VERSION 3.10)
project(go2_stair_zone_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(stair_zone_filter SHARED
  src/stair_zone_filter.cpp
)
target_include_directories(stair_zone_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(stair_zone_filter PUBLIC cxx_std_17)

ament_target_dependencies(stair_zone_filter
  rclcpp nav2_costmap_2d nav2_util nav2_msgs pluginlib
  geometry_msgs nav_msgs tf2 tf2_ros
)

pluginlib_export_plugin_description_file(nav2_costmap_2d
  stair_zone_filter_plugin.xml
)

install(TARGETS stair_zone_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
```

**`stair_zone_filter_plugin.xml`**
```xml
<library path="stair_zone_filter">
  <class type="go2_stair_zone_filter::StairZoneFilter"
         base_class_type="nav2_costmap_2d::Layer">
    <description>
      Clears all costmap cost in designated stair zones, overriding
      static obstacles and dynamic laser scan detections.
    </description>
  </class>
</library>
```

### 1.3 Plugin header — `include/stair_zone_filter.hpp`

```cpp
#ifndef STAIR_ZONE_FILTER_HPP_
#define STAIR_ZONE_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace go2_stair_zone_filter
{

// Mirrors the subscription pattern used by KeepoutFilter.
// CostmapFilter base class does NOT subscribe to the mask for you.

class StairZoneFilter : public nav2_costmap_2d::CostmapFilter
{
public:
  StairZoneFilter() = default;
  ~StairZoneFilter() override = default;

  // --- CostmapFilter pure virtual interface ---

  void initializeFilter(const std::string & filter_info_topic) override;

  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose & pose) override;

  void resetFilter() override;

  // --- Overridable ---

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

private:
  // Mask subscription — same pattern as KeepoutFilter
  void filterInfoCallback(nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
  void maskCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool worldToMask(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr mask,
    double wx, double wy, unsigned int & mx, unsigned int & my) const;

  // Subscriptions
  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  // Stored mask data
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask_;
  std::string global_frame_;
  bool has_updated_data_{false};

  // Bounds expansion on new mask data
  unsigned int mask_x_{0}, mask_y_{0}, mask_width_{0}, mask_height_{0};
};

}  // namespace go2_stair_zone_filter

#endif  // STAIR_ZONE_FILTER_HPP_
```

### 1.4 Plugin implementation — `src/stair_zone_filter.cpp`

```cpp
#include "stair_zone_filter.hpp"
#include <mutex>
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(go2_stair_zone_filter::StairZoneFilter,
                       nav2_costmap_2d::Layer)

namespace go2_stair_zone_filter
{

void StairZoneFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  RCLCPP_INFO(logger_, "StairZoneFilter: initializing on topic '%s'",
              filter_info_topic.c_str());

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("StairZoneFilter: unable to lock node_);
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Subscribe to CostmapFilterInfo (latched QoS to get the one message)
  auto qos = rclcpp::QoS(1).transient_local().reliable();
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic, qos,
    std::bind(&StairZoneFilter::filterInfoCallback, this, std::placeholders::_1));
}

void StairZoneFilter::filterInfoCallback(
  nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  // The info message tells us which topic has the mask OccupancyGrid.
  // Subscribe to it (latched).
  auto node = node_.lock();
  if (!node) { return; }

  mask_topic_ = msg->filter_mask_topic;

  auto qos = rclcpp::QoS(1).transient_local().reliable();
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, qos,
    std::bind(&StairZoneFilter::maskCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "StairZoneFilter: subscribed to mask topic '%s'",
              mask_topic_.c_str());
}

void StairZoneFilter::maskCallback(
  nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(*getMutex());
  filter_mask_ = msg;
  has_updated_data_ = true;

  // Store mask bounds for updateBounds expansion
  mask_x_ = 0;
  mask_y_ = 0;
  mask_width_ = msg->info.width;
  mask_height_ = msg->info.height;

  RCLCPP_INFO(logger_, "StairZoneFilter: received mask %ux%u",
              msg->info.width, msg->info.height);
}

bool StairZoneFilter::worldToMask(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr mask,
  double wx, double wy,
  unsigned int & mx, unsigned int & my) const
{
  // Same calculation as nav2_util::worldToMap for OccupancyGrid
  const double ox = mask->info.origin.position.x;
  const double oy = mask->info.origin.position.y;
  const double res = mask->info.resolution;

  if (res <= 0.0) { return false; }

  const double dx = wx - ox;
  const double dy = wy - oy;
  mx = static_cast<unsigned int>(std::floor(dx / res));
  my = static_cast<unsigned int>(std::floor(dy / res));

  return mx < mask->info.width && my < mask->info.height;
}

void StairZoneFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  // Let base class store robot pose first
  nav2_costmap_2d::CostmapFilter::updateBounds(
    robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  // When a new mask arrives, expand bounds to cover the entire mask
  // so the filter processes all cells in the overlapping region.
  if (has_updated_data_ && filter_mask_) {
    const double res = filter_mask_->info.resolution;
    const double ox = filter_mask_->info.origin.position.x;
    const double oy = filter_mask_->info.origin.position.y;
    *min_x = std::min(*min_x, ox);
    *min_y = std::min(*min_y, oy);
    *max_x = std::max(*max_x, ox + mask_width_ * res);
    *max_y = std::max(*max_y, oy + mask_height_ * res);
    has_updated_data_ = false;
  }
}

void StairZoneFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::msg::Pose & /*pose*/)
{
  if (!filter_mask_ || !enabled_) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(*getMutex());

  unsigned char * master_array = master_grid.getCharMap();
  const unsigned int master_size_x = master_grid.getSizeInCellsX();

  const std::string mask_frame = filter_mask_->header.frame_id;
  const bool same_frame = (mask_frame == global_frame_);

  // Handle cross-frame case: pre-compute transform if mask frame
  // differs from costmap global frame.
  // For most setups the mask is in "map" frame and global_frame_ is
  // also "map" (global_costmap) or "odom" (local_costmap).
  // Same-frame path is the common case and is much faster.
  geometry_msgs::msg::TransformStamped transform;
  if (!same_frame) {
    try {
      transform = tf_->lookupTransform(
        mask_frame, global_frame_, tf2::TimePointZero, transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "StairZoneFilter: TF failed: %s", ex.what());
      return;
    }
  }

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      // Convert master_grid cell (i,j) to world coordinates
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      // If mask is in a different frame, transform the world point
      double msk_wx = wx, msk_wy = wy;
      if (!same_frame) {
        const auto & t = transform.transform;
        double tx = t.translation.x;
        double ty = t.translation.y;
        double c = t.rotation.z;  // simplified for 2D: cos = w, sin = z
        double s = t.rotation.w;
        // Actually use proper quaternion-to-yaw for 2D rotation
        double cos_theta = 1.0 - 2.0 * (t.rotation.z * t.rotation.z);
        double sin_theta = 2.0 * (t.rotation.w * t.rotation.z);
        msk_wx = cos_theta * wx - sin_theta * wy + tx;
        msk_wy = sin_theta * wx + cos_theta * wy + ty;
      }

      // Convert world coords to mask cell
      unsigned int mx, my;
      if (!worldToMask(filter_mask_, msk_wx, msk_wy, mx, my)) {
        continue;  // outside mask bounds
      }

      // Read mask value using base class helper
      // getMaskCost converts OccupancyGrid data [-1..100] to costmap costs:
      //   -1 (unknown) -> 255 (NO_INFORMATION)
      //   0  (free)    -> 0   (FREE_SPACE)
      //   100(occupied)-> 254 (LETHAL_OBSTACLE)
      unsigned char mask_cost = getMaskCost(filter_mask_, mx, my);

      // In our mask:
      //   LETHAL_OBSTACLE (254) = stair zone → clear master cost
      //   FREE_SPACE (0)        = normal area → do nothing
      //   NO_INFORMATION (255)  = unknown     → do nothing
      if (mask_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        unsigned int master_index = master_grid.getIndex(i, j);
        master_array[master_index] = nav2_costmap_2d::FREE_SPACE;
      }
    }
  }
}

void StairZoneFilter::resetFilter()
{
  std::lock_guard<std::recursive_mutex> lock(*getMutex());
  filter_info_sub_.reset();
  mask_sub_.reset();
  filter_mask_.reset();
  RCLCPP_INFO(logger_, "StairZoneFilter: reset");
}

}  // namespace go2_stair_zone_filter
```

### 1.5 Build and verify

```bash
cd ~/go2_ros2_sdk
colcon build --packages-select go2_stair_zone_filter
source install/setup.bash

# Verify plugin is discoverable by Nav2:
ros2 pkg plugins nav2_costmap_2d | grep stair_zone
# Expected:
#   go2_stair_zone_filter/StairZoneFilter (nav2_costmap_2d::Layer)
```

---

## Phase 2: Create Stair Zone Masks

### 2.1 Mask file convention

For each navigation map (e.g., `Studio.yaml`), create a corresponding mask:

| Navigation map | Stair zone mask |
|---|---|
| `Studio.yaml` | `Studio_stair_mask.yaml` + `Studio_stair_mask.pgm` |
| `Studio_Pintu.yaml` | `Studio_Pintu_stair_mask.yaml` + `Studio_Pintu_stair_mask.pgm` |
| `nav_lobby.yaml` | `nav_lobby_stair_mask.yaml` + `nav_lobby_stair_mask.pgm` |

### 2.2 Mask image format

The mask is a standard PGM image matching the navigation map dimensions and resolution:

- **White (255 / OccupancyGrid 0)** = normal area — filter does nothing
- **Black (0 / OccupancyGrid 100)** = stair zone — filter clears all cost
- Use `trinary` mode in the YAML metadata so the thresholding is clean

> **Why use "occupied" (black) for stair zones?** Because the filter code checks `if (mask_cost == LETHAL_OBSTACLE)`. The `getMaskCost()` helper converts OccupancyGrid 100 to costmap `LETHAL_OBSTACLE` (254). This way we reuse Nav2's standard mask encoding — we just interpret it with inverted semantics (clear instead of block).

### 2.3 How to create a mask image

**Step-by-step with GIMP (or any raster editor):**

1. Copy the navigation map PGM as a starting point:
   ```bash
   cp go2_robot_sdk/maps/Studio.pgm go2_robot_sdk/maps/Studio_stair_mask.pgm
   ```

2. Open `Studio_stair_mask.pgm` in GIMP

3. **Flood-fill the entire image with white (255)** — this is the default "no stair zone" state

4. **Paint stair areas with black (0)** — these are the stair zones. Be precise:
   - Only cover the stair tread area and immediate approach/departure zones
   - Do NOT extend into corridors or open areas
   - Keep a ~0.5m buffer around the actual stair edges to ensure clean transition

5. Export as PGM, plain format, no compression

### 2.4 Mask YAML metadata

Create `Studio_stair_mask.yaml`:
```yaml
image: Studio_stair_mask.pgm
mode: trinary
resolution: 0.07      # MUST match the navigation map resolution
origin: [-11.2, -9.8, 0.0]  # MUST match the navigation map origin
negate: false
occupied_thresh: 0.65
free_thresh: 0.25
```

> **Critical:** `resolution` and `origin` must exactly match the navigation map's values. The `StairZoneFilter::process()` method uses world-coordinate alignment to overlay the mask on the costmap. A mismatch will clear the wrong cells.

Current map resolutions for reference:
| Map | Resolution | Origin |
|---|---|---|
| `Studio.yaml` | 0.07 | [-11.2, -9.8, 0] |
| `Studio_Pintu.yaml` | 0.07 | [-6.63, -8.05, 0] |
| `studio-1.yaml` | 0.07 | [-7.17, -9.45, 0] |
| `nav_lobby.yaml` | 0.07 | [-2.49, -4.24, 0] |
| `Lobby-1.yaml` | 0.05 | [-6.32, -8.68, 0] |
| `FullMap.yaml` | 0.07 | [-5.01, -13.2, 0] |

### 2.5 Verify mask alignment

```bash
# Launch with the mask and check in RViz:
# 1. Add Map display → topic: /stair_zone_filter_mask
# 2. Add Map display → topic: /map
# 3. The stair zone overlay should align precisely with stairs on the map
```

---

## Phase 3: Configure Nav2 Parameters

### 3.1 New file — `config/params_stair_zone.yaml`

```yaml
stair_zone_filter_mask_server:
  ros__parameters:
    topic_name: "/stair_zone_filter_mask"
    # yaml_filename is set dynamically via launch argument

stair_zone_costmap_filter_info_server:
  ros__parameters:
    type: 0              # type=0 means mask values are passed through as-is
                         # (base=0, multiplier=1). Our process() method
                         # interprets LETHAL_OBSTACLE as "clear cost here".
    filter_info_topic: "/stair_zone_costmap_filter_info"
    mask_topic: "/stair_zone_filter_mask"
    base: 0.0
    multiplier: 1.0
```

### 3.2 Modify `config/params_navigation.yaml`

Add `stair_zone_filter` to the `filters` list in both costmaps. The filter must appear **after** `keepout_filter` so that stair zones can override keepout zones if they overlap.

**Local costmap (around line 233):**
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # ... existing params unchanged ...

      filters: ["keepout_filter", "stair_zone_filter"]
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        filter_info_topic: "/keepout_costmap_filter_info"
        enabled: false

      stair_zone_filter:
        plugin: "go2_stair_zone_filter::StairZoneFilter"
        filter_info_topic: "/stair_zone_costmap_filter_info"
        enabled: true
```

**Global costmap (around line 286):**
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # ... existing params unchanged ...

      filters: ["keepout_filter", "stair_zone_filter"]
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        filter_info_topic: "/keepout_costmap_filter_info"
        enabled: false

      stair_zone_filter:
        plugin: "go2_stair_zone_filter::StairZoneFilter"
        filter_info_topic: "/stair_zone_costmap_filter_info"
        enabled: true
```

---

## Phase 4: Modify Launch File

### 4.1 Add stair zone launch arguments

In `cyclonedds_navigation.launch.py`, add to `create_launch_arguments()`:

```python
DeclareLaunchArgument(
    'stair_zone_mask',
    default_value='false',
    description='Enable/disable stair zone mask [boolean]'
),
DeclareLaunchArgument(
    'stair_zone_map',
    default_value=os.path.join(
        os.getcwd(), 'src', 'go2_robot_sdk', 'maps', 'Studio_stair_mask.yaml'
    ),
    description='Absolute path to the stair zone mask yaml file'
),
```

### 4.2 Add config path

In `_get_config_paths()`, add:
```python
'stair_zone': os.path.join(self.go2_package_dir, 'config', 'params_stair_zone.yaml'),
```

### 4.3 Add stair zone nodes

Add a new factory method `create_stair_zone_nodes()`:

```python
def create_stair_zone_nodes(self) -> List[Node]:
    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='stair_zone_filter_mask_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('stair_zone_mask')),
            parameters=[
                {'yaml_filename': LaunchConfiguration('stair_zone_map')},
                self.config.config_paths['stair_zone'],
            ],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='stair_zone_costmap_filter_info_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('stair_zone_mask')),
            parameters=[self.config.config_paths['stair_zone']],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_stair_zone',
            output='screen',
            condition=IfCondition(LaunchConfiguration('stair_zone_mask')),
            parameters=[
                {'use_sim_time': False}, {'autostart': True},
                {'node_names': [
                    'stair_zone_filter_mask_server',
                    'stair_zone_costmap_filter_info_server',
                ]}
            ],
        ),
    ]
```

### 4.4 Wire into `generate_launch_description()`

Add to the factory calls and launch_entities:

```python
stair_zone_nodes = factory.create_stair_zone_nodes()

launch_entities = (
    env_setup +
    launch_args +
    core_nodes +
    state_nodes +
    aggregate_nodes +
    laserscan_nodes +
    teleop_nodes +
    visualization_nodes +
    camera_nodes +
    keepout_nodes +
    stair_zone_nodes +     # <-- add here, after keepout_nodes
    nav2_launches +
    localization_launches
)
```

### 4.5 Update `setup.py` data_files

The new `params_stair_zone.yaml` is already covered by the existing glob in `setup.py`:
```python
(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
```

Mask files in `maps/` are not currently installed by `setup.py`. Add if you want them installed:
```python
(os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
```

---

## Phase 5: Handle Go2 Built-in Obstacle Avoidance

The Go2 has a firmware-level obstacle avoidance system separate from Nav2. Even with the costmap filter clearing stair costs, the robot's internal system may still refuse to walk onto stairs.

### 5.1 Disable via WebRTC API

The project exposes obstacle avoidance control in `go2_robot_sdk/go2_robot_sdk/domain/go2/consts/webrtc_topics.py`:
```python
"OBSTACLES_AVOID": "rt/api/obstacles_avoid/request"
```

### 5.2 Create a stair zone safety node

Create a Python node `go2_stair_zone_safety_node` that:

1. Subscribes to `/amcl_pose` to track robot position
2. Loads stair zone polygons from a YAML config
3. When the robot enters a stair zone:
   - Publishes a disable command to `rt/api/obstacles_avoid/request`
4. When the robot exits a stair zone:
   - Publishes an enable command to `rt/api/obstacles_avoid/request`

**Place in:** `go2_robot_sdk/go2_robot_sdk/presentation/stair_zone_safety_node.py`

```python
"""Monitors robot position and disables Go2 firmware obstacle avoidance
when inside a designated stair zone."""
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from shapely.geometry import Point, Polygon


class StairZoneSafetyNode(Node):
    def __init__(self):
        super().__init__('stair_zone_safety_node')

        self.declare_parameter('stair_zones_file', '')
        self.declare_parameter('obstacle_avoid_topic', 'rt/api/obstacles_avoid/request')

        zones_file = self.get_parameter('stair_zones_file').value
        self.zones = self._load_zones(zones_file)
        self.in_stair_zone = False

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10,
        )
        # TODO: publisher for obstacle avoidance enable/disable commands

    def _load_zones(self, filepath):
        """Load stair zone polygons from YAML.
        Format:
          zones:
            - name: "main_stairs"
              polygon: [[x1,y1], [x2,y2], ...]
        """
        with open(filepath) as f:
            data = yaml.safe_load(f)
        return [Polygon(z['polygon']) for z in data.get('zones', [])]

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point = Point(x, y)

        in_zone = any(z.contains(point) for z in self.zones)

        if in_zone and not self.in_stair_zone:
            self.get_logger().warn('Entering stair zone — disabling obstacle avoidance')
            # TODO: publish disable command
            self.in_stair_zone = True
        elif not in_zone and self.in_stair_zone:
            self.get_logger().info('Leaving stair zone — re-enabling obstacle avoidance')
            # TODO: publish enable command
            self.in_stair_zone = False


def main(args=None):
    rclpy.init(args=args)
    node = StairZoneSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 5.3 Stair zones polygon file

Create `config/stair_zones.yaml`:
```yaml
zones:
  - name: "studio_main_stairs"
    polygon: [[1.2, 3.4], [2.5, 3.4], [2.5, 5.1], [1.2, 5.1]]
  - name: "lobby_stairs_north"
    polygon: [[-3.1, 8.2], [-1.8, 8.2], [-1.8, 10.0], [-3.1, 10.0]]
```

Coordinates come from RViz — use the "Publish Point" tool to click corners of the stair area and record the map-frame coordinates.

---

## Phase 6: Enable Collision Monitor as Safety Net

With obstacle avoidance disabled in stair zones, enable Nav2's collision monitor as a last-resort hardware stop. This uses a tight polygon in front of the robot that triggers an immediate stop if anything physically close is detected.

Uncomment and modify in `config/params_navigation.yaml`:

```yaml
collision_monitor:
  ros__parameters:
    enabled: True
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel"
    cmd_vel_out_topic: "cmd_vel_nav"
    transform_tolerance: 0.3
    source_timeout: 2.0
    stop_pub_timeout: 1.0

    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
      enabled: true
      min_height: 0.05
      max_height: 2.0

    polygons: ["FrontStop"]
    FrontStop:
      type: "polygon"
      points: "[[0.25,0.20],[0.25,-0.20],[0.05,-0.20],[0.05,0.20]]"
      action_type: "stop"
```

Use a very tight polygon (0.25m forward, 0.20m lateral) — this only triggers when something is physically touching or about to touch the robot, not when LIDAR sees distant stair edges.

---

## Phase 7: Testing Plan

### 7.1 Unit test — plugin loads

```bash
# After building, verify the plugin is discoverable:
ros2 pkg plugins nav2_costmap_2d | grep stair_zone
# Expected output:
#   go2_stair_zone_filter/StairZoneFilter (nav2_costmap_2d::Layer)
```

### 7.2 Integration test — mask loads and costmap clears

1. Launch navigation with stair zone mask enabled:
   ```bash
   ros2 launch go2_robot_sdk cyclonedds_navigation.launch.py \
     stair_zone_mask:=true \
     stair_zone_map:=$(pwd)/src/go2_robot_sdk/maps/Studio_stair_mask.yaml \
     rviz:=true
   ```

2. In RViz, verify:
   - `/stair_zone_filter_mask` topic shows the stair zones (black areas)
   - `/global_costmap/costmap` shows the stair zone area as FREE (bright, cost=0)
   - `/local_costmap/costmap` shows the same when the robot is near stairs

3. Teleop the robot toward stairs — it should NOT stop when entering the stair zone

### 7.3 Navigation test — path through stairs

1. Set an initial pose on one side of the stairs
2. Send a nav goal on the other side of the stairs
3. Verify:
   - The SMAC planner generates a path through the stair zone
   - The DWB controller follows the path without stopping
   - The robot physically walks through the stairs

### 7.4 Safety test — dynamic obstacle in stair zone

1. Have a person stand in the stair zone
2. Command the robot through the zone
3. **Expected:** The robot will NOT detect the person via Nav2 (this is the trade-off)
4. If the collision monitor is enabled, it should trigger an emergency stop when the person is within the tight polygon (~0.25m)

---

## Phase 8: Rollout Checklist

- [ ] `go2_stair_zone_filter` package builds with `colcon build`
- [ ] Plugin is registered and discoverable by Nav2 as `nav2_costmap_2d::Layer`
- [ ] Stair zone mask images created for all maps with stairs
- [ ] Mask YAML metadata matches map resolution and origin exactly
- [ ] `params_stair_zone.yaml` created with correct topic names
- [ ] `params_navigation.yaml` updated with `stair_zone_filter` in both costmaps
- [ ] `cyclonedds_navigation.launch.py` updated with stair zone nodes and arguments
- [ ] `setup.py` includes new config files and mask maps
- [ ] Stair zone safety node disables Go2 firmware obstacle avoidance in zones
- [ ] Collision monitor enabled with tight stop polygon
- [ ] `stair_zones.yaml` polygons verified against real stair locations
- [ ] End-to-end navigation test through stairs passes
- [ ] Safety test: collision monitor triggers for close-range obstacles in stair zones

---

## Appendix A: Nav2 Costmap Filter Reference

| Filter Type | `type` value | Behavior | Plugin Class |
|---|---|---|---|
| Keepout | 0 | Adds cost — blocks navigation | `nav2_costmap_2d::KeepoutFilter` |
| Speed | 1 | Sets speed limit in zones | `nav2_costmap_2d::SpeedFilter` |
| **Stair Zone** | **0** (reused) | **Clears cost — makes zones traversable** | **`go2_stair_zone_filter::StairZoneFilter`** |

Our plugin reuses `type: 0` from the info server. The mask values use the same encoding (OccupancyGrid 100 = occupied). The difference is in `process()`: KeepoutFilter writes the mask cost to the costmap (adds obstacles), our StairZoneFilter writes `FREE_SPACE` wherever the mask is occupied (clears obstacles).

### Costmap cost values reference

```
FREE_SPACE                  = 0
...
MAX_NON_OBSTACLE            = 252
INSCRIBED_INFLATED_OBSTACLE = 253
LETHAL_OBSTACLE             = 254
NO_INFORMATION              = 255
```

### getMaskCost() conversion

```
OccupancyGrid data  →  costmap cost
-1 (unknown)         →  255 (NO_INFORMATION)
0  (free)            →  0   (FREE_SPACE)
100 (occupied)       →  254 (LETHAL_OBSTACLE)
```

## Appendix B: Key File Locations

| File | Path |
|---|---|
| Navigation params | `go2_robot_sdk/config/params_navigation.yaml` |
| Keepout params | `go2_robot_sdk/config/params_keepout.yaml` |
| Stair zone params (new) | `go2_robot_sdk/config/params_stair_zone.yaml` |
| Navigation launch | `go2_robot_sdk/launch/cyclonedds_navigation.launch.py` |
| Maps directory | `go2_robot_sdk/maps/` |
| WebRTC topics | `go2_robot_sdk/go2_robot_sdk/domain/go2/consts/webrtc_topics.py` |
| Robot URDF | `go2_robot_sdk/urdf/go2.urdf` |
| Go2 driver | `go2_robot_sdk/go2_robot_sdk/main.py` |

## Appendix C: Costmap Filter Execution Order

The current costmap configuration will be:

```
global_costmap:
  plugins:  [static_layer, obstacle_layer, inflation_layer]
  filters:  [keepout_filter, stair_zone_filter]

Execution order per update tick:
  1. static_layer.updateCosts()     → writes static map obstacles (walls, stairs)
  2. obstacle_layer.updateCosts()   → writes /scan laser hits as obstacles
  3. inflation_layer.updateCosts()  → inflates obstacle costs outward
  4. keepout_filter.updateCosts()   → calls process() → adds cost in keepout zones (disabled)
  5. stair_zone_filter.updateCosts()→ calls process() → clears cost in stair zones (active)
```

> **How filters run:** `CostmapFilter` inherits from `Layer`. Nav2 treats filters identically to layers in the update cycle — it calls `updateCosts()` on each entry in the `plugins` + `filters` lists in order. The base class `updateCosts()` is `final` and internally calls your `process()` method. Since `stair_zone_filter` appears last in the `filters` list, it has the final say on cost values.

Since `stair_zone_filter` runs last, it has the final say. Any cell inside a stair zone will be set to FREE regardless of what the layers wrote. This is the desired behavior.

## Appendix D: Safety Considerations

1. **Minimum stair zone width:** Make stair zones no wider than the actual stair path + 0.5m buffer. Oversized zones clear obstacle detection in areas that aren't stairs.

2. **Speed limiting:** Consider combining the stair zone filter with a speed limit in the same area using Nav2's `SpeedFilter`. The robot should traverse stairs at reduced speed (e.g., 0.2 m/s).

3. **Visual indicators:** In RViz, always display `/stair_zone_filter_mask` alongside `/map` so operators can verify zone boundaries at a glance.

4. **Audit trail:** Log every stair zone entry/exit event with timestamp and position for post-incident analysis.

5. **Human proximity:** The collision monitor tight polygon is the last safety net. Test that it reliably triggers for objects within 0.25m before relying on it.

## Appendix E: Nav2 Source Code Reference

The implementation is based on the following files from the `ros-navigation/navigation2` repository (`main` branch):

| File | GitHub path |
|---|---|
| CostmapFilter base header | `nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/costmap_filter.hpp` |
| CostmapFilter base impl | `nav2_costmap_2d/plugins/costmap_filters/costmap_filter.cpp` |
| KeepoutFilter header | `nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/keepout_filter.hpp` |
| KeepoutFilter impl | `nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp` |
| Cost values | `nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp` |
| Filter values | `nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/filter_values.hpp` |
| API docs (rolling) | https://api.nav2.org/nav2-rolling/html/classnav2__costmap__2d_1_1CostmapFilter.html |
