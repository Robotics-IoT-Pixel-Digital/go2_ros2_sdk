# go2_stair_zone_filter

A Nav2 costmap filter plugin for the Unitree Go2 that **clears all cost in designated stair zones**, allowing the robot to navigate through stairs that would otherwise be blocked by LIDAR-detected obstacles and static map walls.

## How It Works

### The Problem

When the Go2 navigates using Nav2, the 2D LIDAR sees stair edges as obstacles. The static map also marks stairs as occupied (black). Both the obstacle layer and inflation layer flag these areas as impassable, causing the planner and controller to refuse routes through stairs.

### The Solution

`StairZoneFilter` is a custom Nav2 costmap filter plugin that runs **after all other costmap layers** (static, obstacle, inflation). Wherever a mask image marks a stair zone, the filter overwrites the costmap cell to `FREE_SPACE` (cost 0), effectively erasing any obstacle cost that was written by earlier layers.

```
Nav2 Costmap2D Update Cycle (per tick)
───────────────────────────────────────
1. static_layer    → loads map, marks walls/stairs as occupied
2. obstacle_layer  → processes /scan, marks LIDAR hits as obstacles
3. inflation_layer → inflates obstacle costs outward
4. keepout_filter  → adds cost in keepout zones (disabled by default)
5. stair_zone_filter → CLEARS cost in stair zones ← runs last, wins
```

Since `stair_zone_filter` runs last in the filter chain, it has the final say on cost values. Any cell inside a stair zone is forced to `FREE_SPACE` regardless of what the layers wrote.

### Architecture

The plugin fits into Nav2's costmap filter infrastructure and reuses the same `CostmapFilterInfo` + `MapServer` mask pipeline used by the built-in `KeepoutFilter`:

1. A `costmap_filter_info_server` node publishes metadata (topic names, type, base, multiplier) on `/stair_zone_costmap_filter_info`
2. A `map_server` node loads a PGM mask image and publishes it as an `OccupancyGrid` on `/stair_zone_filter_mask`
3. The plugin subscribes to both topics, then on each costmap tick, iterates over cells in the update region and clears any cell where the mask is "occupied" (black)

**Mask encoding (inverted from KeepoutFilter):**

| Mask pixel value | OccupancyGrid value | Costmap cost | Filter action |
|---|---|---|---|
| White (255) | 0 (free) | 0 | Do nothing |
| Black (0) | 100 (occupied) | 254 (`LETHAL_OBSTACLE`) | **Clear to `FREE_SPACE`** |

The mask reuses the same OccupancyGrid encoding as a keepout mask — the difference is semantic. KeepoutFilter *adds* cost where the mask is occupied; StairZoneFilter *clears* cost where the mask is occupied.

## Step-by-Step Usage

### Step 1: Build the Package

From the workspace root:

```bash
cd ~/go2_ros2_sdk
colcon build --packages-select go2_stair_zone_filter
source install/setup.bash
```

Verify the plugin is discoverable by Nav2:

```bash
ros2 pkg plugins nav2_costmap_2d | grep stair_zone
# Expected: go2_stair_zone_filter/StairZoneFilter (nav2_costmap_2d::Layer)
```

### Step 2: Create a Stair Zone Mask

For each map that contains stairs, create a PGM mask image and a YAML metadata file.

**2a. Copy the navigation map as a starting point:**

```bash
cp go2_robot_sdk/maps/Studio.pgm go2_robot_sdk/maps/Studio_stair_mask.pgm
```

**2b. Edit the mask in a raster editor (GIMP, Photoshop, etc.):**

- Flood-fill the entire image with **white** (255) — this is the default "no stair zone" state
- Paint **black** (0) over the stair areas — these are the zones where cost will be cleared
- Keep zones tight: only cover the actual stair tread area plus a small (~0.5m) buffer
- Do NOT extend into corridors or open floor areas
- Export as PGM (plain format, no compression)

**2c. Create the YAML metadata file** (e.g., `go2_robot_sdk/maps/Studio_stair_mask.yaml`):

```yaml
image: Studio_stair_mask.pgm
mode: trinary
resolution: 0.07           # MUST match the navigation map resolution
origin: [-11.2, -9.8, 0.0] # MUST match the navigation map origin
negate: false
occupied_thresh: 0.65
free_thresh: 0.25
```

> **Critical:** The `resolution` and `origin` must exactly match the navigation map. A mismatch will clear the wrong cells.

Current map parameters for reference:

| Map | Resolution | Origin |
|---|---|---|
| `Studio.yaml` | 0.07 | [-11.2, -9.8, 0] |
| `Studio_Pintu.yaml` | 0.07 | [-6.63, -8.05, 0] |
| `studio-1.yaml` | 0.07 | [-7.17, -9.45, 0] |
| `nav_lobby.yaml` | 0.07 | [-2.49, -4.24, 0] |
| `Lobby-1.yaml` | 0.05 | [-6.32, -8.68, 0] |
| `FullMap.yaml` | 0.07 | [-5.01, -13.2, 0] |

### Step 3: Configure Nav2 Parameters

The configuration is already wired into the project. Here's what was set up:

**`config/params_navigation.yaml`** — the filter is added to both costmaps:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      filters: ["keepout_filter", "stair_zone_filter"]
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      stair_zone_filter:
        plugin: "go2_stair_zone_filter::StairZoneFilter"
        filter_info_topic: "/stair_zone_costmap_filter_info"
        enabled: true

global_costmap:
  global_costmap:
    ros__parameters:
      filters: ["keepout_filter", "stair_zone_filter"]
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      stair_zone_filter:
        plugin: "go2_stair_zone_filter::StairZoneFilter"
        filter_info_topic: "/stair_zone_costmap_filter_info"
        enabled: true
```

**`config/params_stair_zone.yaml`** — the filter info and mask server parameters:

```yaml
stair_zone_filter_mask_server:
  ros__parameters:
    topic_name: "/stair_zone_filter_mask"

stair_zone_costmap_filter_info_server:
  ros__parameters:
    type: 0
    filter_info_topic: "/stair_zone_costmap_filter_info"
    mask_topic: "/stair_zone_filter_mask"
    base: 0.0
    multiplier: 1.0
```

### Step 4: Launch with Stair Zone Enabled

```bash
ros2 launch go2_robot_sdk cyclonedds_navigation.launch.py \
  stair_zone_mask:=true \
  stair_zone_map:=/path/to/go2_robot_sdk/maps/Studio_stair_mask.yaml \
  rviz:=true
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `stair_zone_mask` | `false` | Enable/disable the stair zone filter |
| `stair_zone_map` | `Studio_stair_mask.yaml` | Absolute path to the stair zone mask YAML |

### Step 5: Verify in RViz

Add the following displays to verify everything is working:

1. **Map display** → topic: `/map` — shows the navigation map with stairs as occupied
2. **Map display** → topic: `/stair_zone_filter_mask` — shows the stair zone overlay (black = stair zones)
3. **Map display** → topic: `/global_costmap/costmap` — the stair zone area should appear as FREE (cost = 0)

Confirm the stair zone overlay aligns precisely with the stair locations on the navigation map.

### Step 6: Test Navigation

1. Set an initial pose on one side of the stairs
2. Send a navigation goal on the other side of the stairs
3. The SMAC planner should generate a path through the stair zone
4. The DWB controller should follow the path without stopping

## ROS 2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/stair_zone_costmap_filter_info` | `nav2_msgs/CostmapFilterInfo` | Published by info server | Filter metadata (mask topic, type, base, multiplier) |
| `/stair_zone_filter_mask` | `nav_msgs/OccupancyGrid` | Published by map server | The stair zone mask grid |

## Plugin API

`StairZoneFilter` inherits from `nav2_costmap_2d::CostmapFilter` and implements three required methods:

| Method | Purpose |
|---|---|
| `initializeFilter(filter_info_topic)` | Subscribes to the `CostmapFilterInfo` topic, then to the mask `OccupancyGrid` topic |
| `process(master_grid, min_i, min_j, max_i, max_j, pose)` | Iterates update region, clears cost where mask is `LETHAL_OBSTACLE` |
| `resetFilter()` | Unsubscribes from all topics and clears stored mask |

The `updateBounds()` override expands the update region to cover the full mask on first receipt, ensuring all overlapping cells are processed.

## Safety Considerations

- **Keep stair zones narrow.** Only cover the actual stair path + small buffer. Oversized zones clear obstacle detection in areas that aren't stairs.
- **Dynamic obstacles are also cleared.** The filter removes *all* cost in stair zones — including costs from the LIDAR obstacle layer. A person standing in a stair zone will not be detected by Nav2.
- **Collision monitor.** Consider enabling Nav2's collision monitor with a tight stop polygon (~0.25m) as a last-resort hardware safety net.
- **Speed limiting.** Consider combining with Nav2's `SpeedFilter` to reduce speed (e.g., 0.2 m/s) when traversing stairs.

## File Structure

```
go2_stair_zone_filter/
├── CMakeLists.txt                    # ament_cmake build configuration
├── package.xml                       # ROS 2 package dependencies
├── stair_zone_filter_plugin.xml      # pluginlib manifest
├── include/
│   └── stair_zone_filter.hpp         # plugin class declaration
└── src/
    └── stair_zone_filter.cpp         # plugin implementation
```

Related files in `go2_robot_sdk/`:

```
go2_robot_sdk/
├── config/
│   ├── params_navigation.yaml        # costmap plugin/filter configuration
│   └── params_stair_zone.yaml        # mask server and info server parameters
├── launch/
│   └── cyclonedds_navigation.launch.py  # launch file with stair_zone_mask arg
└── maps/
    ├── Studio_stair_mask.yaml        # mask metadata (create per map)
    └── Studio_stair_mask.pgm         # mask image (create per map)
```
