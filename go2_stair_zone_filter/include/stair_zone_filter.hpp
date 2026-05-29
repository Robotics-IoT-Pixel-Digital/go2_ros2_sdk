#ifndef STAIR_ZONE_FILTER_HPP_
#define STAIR_ZONE_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace go2_stair_zone_filter
{

class StairZoneFilter : public nav2_costmap_2d::CostmapFilter
{
public:
  StairZoneFilter() = default;
  ~StairZoneFilter() override = default;

  void initializeFilter(const std::string & filter_info_topic) override;

  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose) override;

  void resetFilter() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

private:
  void filterInfoCallback(nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
  void maskCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool worldToMask(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr mask,
    double wx, double wy, unsigned int & mx, unsigned int & my) const;

  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask_;
  std::string global_frame_;
  bool has_updated_data_{false};

  unsigned int mask_width_{0}, mask_height_{0};
};

}  // namespace go2_stair_zone_filter

#endif  // STAIR_ZONE_FILTER_HPP_
