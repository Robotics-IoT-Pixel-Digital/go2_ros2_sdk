#include "stair_zone_filter.hpp"
#include <mutex>
#include <cmath>
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

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
    throw std::runtime_error("StairZoneFilter: unable to lock node_");
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();

  auto qos = rclcpp::QoS(1).transient_local().reliable();
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic, qos,
    std::bind(&StairZoneFilter::filterInfoCallback, this, std::placeholders::_1));
}

void StairZoneFilter::filterInfoCallback(
  nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
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
  const double ox = mask->info.origin.position.x;
  const double oy = mask->info.origin.position.y;
  const double res = mask->info.resolution;

  if (res <= 0.0) { return false; }

  const double dx = wx - ox;
  const double dy = wy - oy;
  if (dx < 0.0 || dy < 0.0) { return false; }

  mx = static_cast<unsigned int>(dx / res);
  my = static_cast<unsigned int>(dy / res);

  return mx < mask->info.width && my < mask->info.height;
}

void StairZoneFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  nav2_costmap_2d::CostmapFilter::updateBounds(
    robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  std::lock_guard<std::recursive_mutex> lock(*getMutex());
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
  const geometry_msgs::msg::Pose2D & /*pose*/)
{
  if (!filter_mask_ || !enabled_) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(*getMutex());

  unsigned char * master_array = master_grid.getCharMap();

  const std::string mask_frame = filter_mask_->header.frame_id;
  const bool same_frame = (mask_frame == global_frame_);

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
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      double msk_wx = wx, msk_wy = wy;
      if (!same_frame) {
        const auto & t = transform.transform;
        double yaw = tf2::getYaw(t.rotation);
        double cos_theta = std::cos(yaw);
        double sin_theta = std::sin(yaw);
        double tx = t.translation.x;
        double ty = t.translation.y;
        msk_wx = cos_theta * wx - sin_theta * wy + tx;
        msk_wy = sin_theta * wx + cos_theta * wy + ty;
      }

      unsigned int mx, my;
      if (!worldToMask(filter_mask_, msk_wx, msk_wy, mx, my)) {
        continue;
      }

      unsigned char mask_cost = static_cast<unsigned char>(
        filter_mask_->data[my * filter_mask_->info.width + mx]);

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
