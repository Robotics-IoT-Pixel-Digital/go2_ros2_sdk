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
        """Load stair zone polygons from YAML."""
        if not filepath:
            self.get_logger().warn('No stair zones file specified')
            return []
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


if __name__ == '__main__':
    main()
