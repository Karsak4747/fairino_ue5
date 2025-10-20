import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class STLMarkerPublisher(Node):
    def __init__(self):
        super().__init__('stl_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)

        # Создаем маркер
        marker = Marker()
        marker.header.frame_id = "base_link"   # или другая система координат робота
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stl_marker"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "package://fairino3_v6_moveit2_config/config/robot_stand.stl"
        marker.pose.position.x = -0.86
        marker.pose.position.y = 0.31
        marker.pose.position.z = -0.8
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # публикуем каждые 1 секунду
        self.timer = self.create_timer(1.0, lambda: self.publisher_.publish(marker))


def main(args=None):
    rclpy.init(args=args)
    node = STLMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
