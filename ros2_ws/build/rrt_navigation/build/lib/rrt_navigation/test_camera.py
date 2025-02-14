import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class SimpleCameraReader(Node):
    def __init__(self):
        super().__init__('simple_camera_reader')

        # Suscribirse a la cámara de profundidad
        self.subscription = self.create_subscription(
            PointCloud2, '/intel_realsense_r200_depth/points', self.camera_callback, 5)

        self.get_logger().info("📡 Nodo de Cámara Iniciado")

    def camera_callback(self, msg):
        """Lee la nube de puntos y muestra coordenadas en consola."""
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if not points:
            self.get_logger().warn("⚠️ Nube de puntos vacía")
            return

        # Mostrar los primeros 10 puntos
        self.get_logger().info("📡 Primeros 10 puntos detectados:")
        for i, (x, y, z) in enumerate(points[:10]):
            self.get_logger().info(f"🔸 Punto {i+1}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCameraReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

