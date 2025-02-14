import rclpy
import numpy as np
import open3d as o3d
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class Open3DObstacleDetector(Node):
    def __init__(self):
        super().__init__('open3d_obstacle_detector')

        # Suscribirse al tópico de la cámara
        self.subscription = self.create_subscription(
            PointCloud2,
            '/intel_realsense_r200_depth/points',
            self.point_cloud_callback,
            10)

        self.get_logger().info("📡 Nodo Open3D Inicializado")

    def point_cloud_callback(self, msg):
        """Procesa la nube de puntos con Open3D y la muestra en 3D."""
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            self.get_logger().warn("⚠️ Nube de puntos vacía")
            return

        # Convertir a matriz NumPy
        cloud_array = np.array(points, dtype=np.float32)

        # Crear nube de puntos Open3D
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(cloud_array)

        # Filtrar puntos muy lejanos (eliminar paredes)
        cloud = cloud.select_by_index(np.where(cloud_array[:, 2] < 1.5)[0])

        # Visualizar la nube de puntos en Open3D
        o3d.visualization.draw_geometries([cloud], window_name="Open3D Viewer")

def main(args=None):
    rclpy.init(args=args)
    node = Open3DObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

