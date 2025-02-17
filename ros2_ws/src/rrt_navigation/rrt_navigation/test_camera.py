import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import tf_transformations
import math
import time

class GlobalLidarPointPublisher(Node):
    def _init_(self):
        super()._init_('global_lidar_point_publisher')

        # Suscribirse al LiDAR y la Odometría
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publicador de puntos detectados
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_points', 10)

        # Buffer y listener para transformación de coordenadas
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables de estado
        self.robot_position = (0.0, 0.0)
        self.robot_yaw = 0.0
        self.detected_points = []  # Lista de puntos detectados globales
        self.f = open("demofile2.txt", "a")

        self.get_logger().info("📡 Nodo de Publicación de Puntos Globales Iniciado")

    def odom_callback(self, msg):
        """Actualizar la posición del robot usando la odometría."""
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Obtener orientación del robot
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def lidar_callback(self, msg):
        """Procesar los datos del LiDAR y transformar los puntos a coordenadas globales."""
        try:
            # Obtener transformación de "base_link" a "map"
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

            # Extraer transformación
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Convertir el cuaternión a ángulos de Euler
            _, _, yaw = tf_transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

            # Procesar datos del LiDAR
            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            new_points = []

            for i, distance in enumerate(msg.ranges):
                if 0.2 < distance < 3.0:  # Filtrar valores irreales o muy lejanos
                    angle = angles[i]

                    # Convertir coordenadas del LiDAR a coordenadas relativas al robot
                    x_rel = distance * np.cos(angle)
                    y_rel = distance * np.sin(angle)

                    # Transformar a coordenadas globales usando la transformación obtenida
                    x_global = trans.x + x_rel * np.cos(yaw) - y_rel * np.sin(yaw)
                    y_global = trans.y + x_rel * np.sin(yaw) + y_rel * np.cos(yaw)

                    new_points.append((x_global, y_global))
                    

            # Agregar nuevos puntos a la lista general (sin borrar los anteriores)
            self.detected_points.extend(new_points)
            self.get_logger().info(f"Puntos {self.detected_points[-1][0]}")
            #time.sleep(1)
            
            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "points"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(self.detected_points[-1][0])
            marker.pose.position.y = float(self.detected_points[-1][1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = float(self.robot_yaw)
            self.get_logger().info(f"pasa2")

            # Tamaño del marcador
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Color (azul)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.lifetime.sec = 0  # No desaparece
            marker_array.markers.append(marker)
            
            self.get_logger().info(f"marcador: {str(marker)}")

            self.marker_pub.publish(marker_array)
            # Publicar puntos en RViz
            #self.publish_markers()
            self.get_logger().info(f"pasa8")
            self.f.write(str([float(self.detected_points[-1][0]),float(self.detected_points[-1][1])]))
            self.f.write("\n")

        except Exception as e:
            self.get_logger().warn(f"⚠ No se pudo transformar coordenadas: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = GlobalLidarPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        self.f.close()
        node.destroy_node()
        rclpy.shutdown()

if _name_ == '_main_':
    main()