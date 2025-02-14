# Robôs autônomos
Este repositorio contiene la implementación de un **sistema de navegación autónoma** en ROS 2, utilizando `Nav2` y un robot equipado con sensores para explorar y moverse en un entorno.

## 📂 **Instalación y Configuración**
### 1️⃣ Clonar el repositorio
```bash
git clone https://github.com/elizabeth21S/Robos_autonomos.git
```
### 2️⃣Configurar el entorno de ROS 2
Asegúrate de tener ROS 2 Humble instalado y configurado correctamente. Luego, compila y configura el entorno:
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
### 3️⃣ Lanzar la simulación y la navegación

Para iniciar la simulación y el sistema de navegación:
```bash
export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models # Iron and older only with Gazebo Classic

ros2 launch rrt_navigation rrt_worldlaunch.py headless:=False world:=ruta/to/world slam:=True
```
### 🛠 Requerimientos

✅ ROS 2 Humble

✅ Nav2

✅ Gazebo / TurtleBot3

✅ RViz 2
