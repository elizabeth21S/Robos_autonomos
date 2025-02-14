cmake_minimum_required(VERSION 3.8)
project(rrt_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Instalar archivos Python
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Finalizar instalación
ament_package()
