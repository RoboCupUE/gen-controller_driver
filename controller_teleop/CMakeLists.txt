cmake_minimum_required(VERSION 3.8)
project(controller_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_interface REQUIRED)

# Añadir ejecutable
add_executable(controller_teleop_node src/controller_teleop_node.cpp)

# Añadir directorios de inclusión
target_include_directories(controller_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Añadir dependencias al ejecutable
ament_target_dependencies(controller_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
  controller_interface
)

# Instalar ejecutable
install(TARGETS controller_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar archivos de encabezado (opcional)
install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)


ament_package()
