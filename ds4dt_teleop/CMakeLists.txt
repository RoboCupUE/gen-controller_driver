cmake_minimum_required(VERSION 3.8)
project(ds4dt_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ds4dt_interface REQUIRED)

# Añadir ejecutable
add_executable(ds4dt_teleop_node src/ds4dt_teleop_node.cpp)

# Añadir directorios de inclusión
target_include_directories(ds4dt_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Añadir dependencias al ejecutable
ament_target_dependencies(ds4dt_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
  ds4dt_interface
)

# Instalar ejecutable
install(TARGETS ds4dt_teleop_node
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


ament_package()
