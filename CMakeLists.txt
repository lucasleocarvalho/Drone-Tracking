cmake_minimum_required(VERSION 3.8)
project(drone_tracking)

# Habilita C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Adiciona o executável C++
add_executable(tracking_node src/tracking_node.cpp)
ament_target_dependencies(tracking_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

# Inclui os headers do OpenCV
target_include_directories(tracking_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# Linka as libs do OpenCV
target_link_libraries(tracking_node
  ${OpenCV_LIBS}
)

# Instala o executável
install(TARGETS
  tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

# Suporte ao setup.py (Python)
ament_python_install_package(${PROJECT_NAME})

# Finaliza o pacote
ament_package()
