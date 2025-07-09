cmake_minimum_required(VERSION 3.8)
project(drone_tracking_cpp)

# Habilita C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Encontrar dependências C++ e Python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # Para instalar os scripts Python
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

# Executável C++ (tracking_node)
add_executable(tracking_node src/tracking_node.cpp)

ament_target_dependencies(tracking_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

include_directories(${OpenCV_INCLUDE_DIRS})
target_include_directories(tracking_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)



install(TARGETS
  tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Finaliza o pacote
ament_package()
