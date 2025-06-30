cmake_minimum_required(VERSION 3.8)
project(ros2_ue_connector_node)

# Установка стандарта C++17 (обязательно для ROS2 Humble)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Опции компилятора
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Добавьте эту строку

# Создание исполняемого файла
add_executable(ue_listener src/ros2_ue_connector.cpp)
ament_target_dependencies(ue_listener rclcpp)

# Установка узла
install(TARGETS ue_listener
        DESTINATION lib/${PROJECT_NAME})

# Тестирование (опционально)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Завершение конфигурации пакета
ament_package()
