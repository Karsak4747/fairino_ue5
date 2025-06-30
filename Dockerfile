FROM osrf/ros:humble-desktop-full

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    nano git apt-utils gnupg \
    build-essential \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-moveit \
    ros-humble-moveit-common \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-move-group \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxrender1 \
    x11-apps

# Копируем основной пакет
COPY frcobot_ros2_main /root/ros2_ws/src/frcobot_ros2_main

# Клонируем warehouse_ros_mongo с исправлением
WORKDIR /root/ros2_ws/src
RUN git clone -b ros2 https://github.com/ros-planning/warehouse_ros_mongo.git && \
    # Применяем исправление для проблемы символических ссылок
    sed -i 's|ament_cmake_python_symlink_warehouse_ros_mongo|# &|' warehouse_ros_mongo/CMakeLists.txt

# Копируем UE connector
COPY ros2_ue_connector_node /root/ros2_ws/src/ros2_ue_connector_node

# Установка зависимостей ОС
WORKDIR /root/ros2_ws
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    pip install pyyaml

# Сборка ВСЕХ пакетов за один шаг
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --parallel-workers 4"

# Настройка окружения
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/ros2_ws/src/frcobot_ros2_main" >> ~/.bashrc

CMD ["/bin/bash"]