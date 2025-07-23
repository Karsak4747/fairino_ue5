FROM osrf/ros:humble-desktop-full

# 1) Удаляем старый список и ключ
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list \
          /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# 2) Ставим утилиты и импортируем новый ключ
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release \
 && mkdir -p /etc/apt/keyrings \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor \
      | tee /etc/apt/keyrings/ros-archive-keyring.gpg > /dev/null

# 3) Добавляем единственный чистый репозиторий
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# Теперь можно обновлять и устанавливать нужные пакеты
RUN apt-get update && apt-get install -y \
    nano git apt-utils build-essential \
    ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-dev-tools \
    python3-colcon-common-extensions python3-pip \
    ros-humble-moveit ros-humble-moveit-common  \
    ros-humble-ros2-control ros-humble-controller-manager ros-humble-control-toolbox \
    ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
    ros-humble-moveit-ros-planning ros-humble-moveit-ros-move-group \
    ros-humble-rqt-tf-tree \
    libxcb-xinerama0 libxkbcommon-x11-0 libxcb-icccm4 libxcb-image0 \
    libxcb-keysyms1 libxcb-render-util0 libxrender1 x11-apps

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