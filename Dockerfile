# Dockerfile для ROS 2 и плагина frcobot_ros2_main с зависимостями MoveIt
FROM osrf/ros:humble-desktop-full

# Обновляем пакеты и устанавливаем необходимые зависимости
RUN apt-get update && apt-get install -y \
    nano git apt-utils gnupg\
    build-essential \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-moveit \
    ros-humble-moveit-common \
    ros-humble-moveit-ros-planning \
 #   ros-humble-ament-cmake-gmock \
    ros-humble-moveit-ros-move-group

RUN apt-get update && apt-get install -y \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxrender1 \
    x11-apps

# Копирование и устанавливка плагина frcobot_ros2_main
COPY frcobot_ros2_main /root/ros2_ws/src/frcobot_ros2_main

WORKDIR /root/ros2_ws/src
RUN git clone -b ros2 https://github.com/ros-planning/warehouse_ros_mongo.git


# Устанавливаем зависимости, если они указаны в package.xml
WORKDIR /root/ros2_ws
RUN apt-get update && rosdep install --from-paths src --ignore-src --skip-keys=warehouse_ros_mongo -r -y && pip install pyyaml


# Сборка ROS2 пакетов и плагинов
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --parallel-workers 4"

# Настройка среды
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

#RUN /bin/bash -c "source /usr/share/gazebo/setup.sh && cmake /root/ros2_ws/src/frcobot_ros2_main/iiwa_description/CMakeLists.txt"


RUN /bin/bash -c "source /usr/share/gazebo/setup.sh && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/frcobot_ros2_main"

# Копирование файлов для конкурса
# COPY trajectories /root/trajectories
# Запуск контейнера в интерактивном режиме с шеллом
CMD ["/bin/bash"]
