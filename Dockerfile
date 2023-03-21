ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Define working directory
ARG WS_DIR=/root/ws
ENV WS_DIR=${WS_DIR}
ENV WS_SRC_DIR=${WS_DIR}/src
ENV WS_INSTALL_DIR=${WS_DIR}/install
ENV WS_LOG_DIR=${WS_DIR}/log
WORKDIR ${WS_DIR}

### Install Gazebo
ARG IGNITION_VERSION=fortress
ENV IGNITION_VERSION=${IGNITION_VERSION}
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    ignition-${IGNITION_VERSION} && \
    rm -rf /var/lib/apt/lists/*

### Install extra dependencies
RUN apt-get update && \
    apt-get install -yq python3-pil.imagetk && \
    apt-get install -yq ros-${ROS_DISTRO}-pilz-industrial-motion-planner && \
    apt-get install -yq tmux && \
    apt-get install -yq python3-pip && \
    apt-get install -yq ros-${ROS_DISTRO}-desktop && \
    apt-get install -yq ros-${ROS_DISTRO}-rclcpp-components

### Install AxiDraw
#RUN apt-get update && \
#    apt-get install -yq python3-pip && \
#    pip install --upgrade --upgrade-strategy eager packaging && \
#    pip install https://cdn.evilmadscientist.com/dl/ad/public/AxiDraw_API.zip --upgrade --upgrade-strategy eager

### Install splipy
#RUN apt-get update && \
#    apt-get install -yq python3-pip && \
#    pip install --upgrade --upgrade-strategy eager splipy

# Build interfaces and generic controller first
COPY ./src/robot_interfaces ${WS_SRC_DIR}/robot_interfaces
COPY ./src/robot_controller ${WS_SRC_DIR}/robot_controller
RUN apt-get update
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" --paths ${WS_SRC_DIR}/robot_interfaces ${WS_SRC_DIR}/robot_controller && \
    rm -rf ${WS_LOG_DIR}

# Build packages
COPY ./src/draw_svg ${WS_SRC_DIR}/draw_svg
COPY ./src/drawing_controller ${WS_SRC_DIR}/drawing_controller
COPY ./src/axidraw_controller ${WS_SRC_DIR}/axidraw_controller
COPY ./src/virtual_drawing_surface ${WS_SRC_DIR}/virtual_drawing_surface
RUN pip install -r ${WS_SRC_DIR}/drawing_controller/requirements.txt
RUN pip install -r ${WS_SRC_DIR}/axidraw_controller/requirements.txt
RUN pip install -r ${WS_SRC_DIR}/virtual_drawing_surface/requirements.txt
RUN apt-get update
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source "${WS_INSTALL_DIR}/local_setup.bash" && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" --paths ${WS_SRC_DIR}/draw_svg ${WS_SRC_DIR}/drawing_controller ${WS_SRC_DIR}/axidraw_controller ${WS_SRC_DIR}/virtual_drawing_surface && \
    rm -rf ${WS_LOG_DIR}

# Build lite6 and xarm packages
COPY ./src/lite6_controller ${WS_SRC_DIR}/lite6_controller
RUN apt-get update
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    vcs import --recursive --shallow ${WS_SRC_DIR} < ${WS_SRC_DIR}/lite6_controller/lite6_controller.repos && \
    mv ${WS_SRC_DIR}/xarm_ros2/xarm* ${WS_SRC_DIR} && \
    rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths ${WS_SRC_DIR}/xarm_* && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" --paths ${WS_SRC_DIR}/xarm_* ${WS_SRC_DIR}/lite6_controller && \
    rm -rf ${WS_LOG_DIR}

# Copy example svg images
COPY ./svg svg

### Add workspace to the ROS entrypoint
### Source ROS workspace inside `~/.bashrc` to enable autocompletion
RUN sed -i '$i source "${WS_INSTALL_DIR}/local_setup.bash" --' /ros_entrypoint.sh && \
    sed -i '$a source "/opt/ros/${ROS_DISTRO}/setup.bash"' ~/.bashrc
