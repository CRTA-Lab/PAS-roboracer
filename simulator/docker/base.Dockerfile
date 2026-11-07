# Layer 1: ROS 2 Jazzy base for the class.
#
# Everything installed with apt for the whole semester lives here, so the
# simulator layer on top (and the weekly labs) never need to touch apt.
# Built with ./docker as the context (see docker-compose.yml).

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Tooling + RViz (Qt/OpenGL runtime, xvfb for headless rendering)
# + every ROS package the simulator and the semester's labs depend on.
# Add a lab's apt dependency here rather than in the lab itself.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git nano vim tmux \
        python3-pip python3-dev libeigen3-dev python3-scipy \
        libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 libxcb-shape0 libxcb-randr0 \
        libxcb-render-util0 libxcb-xinerama0 libxkbcommon-x11-0 \
        libgl1 libegl1 libopengl0 libgl1-mesa-dri xvfb \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-ackermann-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-nav2-lifecycle-manager \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-tf-transformations \
        python3-transforms3d && \
    rm -rf /var/lib/apt/lists/*

# RViz map shader fix for Mesa >= 24: give the shader's two samplers explicit
# texture units so Mesa's post-link validation no longer reports "active
# samplers with a different type refer to the same texture image unit".
# See rviz_fix/indexed_8bit_image.frag for details.
COPY rviz_fix/indexed_8bit_image.frag \
     /opt/ros/${ROS_DISTRO}/share/rviz_rendering/ogre_media/materials/glsl120/indexed_8bit_image.frag

# rosdep is initialised by the ros image; refresh it so students can
# `rosdep install` against an up-to-date index.
RUN rosdep update --rosdistro ${ROS_DISTRO}

CMD ["bash"]
