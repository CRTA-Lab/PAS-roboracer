# One week's exercise: the class ROS 2 base + the packages in labs/<LAB>/.
#
#   docker build -f docker/lab.Dockerfile --build-arg LAB=lab1_week5 \
#       -t ghcr.io/<owner>/f1tenth-lab:lab1_week5 .
#
# The base layers are the same ones the f1tenth-sim image is built on, so
# students already have them: a new lab downloads only the layers below.

ARG BASE_IMAGE=ghcr.io/crta-lab/f1tenth-base:jazzy
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
ARG LAB
ENV LAB=${LAB}

# The complete example. The entrypoint copies it to /lab_ws/src on first start,
# so it lands in the student's mounted folder and survives `docker compose down`.
COPY labs/${LAB} /opt/lab_ws/src

# Lab-specific dependencies (normally none: the base carries the semester's).
# Build once here so a broken example fails CI, not the student's first start.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep install -i --from-paths /opt/lab_ws/src --rosdistro ${ROS_DISTRO} -y && \
    rm -rf /var/lib/apt/lists/* && \
    cd /tmp && colcon build --base-paths /opt/lab_ws/src && \
    rm -rf /tmp/build /tmp/install /tmp/log

COPY docker/lab_entrypoint.sh /entrypoint.sh

# `docker compose exec` shells skip the entrypoint, so source in .bashrc too.
# `cb` rebuilds the lab workspace and re-sources it.
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo '[ -f /lab_ws/install/setup.bash ] && source /lab_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'cb() { (cd /lab_ws && colcon build --symlink-install "$@"); chown -R "$(stat -c %u:%g /lab_ws)" /lab_ws; source /lab_ws/install/setup.bash; }' >> /root/.bashrc

WORKDIR /lab_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
