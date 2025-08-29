FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
SHELL ["/bin/bash", "-lc"]

# 0) Base OS deps on top of ROS Jazzy base image
RUN printf 'Acquire::Retries "5";\nAcquire::http::Pipeline-Depth "0";\nAcquire::http::No-Cache "true";\nAcquire::https::Verify-Peer "true";\n' > /etc/apt/apt.conf.d/99-retries \
    && apt-get update -o Acquire::ForceIPv4=true \
    && apt-get install -y --no-install-recommends \
       build-essential cmake ninja-build pkg-config \
       python3-pip python3-setuptools python3-venv \
       libeigen3-dev libpoco-dev libgtest-dev libfmt-dev libspdlog-dev \
       libdw-dev libboost-dev libcap-dev \
       rt-tests libcap2-bin util-linux \
       python3-filelock python3-coverage python3-jinja2 python3-typeguard python3-pygraphviz \
       python3-rospkg-modules python3-ntplib libcurl4-openssl-dev lm-sensors \
    && rm -rf /var/lib/apt/lists/*

# Build and install GTest (provides GTest::GTest / GTest::Main)
RUN if [ -d /usr/src/googletest ]; then SRC=/usr/src/googletest; else SRC=/usr/src/gtest; fi \
    && cmake -S "$SRC" -B "$SRC/build" \
    && cmake --build "$SRC/build" --target install -j"$(nproc)"

# 1) Locale env and rosdep (install locales package first)
RUN apt-get update -o Acquire::ForceIPv4=true \
    && apt-get install -y --no-install-recommends locales python3-rosdep \
    && echo "en_US.UTF-8 UTF-8" > /etc/locale.gen \
    && locale-gen \
    && update-locale LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# 1b) Developer CLI tools via apt (avoid PEP 668 restrictions)
RUN apt-get update -o Acquire::ForceIPv4=true \
    && apt-get install -y --no-install-recommends python3-vcstool python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2) Drake (precompiled tarball). To pin 1.44, pass DRAKE_TARBALL_URL.
ARG DRAKE_TARBALL_URL=
RUN set -e; \
    if [ -z "$DRAKE_TARBALL_URL" ]; then \
      UBUNTU_CODENAME=$(lsb_release -cs); \
      BASE_URL=https://drake-packages.csail.mit.edu/drake; \
      case "$UBUNTU_CODENAME" in \
        jammy)  DRAKE_URL="$BASE_URL/nightly/drake-latest-jammy.tar.gz" ;; \
        noble)  DRAKE_URL="$BASE_URL/nightly/drake-latest-noble.tar.gz" ;; \
        *) echo "Unsupported Ubuntu codename: $UBUNTU_CODENAME"; exit 1 ;; \
      esac; \
    else \
      DRAKE_URL="$DRAKE_TARBALL_URL"; \
    fi; \
    curl -fsSL -o drake.tar.gz "$DRAKE_URL" \
    && mkdir -p /opt/drake \
    && tar -xzf drake.tar.gz -C /opt/drake --strip-components=1 \
    && rm -f drake.tar.gz

# 3) Prepare workspace
WORKDIR /opt/ws

# Clone sources from GitHub (pins configurable via build args)
# Try the requested branch first; if it doesn't exist, fall back to the repo default branch
ARG LIBFRANKA_REF=panda
ARG FRANKA_DRAKE_REF=main
ARG FRANKA_DESCRIPTION_REF=panda
ARG FRANKA_ROS2_REF=panda
RUN set -e; \
    clone_branch_or_default() { \
      local url="$1"; local branch="$2"; local dest="$3"; \
      if git ls-remote --heads "$url" "$branch" | grep -q "$branch"; then \
        git clone --depth 1 --branch "$branch" "$url" "$dest"; \
      else \
        echo "Branch '$branch' not found on $url; cloning default branch"; \
        git clone --depth 1 "$url" "$dest"; \
      fi; \
    }; \
    clone_branch_or_default https://github.com/KhachDavid/libfranka.git "$LIBFRANKA_REF" /opt/src/libfranka; \
    clone_branch_or_default https://github.com/KhachDavid/franka_drake.git "$FRANKA_DRAKE_REF" /opt/src/franka_drake; \
    clone_branch_or_default https://github.com/KhachDavid/franka_description.git "$FRANKA_DESCRIPTION_REF" /opt/src/franka_description; \
    clone_branch_or_default https://github.com/KhachDavid/franka_ros2.git "$FRANKA_ROS2_REF" /opt/src/franka_ros2

# 4) Build and install libfranka (Release) to /opt/libfranka
WORKDIR /opt/src/libfranka
RUN mkdir -p build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/libfranka \
             -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=ON -DBUILD_DOCUMENTATION=OFF \
             -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. \
    && cmake --build . -j $(nproc) \
    && cmake --install .

# 5) Optionally build franka_drake against Drake tarball and libfranka
ARG BUILD_FRANKA_DRAKE=ON
WORKDIR /opt/src/franka_drake
RUN if [ "$BUILD_FRANKA_DRAKE" = "ON" ]; then \
      mkdir -p build && cd build \
      && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
               -DCMAKE_PREFIX_PATH="/opt/drake;/opt/libfranka" .. \
      && cmake --build . -j $(nproc); \
    else \
      echo "Skipping franka_drake build (BUILD_FRANKA_DRAKE=${BUILD_FRANKA_DRAKE})"; \
    fi

# Ensure colcon ignores franka_drake even if present/symlinked into the workspace
RUN touch /opt/src/franka_drake/COLCON_IGNORE || true

# 6) ROS deps via rosdep, then build franka_ros2 and franka_description (Jazzy)
RUN rosdep init || true \
    && rosdep update \
    && mkdir -p /opt/ws/src \
    && ln -s /opt/src/franka_ros2 /opt/ws/src/franka_ros2 \
    && ln -s /opt/src/franka_description /opt/ws/src/franka_description

RUN cat > /opt/ws/src/overlay.repos <<'EOF'
repositories:
  control_msgs:
    type: git
    url: https://github.com/ros-controls/control_msgs.git
  ros2_control:
    type: git
    url: https://github.com/ros-controls/ros2_control.git
  ros2_controllers:
    type: git
    url: https://github.com/ros-controls/ros2_controllers.git
  realtime_tools:
    type: git
    url: https://github.com/ros-controls/realtime_tools.git
  generate_parameter_library:
    type: git
    url: https://github.com/PickNikRobotics/generate_parameter_library.git
  backward_ros:
    type: git
    url: https://github.com/pal-robotics/backward_ros.git
  angles:
    type: git
    url: https://github.com/ros/angles.git
  control_toolbox:
    type: git
    url: https://github.com/ros-controls/control_toolbox.git
  diagnostics:
    type: git
    url: https://github.com/ros/diagnostics.git
EOF

RUN vcs import /opt/ws/src < /opt/ws/src/overlay.repos \
    && source /opt/ros/jazzy/setup.bash \
    && apt-get update -o Acquire::ForceIPv4=true \
    && rosdep install --rosdistro jazzy --ignore-src -r -y \
         --from-paths \
           /opt/ws/src/control_msgs \
           /opt/ws/src/ros2_control \
           /opt/ws/src/ros2_controllers \
           /opt/ws/src/realtime_tools \
           /opt/ws/src/generate_parameter_library \
           /opt/ws/src/backward_ros \
           /opt/ws/src/angles \
           /opt/ws/src/control_toolbox \
           /opt/ws/src/diagnostics \
           /opt/ws/src/franka_description \
           /opt/ws/src/franka_ros2/franka_msgs \
           /opt/ws/src/franka_ros2/franka_semantic_components \
           /opt/ws/src/franka_ros2/franka_hardware \
           /opt/ws/src/franka_ros2/franka_robot_state_broadcaster \
           /opt/ws/src/franka_ros2/franka_example_controllers \
           /opt/ws/src/franka_ros2/franka_gripper \
         --skip-keys "libfranka python3-rospkg ament-cmake-clang-format ament-cmake-clang-tidy" \
    && if [ -d /opt/ws/src/franka_drake ]; then touch /opt/ws/src/franka_drake/COLCON_IGNORE; fi \
    && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCHECK_TIDY=OFF \
         --packages-skip franka_drake franka_gazebo franka_fer_moveit_config franka_bringup

# 7) Environment setup for interactive shells and runtime
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
    && echo 'source /opt/ws/install/setup.bash' >> /etc/bash.bashrc

# Runtime env so Drake and libfranka work without manual setup
ENV DRAKE_INSTALL_DIR=/opt/drake
ENV LD_LIBRARY_PATH=/opt/libfranka/lib:/opt/drake/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/drake/bin:$PATH

# Default port configuration for franka_drake (can be overridden at runtime)
ENV FRANKA_TCP_PORT=11337
ENV FRANKA_UDP_PORT=11340
ENV MESHCAT_PORT=17000
ENV FRANKA_GRIPPER_PORT=11338
ENV FRANKA_TELEMETRY_PORT=15511

# Provide a simple entrypoint to start the FCI sim server immediately
RUN cat > /usr/local/bin/start-franka-sim <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

# Ensure runtime env for libraries (already set via Dockerfile ENV, but keep for safety)
export DRAKE_INSTALL_DIR="${DRAKE_INSTALL_DIR:-/opt/drake}"
export LD_LIBRARY_PATH="/opt/libfranka/lib:${DRAKE_INSTALL_DIR}/lib:${LD_LIBRARY_PATH:-}"
export PATH="${DRAKE_INSTALL_DIR}/bin:${PATH}"

# Port configuration (can be overridden at docker run)
export FRANKA_TCP_PORT="${FRANKA_TCP_PORT:-11337}"
export FRANKA_UDP_PORT="${FRANKA_UDP_PORT:-11340}"
export MESHCAT_PORT="${MESHCAT_PORT:-17000}"
export FRANKA_GRIPPER_PORT="${FRANKA_GRIPPER_PORT:-11338}"
export FRANKA_TELEMETRY_PORT="${FRANKA_TELEMETRY_PORT:-15511}"

echo "Starting franka_drake with ports:"
echo "  FCI TCP: $FRANKA_TCP_PORT"
echo "  FCI UDP: $FRANKA_UDP_PORT"
echo "  Meshcat: $MESHCAT_PORT"
echo "  Gripper: $FRANKA_GRIPPER_PORT"
echo "  Telemetry: $FRANKA_TELEMETRY_PORT"

# Realtime knobs (can be overridden at `docker run`):
#  - ENABLE_RT=1 to attempt SCHED_FIFO
#  - RT_PRIORITY=90 desired FIFO priority (1-99)
#  - CPU_PIN="0" to pin to CPU core(s) via taskset
ENABLE_RT="${ENABLE_RT:-1}"
RT_PRIORITY="${RT_PRIORITY:-90}"
CPU_PIN="${CPU_PIN:-}"

BIN="/opt/src/franka_drake/build/bin/franka-fci-sim-embed-example"
if [[ ! -x "$BIN" ]]; then
  echo "Binary not found: $BIN" >&2
  exit 1
fi

# Try to raise limits if the container was started with appropriate flags
# These will succeed only if run with e.g.: --cap-add=sys_nice --ulimit rtprio=99 --ulimit memlock=-1
ulimit -l unlimited || true
ulimit -r 99 || true

# Compose the command with optional CPU pinning
if [[ -n "$CPU_PIN" ]]; then
  CMD=(taskset -c "$CPU_PIN" "$BIN")
else
  CMD=("$BIN")
fi

if [[ "$ENABLE_RT" == "1" ]]; then
  if command -v chrt >/dev/null 2>&1; then
    if chrt -f "$RT_PRIORITY" true 2>/dev/null; then
      echo "Starting with SCHED_FIFO priority $RT_PRIORITY"
      exec chrt -f "$RT_PRIORITY" "${CMD[@]}"
    else
      echo "Warning: Unable to set SCHED_FIFO. Start container with: --cap-add=sys_nice --ulimit rtprio=99 --ulimit memlock=-1" >&2
      echo "Proceeding without realtime scheduling." >&2
      exec "${CMD[@]}"
    fi
  else
    echo "Warning: 'chrt' not available; install util-linux. Proceeding without realtime." >&2
    exec "${CMD[@]}"
  fi
else
  exec "${CMD[@]}"
fi
EOF
RUN chmod +x /usr/local/bin/start-franka-sim

WORKDIR /opt/ws
CMD ["/usr/local/bin/start-franka-sim"]


