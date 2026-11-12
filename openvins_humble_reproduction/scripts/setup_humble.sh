#!/usr/bin/env bash
# Build the authors' implementation at a fixed, inspected commit.
set -eo pipefail
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble on Ubuntu 22.04 first.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
set -u
workspace="${1:-$HOME/openvins_humble_ws}"
revision=69488123ed9362dd44b6f28e7f4680abbff1442b
sudo apt-get update
sudo apt-get install -y git build-essential cmake libeigen3-dev \
  libboost-all-dev libceres-dev libopencv-dev libopencv-contrib-dev \
  python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-matplotlib ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
mkdir -p "$workspace/src"
repo="$workspace/src/open_vins"
if [[ -e "$repo" ]]; then
  [[ -d "$repo/.git" ]] || { echo "Not a Git checkout: $repo" >&2; exit 1; }
  [[ "$(git -C "$repo" rev-parse HEAD)" == "$revision" ]] || {
    echo 'Existing checkout uses another revision; choose a fresh workspace.' >&2; exit 1;
  }
  [[ -z "$(git -C "$repo" status --porcelain)" ]] || {
    echo 'Existing checkout has changes; choose a fresh workspace.' >&2; exit 1;
  }
else
  git clone https://github.com/rpng/open_vins.git "$repo"
  git -C "$repo" checkout --detach "$revision"
fi
cd "$workspace"
# Build the estimator and its dependencies; plotting is handled by Python.
colcon build --packages-up-to ov_msckf --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
printf '\nBuild finished. In your terminal run:\nsource %q/install/setup.bash\n' "$workspace"
