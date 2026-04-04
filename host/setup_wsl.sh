#!/usr/bin/env bash
# Phase 0: one-time WSL (Ubuntu 24.04) toolchain setup for the ROS2 layer.
#
# Installs:
#   - qemu-system-arm, arm-none-eabi-gcc  (device-side firmware build + run)
#   - ROS2 Jazzy desktop                  (host-side ROS2 graph, `ros2` CLI)
#   - eProsima Micro-XRCE-DDS-Agent       (built from source; this is the
#     real, unmodified agent binary Option A interop is verified against)
#
# Requires sudo, run interactively (asks for your password) -- not something
# an agent can do non-interactively. Run it yourself:
#   bash host/setup_wsl.sh
set -euo pipefail

echo "== base build tooling =="
sudo apt update
sudo apt install -y software-properties-common curl build-essential cmake git python3-pip

echo "== QEMU + ARM cross toolchain =="
sudo apt install -y qemu-system-arm gcc-arm-none-eabi

echo "== ROS2 Jazzy apt repo =="
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "== ROS2 Jazzy desktop (large download) =="
sudo apt install -y ros-jazzy-desktop

echo "== eProsima Micro-XRCE-DDS-Agent, built from source =="
AGENT_SRC="$HOME/Micro-XRCE-DDS-Agent"
if [ ! -d "$AGENT_SRC" ]; then
    git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_SRC"
fi
mkdir -p "$AGENT_SRC/build"
cd "$AGENT_SRC/build"
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig /usr/local/lib/

echo
echo "Done. Sanity checks:"
echo "  qemu-system-arm --version"
echo "  arm-none-eabi-gcc --version"
echo "  source /opt/ros/jazzy/setup.bash && ros2 doctor"
echo "  MicroXRCEAgent --help"
