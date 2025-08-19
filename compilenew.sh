#!/usr/bin/env bash
# Humanoid build helper (nounset-safe ROS sourcing)

set -euo pipefail

# --- helpers ---
_safe_source() {
  # Temporarily disable nounset while sourcing files that may reference unset vars
  local _had_u=0
  if [[ -o nounset ]]; then _had_u=1; set +u; fi
  # shellcheck disable=SC1090
  source "$1"
  (( _had_u )) && set -u
}

_detect_ros_distro() {
  if [ -z "${ROS_DISTRO:-}" ] && [ -r /etc/os-release ]; then
    . /etc/os-release
    case "${VERSION_CODENAME:-}" in
      focal) export ROS_DISTRO=foxy ;;
      jammy) export ROS_DISTRO=humble ;;
      *) echo "Unsupported/unknown Ubuntu codename: ${VERSION_CODENAME:-?}"; exit 1 ;;
    esac
  fi
}

_source_ros() {
  _detect_ros_distro
  local ros_setup="/opt/ros/$ROS_DISTRO/setup.bash"
  if [ -f "$ros_setup" ]; then
    # Predefine to satisfy nounset inside ROS setup scripts
    export AMENT_TRACE_SETUP_FILES=""
    _safe_source "$ros_setup"
  else
    echo "ROS setup not found at $ros_setup. Did you install ROS $ROS_DISTRO?"
    exit 1
  fi
}

_source_ws_if_present() {
  local ws_setup_bash="$HUMANOID_WS/install/setup.bash"
  local ws_setup_sh="$HUMANOID_WS/install/setup.sh"
  if [ -f "$ws_setup_bash" ]; then
    _safe_source "$ws_setup_bash"
  elif [ -f "$ws_setup_sh" ]; then
    _safe_source "$ws_setup_sh"
  else
    echo "Note: workspace setup not found (expected at $ws_setup_bash or $ws_setup_sh)."
  fi
}

# --- paths ---
HUMANOID_WS="$HOME/humanoid_ws"
LIVOX_DIR="$HOME/humanoid_ws/src/livox_ros_driver2"

# --- main ---
echo ">> Sourcing ROS environment..."
_source_ros

echo ">> Sourcing humanoid_ws (if already built)..."
_source_ws_if_present

# Build Livox ROS driver (humble) if present
if [ -d "$LIVOX_DIR" ]; then
  echo ">> Building Livox ROS driver in: $LIVOX_DIR"
  cd "$LIVOX_DIR"
  chmod +x ./build.sh || true
  ./build.sh humble
else
  echo "Warning: Livox driver directory not found at $LIVOX_DIR"
fi

echo ">> Done."
