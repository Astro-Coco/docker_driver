#!/usr/bin/env bash
set -euo pipefail

# Repo URLs and where to clone them
declare -A repos=(
  [ws_livox/src/livox_ros_driver2]="https://github.com/Livox-SDK/livox_ros_driver2.git"
  [Livox-SDK2]="https://github.com/Livox-SDK/Livox-SDK2.git"
  [ws_livox/src/rviz_2d_overlay_plugins]="https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins.git"
  [ws_livox/src/SuperOdom]="https://github.com/superxslam/SuperOdom.git"
)

for target in "${!repos[@]}"; do
  url=${repos[$target]}
  if [ -d "$target" ]; then
    echo "✔ $target already exists; skipping."
  else
    echo "⏳ Cloning $url → $target"
    mkdir -p "$(dirname "$target")"
    git clone "$url" "$target"
  fi
done

TARGET_DIR="ws_livox/src/livox_ros_driver2"
OLD_FILE="$TARGET_DIR/package_ROS2.xml"
NEW_FILE="$TARGET_DIR/package.xml"

if [ -f "$OLD_FILE" ]; then
  echo "Renaming $OLD_FILE → $NEW_FILE"
  mv "$OLD_FILE" "$NEW_FILE"
  echo "Done."
else
  echo "⚠️  File not found: $OLD_FILE" >&2
  exit 1
fi
