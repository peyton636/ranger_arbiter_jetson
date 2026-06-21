#!/usr/bin/env bash

rm -rf build log install
colcon build --symlink-install --cmake-args -DUSE_CUDA=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select yolo_detector image_preprocess fusion_pose

# 合并 compile_commands.json 供 clangd 使用
python3 - <<'PYEOF'
import json
from pathlib import Path
merged = []
for path in sorted(Path("build").rglob("compile_commands.json")):
    data = json.loads(path.read_text())
    if isinstance(data, list):
        merged.extend(data)
Path("compile_commands.json").write_text(json.dumps(merged, indent=2))
PYEOF
# colcon build --symlink-install --cmake-args -DUSE_CUDA=ON --packages-select yolo_detector image_preprocess fusion_pose \
#     agx_arm_controller agx_arm_description agx_arm_moveit agx_gripper_controller agx_motion_planner


source install/setup.bash

pids=()

cleanup() {
  echo
  echo "[INFO] Stopping all launch processes..."
  for pid in "${pids[@]:-}"; do
    # 向整个进程组发 SIGINT，确保 ros2 launch 及其子进程都退出
    kill -SIGINT -- "-${pid}" 2>/dev/null || true
  done

  # 等待退出，避免僵尸进程
  for pid in "${pids[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
  echo "[INFO] All launches stopped."
}

trap cleanup EXIT INT TERM

start_launch() {
  local cmd="$1"
  echo "[INFO] Starting: ${cmd}"
  # setsid 启动独立进程组，便于统一关闭
  setsid bash -c "${cmd}" &
  pids+=("$!")
}

start_launch "ros2 launch image_preprocess preprocess.launch.py"
# start_launch "ros2 launch yolo_detector classifier.launch.py"
start_launch "ros2 launch yolo_detector detector.launch.py"
# start_launch "ros2 launch yolo_detector obb.launch.py"
# start_launch "ros2 launch yolo_detector pose.launch.py"
# start_launch "ros2 launch yolo_detector segmentor.launch.py"
start_launch "ros2 launch fusion_pose fusion_pose.launch.py"
# start_launch "ros2 launch agx_arm_controller arm_controller.launch.py"
# start_launch "ros2 launch agx_gripper_controller gripper_controller.launch.py"
# start_launch "ros2 launch agx_motion_planner agx_motion_planner_node.launch.py"

echo "[INFO] All launches are running. Press Ctrl+C to stop all."
wait