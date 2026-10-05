#!/usr/bin/env bash
# 录制全局优化里程计到 rosbag, 供 evo 评估使用.
# 用法: 在已经 roslaunch 起来后, 另开终端运行本脚本, 再去播放数据包; 数据包播完后按 Ctrl-C 结束录制.
#   ./record_odom.sh <输出bag路径> [里程计话题]
# 例:
#   ./record_odom.sh ~/m3dgr_eval/ours/Outdoor01.bag
#   ./record_odom.sh ~/m3dgr_eval/base/Outdoor01.bag /lvi_sam/lidar/mapping/odometry   # 原版LVI-SAM
set -euo pipefail

OUT_BAG="${1:?用法: ./record_odom.sh <输出bag路径> [里程计话题]}"
TOPIC="${2:-/lvi_sam_ikdtree/lidar/mapping/odometry}"

mkdir -p "$(dirname "$OUT_BAG")"
echo "[record_odom] 录制话题: $TOPIC"
echo "[record_odom] 输出文件: $OUT_BAG"
echo "[record_odom] 数据包播完后, 在本终端按 Ctrl-C 停止录制."
exec rosbag record -O "$OUT_BAG" "$TOPIC"
