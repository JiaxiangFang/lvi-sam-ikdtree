#!/usr/bin/env bash
# 用 evo 对录制的里程计 bag 计算 APE / RPE, 并与 GT(TUM) 对比, 输出数值与图.
# 用法:
#   ./eval_traj.sh <里程计bag> <GT.txt> <输出目录> [里程计话题]
# 例:
#   ./eval_traj.sh ~/m3dgr_eval/ours/Outdoor01.bag \
#       ~/study/LVI-SAM-IKD/bags/M3DGR/GT/Outdoor01.txt \
#       ~/m3dgr_eval/ours/Outdoor01
#
# 说明:
#   - LiDAR 里程计为公制尺度, 用 -a (SE3 对齐), 不加尺度对齐 -s.
#   - GT 为 TUM 格式: timestamp tx ty tz qx qy qz qw.
set -euo pipefail

ODOM_BAG="${1:?用法: ./eval_traj.sh <里程计bag> <GT.txt> <输出目录> [里程计话题]}"
GT="${2:?缺少 GT.txt}"
OUT_DIR="${3:?缺少 输出目录}"
TOPIC="${4:-/lvi_sam_ikdtree/lidar/mapping/odometry}"

mkdir -p "$OUT_DIR"
EST_TUM="$OUT_DIR/est.tum"

echo "[eval] 1) bag -> TUM"
# evo_traj 会按话题名生成文件, 这里指定输出目录后重命名为 est.tum
tmpd="$(mktemp -d)"
( cd "$tmpd" && evo_traj bag "$ODOM_BAG" "$TOPIC" --save_as_tum >/dev/null )
mv "$tmpd"/*.tum "$EST_TUM"
rm -rf "$tmpd"

echo "[eval] 2) APE (绝对轨迹误差, SE3 对齐)"
evo_ape tum "$GT" "$EST_TUM" -a \
    --save_results "$OUT_DIR/ape.zip" \
    --plot_mode xy --save_plot "$OUT_DIR/ape_plot.pdf" 2>/dev/null || \
evo_ape tum "$GT" "$EST_TUM" -a --save_results "$OUT_DIR/ape.zip"

echo "[eval] 3) RPE (相对位姿误差, 每 100m, 仅平移)"
evo_rpe tum "$GT" "$EST_TUM" -a \
    --delta 100 --delta_unit m --pose_relation trans_part \
    --save_results "$OUT_DIR/rpe_trans.zip" 2>/dev/null || true

echo "[eval] 4) RPE (每 100m, 仅旋转)"
evo_rpe tum "$GT" "$EST_TUM" -a \
    --delta 100 --delta_unit m --pose_relation angle_deg \
    --save_results "$OUT_DIR/rpe_rot.zip" 2>/dev/null || true

echo "[eval] 完成. 结果在: $OUT_DIR"
echo "       - est.tum / ape.zip / rpe_trans.zip / rpe_rot.zip / ape_plot.pdf"
echo "       多次/多配置汇总对比: evo_res $OUT_DIR/../*/ape.zip --use_filenames -p --save_table summary.csv"
