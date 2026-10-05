#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
改进点二消融实验 —— 深度恢复层面定量分析.

读取 ON(改进二) / OFF(基线) 两个逐帧统计 CSV (由 depth_stats_csv 落盘),
从"深度恢复数量、结构来源、拒绝构成、实时性"等角度对比, 输出表格(+可选图).

CSV 列: stamp,robust,feat,success,plane,line,fallback,noNbr,sight,cluster,scatter,geomFail,cost_ms

用法:
  python3 analyze_depth_stats.py --on  ours_Outdoor01.csv \
                                 --off base_Outdoor01.csv \
                                 --name Outdoor01 \
                                 [--plot out_dir]
"""
import argparse
import csv
import math
import os


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except ValueError:
                continue
    return rows


def col(rows, key):
    return [r[key] for r in rows if key in r]


def stats(vals):
    if not vals:
        return (0.0, 0.0, 0.0, 0.0)
    n = len(vals)
    m = sum(vals) / n
    var = sum((x - m) ** 2 for x in vals) / n
    sd = math.sqrt(var)
    s = sorted(vals)
    med = s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])
    return (m, sd, med, max(vals))


def coverage(rows):
    out = []
    for r in rows:
        if r.get("feat", 0) > 0:
            out.append(100.0 * r["success"] / r["feat"])
    return out


def summarize(tag, rows):
    feat = col(rows, "feat")
    succ = col(rows, "success")
    cov = coverage(rows)
    cost = col(rows, "cost_ms")
    cm, cs, cmed, cmax = stats(cov)
    sm, ss, _, _ = stats(succ)
    tm, ts, _, tmax = stats(cost)
    total_succ = int(sum(succ))
    total_feat = int(sum(feat))
    plane = int(sum(col(rows, "plane")))
    line = int(sum(col(rows, "line")))
    fb = int(sum(col(rows, "fallback")))
    noNbr = int(sum(col(rows, "noNbr")))
    sight = int(sum(col(rows, "sight")))
    clus = int(sum(col(rows, "cluster")))
    scat = int(sum(col(rows, "scatter")))
    print(f"\n===== [{tag}] 帧数={len(rows)} =====")
    print(f"  深度覆盖率(success/feat): mean={cm:5.2f}%  std={cs:4.2f}  median={cmed:5.2f}%  max={cmax:5.2f}%")
    print(f"  每帧成功深度数         : mean={sm:5.2f}   std={ss:4.2f}")
    print(f"  累计成功/特征          : {total_succ} / {total_feat}")
    print(f"  成功来源(累计)         : plane={plane}  line={line}  fallback={fb}")
    if total_succ > 0:
        print(f"  成功来源占比           : plane={100*plane/total_succ:4.1f}%  "
              f"line={100*line/total_succ:4.1f}%  fallback={100*fb/total_succ:4.1f}%")
    print(f"  拒绝构成(累计)         : noNbr={noNbr}  sight={sight}  cluster={clus}  scatter={scat}")
    print(f"  单帧耗时               : mean={tm:5.3f}ms  std={ts:.3f}  max={tmax:.3f}ms")
    return dict(cov_mean=cm, cov_std=cs, cov_med=cmed,
                succ_mean=sm, total_succ=total_succ, total_feat=total_feat,
                plane=plane, line=line, fb=fb, cost_mean=tm)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--on", required=True, help="ON(改进二) 的逐帧CSV")
    ap.add_argument("--off", required=True, help="OFF(基线) 的逐帧CSV")
    ap.add_argument("--name", default="seq", help="序列名(用于标题)")
    ap.add_argument("--plot", default="", help="给出目录则保存覆盖率分布图(需 matplotlib)")
    args = ap.parse_args()

    on = load(args.on)
    off = load(args.off)
    print(f"\n########## 序列: {args.name} —— 改进点二消融(深度恢复层面) ##########")
    s_on = summarize("ON  改进二", on)
    s_off = summarize("OFF 基线 ", off)

    print("\n===== 对比(ON vs OFF) =====")
    dcov = s_on["cov_mean"] - s_off["cov_mean"]
    dsucc = s_on["succ_mean"] - s_off["succ_mean"]
    print(f"  平均深度覆盖率: {s_off['cov_mean']:.2f}% -> {s_on['cov_mean']:.2f}%  "
          f"(Δ={dcov:+.2f} 个百分点)")
    print(f"  每帧成功深度数: {s_off['succ_mean']:.2f} -> {s_on['succ_mean']:.2f}  (Δ={dsucc:+.2f})")
    print(f"  累计成功深度数: {s_off['total_succ']} -> {s_on['total_succ']}  "
          f"(Δ={s_on['total_succ']-s_off['total_succ']:+d})")
    print(f"  ON 中线状结构(line)贡献深度: {s_on['line']} "
          f"(基线3-NN平面拟合无法稳健恢复的部分)")
    print(f"  单帧耗时: {s_off['cost_mean']:.3f}ms -> {s_on['cost_mean']:.3f}ms")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            os.makedirs(args.plot, exist_ok=True)
            cov_on, cov_off = coverage(on), coverage(off)
            fig, ax = plt.subplots(1, 2, figsize=(11, 4))
            ax[0].hist(cov_off, bins=30, alpha=0.6, label="OFF baseline")
            ax[0].hist(cov_on, bins=30, alpha=0.6, label="ON robust")
            ax[0].set_xlabel("per-frame depth coverage (%)")
            ax[0].set_ylabel("frame count")
            ax[0].legend(); ax[0].set_title(f"{args.name} coverage dist.")
            ax[1].plot(cov_off, lw=0.7, label="OFF baseline")
            ax[1].plot(cov_on, lw=0.7, label="ON robust")
            ax[1].set_xlabel("frame index"); ax[1].set_ylabel("coverage (%)")
            ax[1].legend(); ax[1].set_title(f"{args.name} coverage over time")
            out = os.path.join(args.plot, f"{args.name}_depth_coverage.png")
            fig.tight_layout(); fig.savefig(out, dpi=150)
            print(f"\n[plot] 已保存: {out}")
        except ImportError:
            print("\n[plot] 未安装 matplotlib, 跳过画图 (pip install matplotlib)")


if __name__ == "__main__":
    main()
