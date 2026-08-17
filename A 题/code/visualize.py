# -*- coding: utf-8 -*-
"""A 题三问可视化脚本（只读结果文件，不修改求解器）。

生成图片清单（输出到 A 题/figures/）：
  1. q1_routes_{case}.png     问题1 各算例的无人机巡检路线图
  2. q1_lowerbound.png        问题1 循环覆盖下界 vs 9h 论证图（4 算例）
  3. q2_balance_{case}.png    问题2 各机时长均衡对比（Q1 vs Q2 + 容差带）
  4. q3_spacetime_{case}.png  问题3 时空轨迹图（禁飞区 + 路径 + 等待/绕行）
  5. q3_timeline_{case}.png   问题3 时间轴（飞行/服务/等待 + 禁飞窗带）
  6. summary.png              三问 Tmax / delta 汇总对比

用法：python visualize.py [case]   # 不给参数则生成全部
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from solve_q1 import LEVEL_TIMES, load_case
from solve_q2_q3 import (
    EPS, RESULT1, RESULT2, RESULT3, SERVICE_S,
    clearance_after_visit, dynamic_metric, load_routes, load_zones, static_metric,
)
from audit_q2_q3 import _timed_leg_trace

FIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "figures")
FIG_Q1 = os.path.join(FIG_DIR, "q1")
FIG_Q2 = os.path.join(FIG_DIR, "q2")
FIG_Q3 = os.path.join(FIG_DIR, "q3")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"]
LVL_STYLE = {"I": ("^", "#d62728"), "II": ("s", "#ff7f0e"), "III": ("o", "#1f77b4")}
CASES = ("Case1", "Case2", "Case3", "Case4")


# ---------------------------- 公共数据 ----------------------------
def _coords_and_levels(sheet):
    pts, _ = load_case(sheet)
    coords = {int(p[0]): (float(p[1]), float(p[2])) for p in pts}
    levels = {int(p[0]): str(p[3]).strip() for p in pts}
    return coords, levels


def _trace_route(route, coords, zones):
    """逐段重建轨迹：返回 (activities, labels, total)。activities 为
    (kind, t0, t1, x0, y0, x1, y1, note) 列表。"""
    now = 0.0
    prev = (0.0, 0.0)
    activities, labels = [], []
    dests = [coords[p] for p in route] + [(0.0, 0.0)]
    servs = [SERVICE_S] * len(route) + [0.0]
    for leg_no, (dst, serv) in enumerate(zip(dests, servs), 1):
        following = dests[leg_no] if leg_no < len(dests) else None
        clear = None
        if following is not None and serv > 0.0:
            clear = {}
            for z in zones:
                v = clearance_after_visit(route, leg_no - 1, coords, z)
                if v > EPS:
                    clear[z.zid] = v
        metric, acts, label = _timed_leg_trace(now, prev, dst, serv, zones, clear)
        for a in acts:
            activities.append((a.kind, a.start, a.end, a.a[0], a.a[1], a.b[0], a.b[1], a.note))
        labels.append(label)
        if not math.isfinite(metric.total):
            return activities, labels, float("inf")
        now += metric.total
        prev = dst
    return activities, labels, now


# ---------------------------- 图1：Q1 路线图 ----------------------------
def plot_q1_routes(sheets=CASES):
    for sheet in sheets:
        coords, levels = _coords_and_levels(sheet)
        routes = load_routes(RESULT1, sheet)
        fig, ax = plt.subplots(figsize=(6.4, 6.4))
        # 巡检点（按等级）
        for lvl, (mk, c) in LVL_STYLE.items():
            ids = [p for p, l in levels.items() if l == lvl]
            if ids:
                ax.scatter([coords[p][0] for p in ids], [coords[p][1] for p in ids],
                           marker=mk, s=26, c=c, edgecolors="white", linewidths=0.4,
                           label=f"{lvl}级（{LEVEL_TIMES[lvl]}次）", zorder=3)
        # 基地
        ax.scatter([0], [0], marker="*", s=260, c="black", zorder=4, label="基地 (0,0)")
        # 各机路径
        for k, r in enumerate(routes):
            xs, ys = [0.0], [0.0]
            for pid in r:
                xs.append(coords[pid][0])
                ys.append(coords[pid][1])
            xs.append(0.0)
            ys.append(0.0)
            ax.plot(xs, ys, "-", color=COLORS[k % len(COLORS)], linewidth=1.4,
                    alpha=0.85, label=f"UAV{k+1}（{len(r)}点）", zorder=2)
        ax.set_title(f"问题1 巡检路线图（{sheet}，N={len(routes)}）")
        ax.set_xlabel("x（坐标单位，1单位=100m）")
        ax.set_ylabel("y（坐标单位）")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=8, loc="best")
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(os.path.join(FIG_Q1, f"q1_routes_{sheet}.png"), dpi=150)
        plt.close(fig)
    print("图1 完成：q1_routes_*.png")


# ---------------------------- 图2：Q1 下界论证 ----------------------------
def plot_q1_lowerbound():
    bounds_path = os.path.join(os.path.dirname(FIG_DIR), "docs", "cycle_cover_bounds.txt")
    data = {}
    if os.path.exists(bounds_path):
        for line in open(bounds_path, encoding="utf-8"):
            parts = line.split()
            # 列: case N visits distance_lb(km) total_lb(h) avg_lb(h) excluded
            if len(parts) >= 7 and parts[0] in CASES:
                data.setdefault(parts[0], []).append((int(parts[1]), float(parts[5])))
    if not data:
        print("图2 跳过：未找到 cycle_cover_bounds.txt")
        return
    fig, axes = plt.subplots(1, 4, figsize=(14, 3.2))
    for ax, sheet in zip(axes, CASES):
        rows = data.get(sheet, [])
        xs = [r[0] for r in rows]
        ys = [r[1] for r in rows]
        colors = ["#2ca02c" if y <= 9.0 else "#d62728" for y in ys]
        ax.bar(xs, ys, color=colors, alpha=0.85)
        ax.axhline(9.0, color="k", linestyle="--", linewidth=1.0)
        ax.text(0.02, 9.05, "9h 判定线", fontsize=8)
        ax.set_title(sheet, fontsize=11)
        ax.set_xlabel("N（无人机数）")
        ax.set_ylabel("平均负载下界 (h/架)")
        ax.set_ylim(0, max(ys) * 1.15)
        ax.grid(alpha=0.25)
    fig.suptitle("问题1 循环覆盖平均负载下界（绿色=未能排除；红色=该 N 严格不可行）",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_Q1, "q1_lowerbound.png"), dpi=150)
    plt.close(fig)
    print("图2 完成：q1_lowerbound.png")


# ---------------------------- 图3：Q2 均衡对比 ----------------------------
def plot_q2_balance(sheets=CASES):
    for sheet in sheets:
        coords, _ = _coords_and_levels(sheet)
        r1 = load_routes(RESULT1, sheet)
        r2 = load_routes(RESULT2, sheet)
        t1 = [static_metric(r, coords).total / 3600 for r in r1]
        t2 = [static_metric(r, coords).total / 3600 for r in r2]
        n = len(r1)
        fig, ax = plt.subplots(figsize=(6.6, 3.6))
        x = list(range(n))
        w = 0.38
        ax.bar([i - w / 2 for i in x], t1, w, color="#9ecae1", label="问题1 方案")
        ax.bar([i + w / 2 for i in x], t2, w, color="#1f77b4", label="问题2 方案")
        tmax_star = max(t1)
        ax.axhline(tmax_star, color="gray", linestyle="--", linewidth=1.0)
        ax.axhspan(tmax_star, tmax_star * 1.01, color="#ffd92f", alpha=0.30)
        ax.text(n - 0.45, tmax_star * 1.008, "Tmax* 容差带 (+1%)", fontsize=8, va="bottom")
        ax.set_xticks(x)
        ax.set_xticklabels([f"UAV{i+1}" for i in x])
        ax.set_ylabel("工作时长 (h)")
        ax.set_title(f"问题2 负载均衡（{sheet}）：δ {max(t1)-min(t1):.3f}h → "
                     f"{max(t2)-min(t2):.3f}h")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.25, axis="y")
        fig.tight_layout()
        fig.savefig(os.path.join(FIG_Q2, f"q2_balance_{sheet}.png"), dpi=150)
        plt.close(fig)
    print("图3 完成：q2_balance_*.png")


# ---------------------------- 图4：Q3 时空轨迹 ----------------------------
def plot_q3_spacetime(sheets=CASES):
    for sheet in sheets:
        coords, levels = _coords_and_levels(sheet)
        zones = load_zones(sheet)
        routes = load_routes(RESULT3, sheet)
        fig, ax = plt.subplots(figsize=(7.4, 7.4))
        # 巡检点
        for lvl, (mk, c) in LVL_STYLE.items():
            ids = [p for p, l in levels.items() if l == lvl]
            if ids:
                ax.scatter([coords[p][0] for p in ids], [coords[p][1] for p in ids],
                           marker=mk, s=22, c=c, edgecolors="white", linewidths=0.4,
                           label=f"{lvl}级", zorder=3)
        ax.scatter([0], [0], marker="*", s=260, c="black", zorder=4, label="基地")
        # 分时禁飞区：灰色虚线表示“仅在标注时段生效”，避免误读为全天禁飞
        for z in zones:
            h0 = z.start / 3600 + 8
            h1 = z.end / 3600 + 8
            ax.add_patch(Circle((z.x, z.y), z.radius, fill=False,
                                edgecolor="#888888", linewidth=1.2,
                                linestyle=(0, (4, 3)), zorder=1))
            ax.text(z.x, z.y + z.radius + 3, f"{z.zid} {h0:.1f}-{h1:.1f}h",
                    fontsize=7.5, color="#555555", ha="center")
        # 各机轨迹 + 等待点
        for k, r in enumerate(routes):
            color = COLORS[k % len(COLORS)]
            acts, labels, total = _trace_route(r, coords, zones)
            xs, ys = [0.0], [0.0]
            for kind, t0, t1, x0, y0, x1, y1, note in acts:
                if kind == "flight":
                    if xs and (xs[-1] != x0 or ys[-1] != y0):
                        xs.append(x0)
                        ys.append(y0)
                    xs.append(x1)
                    ys.append(y1)
            ax.plot(xs, ys, "-", color=color, linewidth=1.5, alpha=0.9,
                    label=f"UAV{k+1}（{total/3600:.2f}h）", zorder=2)
            for kind, t0, t1, x0, y0, x1, y1, note in acts:
                if kind == "wait" and t1 > t0 + 1e-3:
                    ax.scatter([x0], [y0], marker="v", s=60, color=color,
                               edgecolors="black", linewidths=0.5, zorder=5)
        ax.set_title(f"问题3 全天轨迹投影（{sheet}，N={len(routes)}）\n"
                     f"灰虚圈=分时禁飞区（仅在标注时段生效）；路径穿圈仅发生在管制时段外",
                     fontsize=10.5)
        ax.set_xlabel("x（坐标单位）")
        ax.set_ylabel("y（坐标单位）")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=7.5, loc="best")
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(os.path.join(FIG_Q3, f"q3_spacetime_{sheet}.png"), dpi=150)
        plt.close(fig)
    print("图4 完成：q3_spacetime_*.png")


# ---------------------------- 图4b：Q3 多时刻快照 ----------------------------
def _position_at(acts, t):
    for kind, t0, t1, x0, y0, x1, y1, note in acts:
        if t0 - 1e-6 <= t <= t1 + 1e-6:
            if kind in ("wait", "service") or t1 <= t0:
                return (x0, y0)
            f = (t - t0) / (t1 - t0)
            return (x0 + (x1 - x0) * f, y0 + (y1 - y0) * f)
    return None


def plot_q3_snapshots(sheets=CASES):
    for sheet in sheets:
        coords, levels = _coords_and_levels(sheet)
        zones = load_zones(sheet)
        routes = load_routes(RESULT3, sheet)
        traces = []
        for r in routes:
            acts, labels, total = _trace_route(r, coords, zones)
            traces.append((acts, total))
        # 快照时刻：每个禁飞窗的起始与结束（保证覆盖“最危险”的时刻）
        times = sorted({z.start / 3600 for z in zones} | {z.end / 3600 for z in zones})
        times = [t for t in times if t <= max((tr[1] for tr in traces), default=0) / 3600]
        if not times:
            continue
        cols = 3
        rows_n = (len(times) + cols - 1) // cols
        fig, axes = plt.subplots(rows_n, cols, figsize=(cols * 4.4, rows_n * 4.2))
        flat = axes.flatten() if hasattr(axes, "flatten") else [axes]
        for ax, t in zip(flat, times):
            for lvl, (mk, c) in LVL_STYLE.items():
                ids = [p for p, l in levels.items() if l == lvl]
                if ids:
                    ax.scatter([coords[p][0] for p in ids], [coords[p][1] for p in ids],
                               marker=mk, s=16, c=c, edgecolors="white", linewidths=0.3, zorder=2)
            ax.scatter([0], [0], marker="*", s=200, c="black", zorder=4)
            for z in zones:
                active = z.start / 3600 <= t <= z.end / 3600
                if active:
                    ax.add_patch(Circle((z.x, z.y), z.radius, fill=True,
                                        facecolor="#d62728", alpha=0.12,
                                        edgecolor="#d62728", linewidth=1.6, zorder=1))
                    ax.text(z.x, z.y, z.zid, fontsize=8, color="#a50f15",
                            ha="center", va="center", fontweight="bold")
                else:
                    ax.add_patch(Circle((z.x, z.y), z.radius, fill=False,
                                        edgecolor="#aaaaaa", linewidth=0.9,
                                        linestyle=(0, (4, 3)), zorder=1))
            for k, (acts, total) in enumerate(traces):
                pos = _position_at(acts, t * 3600)
                if pos is not None:
                    ax.scatter([pos[0]], [pos[1]], marker="o", s=60,
                               c=COLORS[k % len(COLORS)], edgecolors="black",
                               linewidths=0.6, zorder=5)
            ax.set_title(f"t = {8 + t:.2f} h（红=生效禁飞区）", fontsize=10)
            ax.set_aspect("equal", adjustable="datalim")
            ax.tick_params(labelsize=7)
            ax.grid(alpha=0.2)
        for ax in flat[len(times):]:
            ax.axis("off")
        fig.suptitle(f"问题3 分时禁飞区快照（{sheet}）：各无人机位置与当时生效的禁飞区",
                     fontsize=12)
        fig.tight_layout()
        fig.savefig(os.path.join(FIG_Q3, f"q3_snapshots_{sheet}.png"), dpi=150)
        plt.close(fig)
    print("图4b 完成：q3_snapshots_*.png")


# ---------------------------- 图5：Q3 时间轴 ----------------------------
def plot_q3_timeline(sheets=CASES):
    for sheet in sheets:
        coords, _ = _coords_and_levels(sheet)
        zones = load_zones(sheet)
        routes = load_routes(RESULT3, sheet)
        n = len(routes)
        fig, ax = plt.subplots(figsize=(10, 1.0 + 0.85 * n))
        # 禁飞窗带
        for zi, z in enumerate(zones):
            ax.axvspan(z.start / 3600, z.end / 3600, color="#d62728", alpha=0.10)
            ax.text((z.start + z.end) / 7200, n + 0.30 + 0.35 * (zi % 2),
                    f"{z.zid}", fontsize=7, color="#a50f15", ha="center")
        for k, r in enumerate(routes):
            acts, labels, total = _trace_route(r, coords, zones)
            for kind, t0, t1, x0, y0, x1, y1, note in acts:
                h0, h1 = t0 / 3600, t1 / 3600
                if h1 - h0 < 1e-5:
                    continue
                if kind == "flight":
                    ax.barh(n - 1 - k, h1 - h0, left=h0, height=0.42,
                            color=COLORS[k % len(COLORS)], alpha=0.75)
                elif kind == "service":
                    ax.barh(n - 1 - k, h1 - h0, left=h0, height=0.30,
                            color="#ff7f0e", alpha=0.95)
                elif kind == "wait":
                    ax.barh(n - 1 - k, h1 - h0, left=h0, height=0.42,
                            color="#d62728", alpha=0.95)
        ax.set_yticks(list(range(n)))
        ax.set_yticklabels([f"UAV{i+1}" for i in range(n)][::-1])
        ax.set_xlabel("时间（h，自 08:00 起）")
        ax.set_xlim(0, max((z.end for z in zones), default=0) / 3600 + 0.5)
        ax.set_title(f"问题3 时间轴（{sheet}）：蓝=飞行 橙=服务 红=等待，浅红带=禁飞窗")
        ax.grid(alpha=0.25, axis="x")
        fig.tight_layout()
        fig.savefig(os.path.join(FIG_Q3, f"q3_timeline_{sheet}.png"), dpi=150)
        plt.close(fig)
    print("图5 完成：q3_timeline_*.png")


# ---------------------------- 图6：三问汇总 ----------------------------
def plot_summary():
    rows = []
    for sheet in CASES:
        coords, _ = _coords_and_levels(sheet)
        r1 = load_routes(RESULT1, sheet)
        r2 = load_routes(RESULT2, sheet)
        r3 = load_routes(RESULT3, sheet)
        zones = load_zones(sheet)
        t1 = [static_metric(r, coords).total / 3600 for r in r1]
        t2 = [static_metric(r, coords).total / 3600 for r in r2]
        t3 = [dynamic_metric(r, coords, zones).total / 3600 for r in r3]
        rows.append((sheet, len(r1), max(t1), max(t1) - min(t1), max(t2),
                     max(t2) - min(t2), max(t3), max(t3) - min(t3)))
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.4))
    x = list(range(4))
    # Tmax
    ax = axes[0]
    w = 0.26
    ax.bar([i - w for i in x], [r[2] for r in rows], w, color="#9ecae1", label="问题1")
    ax.bar([i for i in x], [r[4] for r in rows], w, color="#1f77b4", label="问题2")
    ax.bar([i + w for i in x], [r[6] for r in rows], w, color="#d62728", label="问题3")
    ax.axhline(9, color="k", linestyle="--", linewidth=1)
    ax.set_xticks(x)
    ax.set_xticklabels([r[0] for r in rows])
    ax.set_ylabel("Tmax (h)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25, axis="y")
    # delta
    ax = axes[1]
    ax.bar([i - w / 2 for i in x], [r[5] for r in rows], w, color="#1f77b4", label="问题2 δ")
    ax.bar([i + w / 2 for i in x], [r[7] for r in rows], w, color="#d62728", label="问题3 δ")
    ax.set_xticks(x)
    ax.set_xticklabels([r[0] for r in rows])
    ax.set_ylabel("δ (h)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25, axis="y")
    n_labels = " / ".join(str(r[1]) for r in rows)
    fig.suptitle(f"三问结果汇总（各算例 N：{n_labels}）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "summary.png"), dpi=150)
    plt.close(fig)
    print("图6 完成：summary.png")


# ---------------------------- 图7：渐变色路线图 ----------------------------
def plot_q1_routes_gradient(sheets=CASES):
    """路线按访问顺序着色（plasma 渐变色带），基地出发-返回全程可见顺序。"""
    from matplotlib.collections import LineCollection
    import numpy as np
    for sheet in sheets:
        coords, levels = _coords_and_levels(sheet)
        routes = load_routes(RESULT1, sheet)
        fig, ax = plt.subplots(figsize=(6.4, 6.4))
        for lvl, (mk, c) in LVL_STYLE.items():
            ids = [p for p, l in levels.items() if l == lvl]
            if ids:
                ax.scatter([coords[p][0] for p in ids], [coords[p][1] for p in ids],
                           marker=mk, s=26, c=c, edgecolors="white", linewidths=0.4,
                           label=f"{lvl}级（{LEVEL_TIMES[lvl]}次）", zorder=3)
        ax.scatter([0], [0], marker="*", s=260, c="black", zorder=5, label="基地 (0,0)")
        cmap = plt.get_cmap("plasma")
        for k, r in enumerate(routes):
            pts = [(0.0, 0.0)] + [coords[p] for p in r] + [(0.0, 0.0)]
            segs = [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
            lc = LineCollection(segs, cmap=cmap, linewidth=2.2, zorder=2)
            lc.set_array(np.linspace(0, 1, len(segs)))
            ax.add_collection(lc)
        ax.set_title(f"问题1 巡检路线渐变色图（{sheet}，N={len(routes)}）\n"
                     "颜色深浅表示访问先后（深紫=出发段，亮黄=返航段）", fontsize=10.5)
        ax.set_xlabel("x（坐标单位，1单位=100m）")
        ax.set_ylabel("y（坐标单位）")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=8, loc="best")
        ax.grid(alpha=0.25)
        fig.colorbar(lc, ax=ax, fraction=0.04, pad=0.02, label="访问进度")
        fig.tight_layout()
        fig.savefig(os.path.join(FIG_Q1, f"q1_routes_gradient_{sheet}.png"), dpi=150)
        plt.close(fig)
    print("图7 完成：q1_routes_gradient_*.png")


# ---------------------------- 图8：三问热力图 ----------------------------
def plot_summary_heatmap():
    import numpy as np
    q_tmax = np.zeros((4, 3))
    q_delta = np.zeros((4, 2))
    for i, sheet in enumerate(CASES):
        coords, _ = _coords_and_levels(sheet)
        r1 = load_routes(RESULT1, sheet)
        r2 = load_routes(RESULT2, sheet)
        r3 = load_routes(RESULT3, sheet)
        zones = load_zones(sheet)
        t1 = [static_metric(r, coords).total / 3600 for r in r1]
        t2 = [static_metric(r, coords).total / 3600 for r in r2]
        t3 = [dynamic_metric(r, coords, zones).total / 3600 for r in r3]
        q_tmax[i] = [max(t1), max(t2), max(t3)]
        q_delta[i] = [max(t2) - min(t2), max(t3) - min(t3)]
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 4.2))
    cmap1 = plt.get_cmap("RdYlGn_r")
    im = axes[0].imshow(q_tmax, cmap=cmap1, vmin=6.5, vmax=12.0, aspect="auto")
    axes[0].set_xticks(range(3))
    axes[0].set_xticklabels(["问题1", "问题2", "问题3"])
    axes[0].set_yticks(range(4))
    axes[0].set_yticklabels(CASES)
    for i in range(4):
        for j in range(3):
            axes[0].text(j, i, f"{q_tmax[i, j]:.3f}", ha="center", va="center",
                         color="white" if q_tmax[i, j] > 9.5 else "black", fontsize=9)
    axes[0].set_title("Tmax (h) 热力图")
    fig.colorbar(im, ax=axes[0], fraction=0.046, pad=0.02)
    cmap2 = plt.get_cmap("YlOrRd")
    im2 = axes[1].imshow(q_delta, cmap=cmap2, vmin=0, vmax=0.25, aspect="auto")
    axes[1].set_xticks(range(2))
    axes[1].set_xticklabels(["问题2", "问题3"])
    axes[1].set_yticks(range(4))
    axes[1].set_yticklabels(CASES)
    for i in range(4):
        for j in range(2):
            axes[1].text(j, i, f"{q_delta[i, j]:.3f}", ha="center", va="center",
                         color="white" if q_delta[i, j] > 0.15 else "black", fontsize=9)
    axes[1].set_title("δ (h) 热力图")
    fig.colorbar(im2, ax=axes[1], fraction=0.046, pad=0.02)
    fig.suptitle("三问结果热力图（颜色越深表示数值越大）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "summary_heatmap.png"), dpi=150)
    plt.close(fig)
    print("图8 完成：summary_heatmap.png")


# ---------------------------- 图9：Tmax-δ 权衡曲线 ----------------------------
def plot_q2_tradeoff(sheets=CASES):
    import json
    import numpy as np
    data_path = os.path.join(FIG_Q2, "q2_tradeoff_data.json")
    if not os.path.exists(data_path):
        print("图9 跳过：未找到 q2_tradeoff_data.json（先运行 tradeoff_q2.py）")
        return
    data = json.load(open(data_path, encoding="utf-8"))
    cmap = plt.get_cmap("coolwarm")
    for sheet in sheets:
        entry = data.get(sheet)
        if entry is None:
            continue
        pts = entry["points"]
        fig, ax = plt.subplots(figsize=(6.4, 4.2))
        cmap = plt.get_cmap("coolwarm")
        colors = [cmap(i / max(1, len(pts) - 1)) for i in range(len(pts))]
        for idx, p in enumerate(pts):
            ax.scatter([p["Tmax_h"]], [p["delta_h"]], s=110, color=colors[idx],
                       edgecolors="black", linewidths=0.6, zorder=3)
            ax.annotate(f"ε={p['relax']*100:.1f}%", (p["Tmax_h"], p["delta_h"]),
                        textcoords="offset points", xytext=(8, 5), fontsize=8)
        # Pareto 下包络（有效前沿）：按 Tmax 升序取 δ 单调下降的拐点
        ordered = sorted(pts, key=lambda p: p["Tmax_h"])
        envelope = []
        best_delta = float("inf")
        for p in ordered:
            if p["delta_h"] < best_delta - 1e-9:
                envelope.append(p)
                best_delta = p["delta_h"]
        if envelope:
            ax.plot([p["Tmax_h"] for p in envelope], [p["delta_h"] for p in envelope],
                    "-", color="#555555", linewidth=1.6, zorder=2, label="有效前沿")
        ax.set_xlabel("Tmax (h)")
        ax.set_ylabel("δ = Tmax - Tmin (h)")
        ax.set_title(f"问题2 均衡代价权衡曲线（{sheet}，Tmax*={entry['Tmax_star_h']:.3f}h）")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(FIG_Q2, f"q2_tradeoff_{sheet}.png"), dpi=150)
        plt.close(fig)
    print("图9 完成：q2_tradeoff_*.png")


def main():
    os.makedirs(FIG_Q1, exist_ok=True)
    os.makedirs(FIG_Q2, exist_ok=True)
    os.makedirs(FIG_Q3, exist_ok=True)
    args = sys.argv[1:]
    sheets = args if args else CASES
    plot_q1_routes(sheets)
    plot_q1_routes_gradient(sheets)
    plot_q1_lowerbound()
    plot_q2_balance(sheets)
    plot_q2_tradeoff(sheets)
    plot_q3_spacetime(sheets)
    plot_q3_snapshots(sheets)
    plot_q3_timeline(sheets)
    plot_summary()
    plot_summary_heatmap()
    print(f"全部图片已输出到 {FIG_DIR}")


if __name__ == "__main__":
    main()
