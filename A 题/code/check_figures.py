# -*- coding: utf-8 -*-
"""图片严谨性自检：目录结构、完整性、数值一致性、快照图合法性断言。"""
import importlib.util
import math
import os
import sys

sys.path.insert(0, r"d:\数模校赛\A 题\code")
sys.stdout.reconfigure(encoding="utf-8")
from PIL import Image

from solve_q1 import load_case
from solve_q2_q3 import RESULT1, RESULT2, RESULT3, dynamic_metric, load_routes, load_zones, static_metric

FIG = r"d:\数模校赛\A 题\figures"

print("== 1. 目录结构与图片完整性 ==")
count = 0
fails = []
for root, dirs, files in os.walk(FIG):
    for f in sorted(files):
        if not f.lower().endswith(".png"):
            continue
        p = os.path.join(root, f)
        try:
            im = Image.open(p)
            im.load()
            ok = im.size[0] > 100 and im.size[1] > 100
            if not ok:
                fails.append(f)
        except Exception as exc:
            fails.append(f + ":" + str(exc))
            continue
        count += 1
        print("  %-42s %s" % (os.path.relpath(p, FIG), im.size))
print("  共 %d 张，异常 %d 张" % (count, len(fails)))
for f in fails:
    print("  FAIL:", f)

print()
print("== 2. 数值一致性核对（与审计口径） ==")
for sheet in ("Case1", "Case2", "Case3", "Case4"):
    pts, _ = load_case(sheet)
    coords = {int(p[0]): (float(p[1]), float(p[2])) for p in pts}
    r1 = load_routes(RESULT1, sheet)
    r2 = load_routes(RESULT2, sheet)
    r3 = load_routes(RESULT3, sheet)
    zones = load_zones(sheet)
    t1 = [static_metric(r, coords).total / 3600 for r in r1]
    t2 = [static_metric(r, coords).total / 3600 for r in r2]
    t3 = [dynamic_metric(r, coords, zones).total / 3600 for r in r3]
    print("  %s: Q1 Tmax=%.4f  Q2 Tmax=%.4f  Q2 d=%.4f  Q3 Tmax=%.4f  Q3 d=%.4f"
          % (sheet, max(t1), max(t2), max(t2) - min(t2), max(t3), max(t3) - min(t3)))

print()
print("== 3. 下界图数据核对（cycle_cover_bounds.txt avg 列） ==")
bp = os.path.join(os.path.dirname(FIG), "docs", "cycle_cover_bounds.txt")
if os.path.exists(bp):
    for line in open(bp, encoding="utf-8"):
        parts = line.split()
        if len(parts) >= 7 and parts[0] in ("Case1", "Case2", "Case3", "Case4"):
            avg = float(parts[5])
            verdict = "红(排除)" if avg > 9.0 else "绿(未排除)"
            print("  %s N=%s avg=%.4fh %s" % (parts[0], parts[1], avg, verdict))

print()
print("== 4. 快照图严谨性断言：快照时刻无无人机位于生效圆内 ==")
spec = importlib.util.spec_from_file_location("viz", r"d:\数模校赛\A 题\code\visualize.py")
viz = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viz)
viol = 0
for sheet in ("Case1", "Case2", "Case3", "Case4"):
    coords, levels = viz._coords_and_levels(sheet)
    zones = load_zones(sheet)
    routes = load_routes(RESULT3, sheet)
    traces = []
    for r in routes:
        acts, labels, total = viz._trace_route(r, coords, zones)
        traces.append((acts, total))
    times = sorted({z.start / 3600 for z in zones} | {z.end / 3600 for z in zones})
    for t in times:
        active = [z for z in zones if z.start / 3600 - 1e-9 <= t <= z.end / 3600 + 1e-9]
        for k, (acts, total) in enumerate(traces):
            pos = viz._position_at(acts, t * 3600)
            if pos is None:
                continue
            for z in active:
                if math.dist(pos, (z.x, z.y)) < z.radius - 1e-6:
                    viol += 1
                    print("  违规: %s t=%.2fh UAV%d 在生效圆 %s 内" % (sheet, 8 + t, k + 1, z.zid))
print("  违规数: %d -> %s" % (viol, "PASS" if viol == 0 else "FAIL"))

print()
print("== 5. 中文标注字体检查 ==")
import matplotlib.font_manager as fm
fonts = {f.name for f in fm.fontManager.ttflist}
print("  Microsoft YaHei 可用:", "Microsoft YaHei" in fonts, "| SimHei 可用:", "SimHei" in fonts)
print()
print("总体结论:", "全部通过" if not fails and viol == 0 else "存在异常")
