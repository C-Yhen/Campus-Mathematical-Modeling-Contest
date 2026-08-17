# -*- coding: utf-8 -*-
"""Q2 路线更新后联动重跑问题 3，并刷新 result3.xlsx 与 q2_q3_summary.txt。"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from solve_q2_q3 import (
    RESULT2,
    RESULT3,
    SUMMARY,
    cyclic_improve,
    dynamic_metric,
    lex_key,
    load_case,
    load_routes,
    load_zones,
    optimise,
    static_metric,
    summary_line,
    write_workbook,
)

CASES = ("Case1", "Case2", "Case3", "Case4")
CKPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_redo_q3_checkpoint.json")
OLD3 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_old_result3.xlsx")


def times_of(routes, metric):
    return [metric(r).total for r in routes]


def main() -> int:
    if os.path.exists(CKPT):
        with open(CKPT, encoding="utf-8") as f:
            done = json.load(f)
    else:
        done = {}
    q3_routes, q3_metrics = {}, {}
    for idx, sheet in enumerate(CASES, 1):
        if sheet in done:
            print(f"[{sheet}] 已完成，跳过", flush=True)
            q3_routes[sheet] = done[sheet]
            continue
        points, _ = load_case(sheet)
        coords = {int(p[0]): (float(p[1]), float(p[2])) for p in points}
        r2 = load_routes(RESULT2, sheet)
        metric2 = lambda r, c=coords: static_metric(r, c)
        m2 = [metric2(r) for r in r2]
        times2 = [m.total for m in m2]
        print(f"[{sheet}] 新问题2: Tmax={max(times2)/3600:.4f}h, "
              f"delta={(max(times2)-min(times2))/3600:.4f}h", flush=True)

        print(f"[{sheet}] 问题3 联动重优化", flush=True)
        zones = load_zones(sheet)
        metric3 = lambda r, c=coords, z=zones: dynamic_metric(r, c, z)
        cyclic_routes, cyclic_metrics = cyclic_improve(r2, metric3)
        cyclic_times = times_of(cyclic_routes, metric3)
        print(f"    循环切入优化：Tmax={max(cyclic_times)/3600:.4f}h, "
              f"spread={(max(cyclic_times)-min(cyclic_times))/3600:.4f}h", flush=True)
        r3, m3 = optimise(cyclic_routes, metric3, seed=20260916 + idx,
                          iterations=50000, restarts=4, balance_weight=0.025)
        t3 = times_of(r3, metric3)
        print(f"    Q3(新路线): Tmax={max(t3)/3600:.4f}h, "
              f"delta={(max(t3)-min(t3))/3600:.4f}h", flush=True)
        # 旧 Q3 方案仍是合法方案，纳入候选避免结果退化；若更优则以它为起点再搜一轮
        if os.path.exists(OLD3):
            old = load_routes(OLD3, sheet)
            old_cyc, _ = cyclic_improve(old, metric3)
            old_times = times_of(old_cyc, metric3)
            print(f"    旧 Q3(循环切入): Tmax={max(old_times)/3600:.4f}h, "
                  f"delta={(max(old_times)-min(old_times))/3600:.4f}h", flush=True)
            if lex_key(old_times) < lex_key(t3):
                r3b, m3b = optimise(old_cyc, metric3, seed=20260916 + idx,
                                    iterations=50000, restarts=4, balance_weight=0.025)
                t3b = times_of(r3b, metric3)
                print(f"    旧 Q3 再搜索: Tmax={max(t3b)/3600:.4f}h, "
                      f"delta={(max(t3b)-min(t3b))/3600:.4f}h", flush=True)
                if lex_key(t3b) < lex_key(t3):
                    r3, m3, t3 = r3b, m3b, t3b
        q3_routes[sheet], q3_metrics[sheet] = r3, m3
        print(f"    Q3 采用: Tmax={max(t3)/3600:.4f}h, "
              f"delta={(max(t3)-min(t3))/3600:.4f}h", flush=True)
        done[sheet] = r3
        with open(CKPT, "w", encoding="utf-8") as f:
            json.dump(done, f, ensure_ascii=False)

    write_workbook(RESULT3, q3_routes)
    # 汇总：Q2 用新 result2，Q3 用新 result3（metrics 统一重算，兼作复核）
    q2_routes = {s: load_routes(RESULT2, s) for s in CASES}
    q3_metrics = {}
    for sheet in CASES:
        points, _ = load_case(sheet)
        coords = {int(p[0]): (float(p[1]), float(p[2])) for p in points}
        zones = load_zones(sheet)
        q3_metrics[sheet] = [dynamic_metric(r, coords, zones)
                             for r in q3_routes[sheet]]
    os.makedirs(os.path.dirname(SUMMARY), exist_ok=True)
    with open(SUMMARY, "w", encoding="utf-8") as f:
        f.write("问题2（固定问题1机队规模；ε=0 主口径，多种子热启动分层 GA）\n")
        for sheet in CASES:
            points, _ = load_case(sheet)
            coords = {int(p[0]): (float(p[1]), float(p[2])) for p in points}
            m2s = [static_metric(r, coords) for r in q2_routes[sheet]]
            f.write(summary_line(sheet, q2_routes[sheet], m2s))
        f.write("\n问题3（08:00出发；等待直飞与圆外切线绕飞取较早到达）\n")
        for sheet in CASES:
            f.write(summary_line(sheet, q3_routes[sheet], q3_metrics[sheet]))
    if os.path.exists(CKPT):
        os.remove(CKPT)
    print(f"已写入 {RESULT3}\n已写入 {SUMMARY}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
