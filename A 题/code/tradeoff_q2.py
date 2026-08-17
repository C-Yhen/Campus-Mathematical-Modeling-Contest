# -*- coding: utf-8 -*-
"""问题2 Tmax-δ 权衡曲线数据：不同容差 ε 下运行分层遗传算法。

对每个算例、每个容差 ε ∈ {0, 0.5%, 1%, 2%, 5%} 求解
  min δ   s.t. Tmax <= Tmax*(1+ε)
记录 (ε, Tmax, δ) 点，输出到 figures/q2/q2_tradeoff_data.json。
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from solve_q1 import load_case
from solve_q2_q3 import (
    RESULT1,
    RESULT2,
    cover_signature,
    expected_signature,
    hierarchical_key,
    load_routes,
    optimise,
    static_metric,
    valid_route,
    write_workbook,
)

CASES = ("Case1", "Case2", "Case3", "Case4")
RELAXES = (0.0, 0.005, 0.01, 0.02, 0.05)
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "figures", "q2", "q2_tradeoff_data.json")


def main() -> int:
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    results = {}
    adopted = {}  # 各算例 ε=0 的最优路线（写回 result2）
    for sheet in CASES:
        pts, _ = load_case(sheet)
        coords = {int(p[0]): (float(p[1]), float(p[2])) for p in pts}
        initial = load_routes(RESULT1, sheet)
        official_q2 = load_routes(RESULT2, sheet)
        metric = lambda r, c=coords: static_metric(r, c)
        tmax_star = max(metric(r).total for r in initial)
        print(f"[{sheet}] Tmax* = {tmax_star/3600:.4f} h", flush=True)
        points = []
        # 容差逐步放宽时，上一容差下的最优解在当前容差下仍然可行。
        # 将它作为下一轮初始解，既能热启动，也能保证“已发现的”δ 不退化。
        incumbent = [list(r) for r in initial]
        for idx, relax in enumerate(RELAXES):
            t0 = time.time()
            # 多种子多次运行取最优，降低随机性噪声
            best_times = None
            best_key = None
            best_routes = None

            def consider(routes, metrics=None):
                nonlocal incumbent, best_times, best_key, best_routes
                metrics = metrics or [metric(r) for r in routes]
                times = [m.total for m in metrics]
                key = hierarchical_key(times, tmax_star, relax)
                if best_key is None or key < best_key:
                    best_key = key
                    best_times = [t / 3600.0 for t in times]
                    best_routes = [list(r) for r in routes]
                    incumbent = [list(r) for r in routes]

            # 显式纳入上一容差解和正式问题2方案，避免随机搜索漏掉已知优解。
            consider(incumbent)
            consider(official_q2)
            for sd in (20260900 + idx, 20260917 + idx, 20260931 + idx):
                r2, m2 = optimise(incumbent, metric, seed=sd,
                                  iterations=20000, restarts=2, balance_weight=0.015,
                                  tmax_cap=tmax_star, relax=relax)
                consider(r2, m2)
            pt = {
                "relax": relax,
                "cap_h": tmax_star * (1.0 + relax) / 3600.0,
                "Tmax_h": max(best_times),
                "delta_h": max(best_times) - min(best_times),
                "routes": best_routes,
            }
            points.append(pt)
            if relax == 0.0:
                adopted[sheet] = best_routes
            print(f"    eps={relax:g}: Tmax={pt['Tmax_h']:.4f}h "
                  f"delta={pt['delta_h']:.4f}h ({time.time()-t0:.0f}s)", flush=True)
        results[sheet] = {"Tmax_star_h": tmax_star / 3600.0, "points": points}
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print("权衡数据已写入", OUT)
    # ε=0 方案写回 result2：覆盖合法性与路线合法性断言
    for sheet, routes in adopted.items():
        assert cover_signature(routes) == expected_signature(sheet), sheet
        assert all(valid_route(r) for r in routes), sheet
    write_workbook(RESULT2, adopted)
    print("ε=0 最优路线已写回", RESULT2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
