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
from solve_q2_q3 import RESULT1, load_routes, optimise, static_metric

CASES = ("Case1", "Case2", "Case3", "Case4")
RELAXES = (0.0, 0.005, 0.01, 0.02, 0.05)
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "figures", "q2", "q2_tradeoff_data.json")


def main() -> int:
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    results = {}
    for sheet in CASES:
        pts, _ = load_case(sheet)
        coords = {int(p[0]): (float(p[1]), float(p[2])) for p in pts}
        initial = load_routes(RESULT1, sheet)
        metric = lambda r, c=coords: static_metric(r, c)
        tmax_star = max(metric(r).total for r in initial)
        print(f"[{sheet}] Tmax* = {tmax_star/3600:.4f} h", flush=True)
        points = []
        for idx, relax in enumerate(RELAXES):
            t0 = time.time()
            # 多种子多次运行取最优，降低随机性噪声
            best_times = None
            best_key = None
            for sd in (20260900 + idx, 20260917 + idx, 20260931 + idx):
                r2, m2 = optimise(initial, metric, seed=sd,
                                  iterations=20000, restarts=2, balance_weight=0.015,
                                  tmax_cap=tmax_star, relax=relax)
                ts = [m.total / 3600.0 for m in m2]
                key = (max(ts), max(ts) - min(ts))
                if best_key is None or key < best_key:
                    best_key = key
                    best_times = ts
            pt = {
                "relax": relax,
                "cap_h": tmax_star * (1.0 + relax) / 3600.0,
                "Tmax_h": max(best_times),
                "delta_h": max(best_times) - min(best_times),
            }
            points.append(pt)
            print(f"    eps={relax:g}: Tmax={pt['Tmax_h']:.4f}h "
                  f"delta={pt['delta_h']:.4f}h ({time.time()-t0:.0f}s)", flush=True)
        results[sheet] = {"Tmax_star_h": tmax_star / 3600.0, "points": points}
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print("权衡数据已写入", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
