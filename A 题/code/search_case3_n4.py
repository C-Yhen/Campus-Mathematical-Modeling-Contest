# -*- coding: utf-8 -*-
"""Case3 N=4 超强可行解搜索（多种子大强度），找到 <=9h 即停止。"""
import sys
import time

sys.path.insert(0, "d:/数模校赛/A 题/code")
import solve_q1 as S

pts, copies = S.load_case("Case3")
dist = S.build_dist(copies)
orig_of = [None] + [c[0] for c in copies]
req = {pid: S.LEVEL_TIMES.get(lvl, 1) for pid, x, y, lvl in pts}

best = float("inf")
best_routes = None
t0 = time.time()
LIMIT = 9.0 * 3600.0

for sd in range(24):
    bt, br = S.solve_vrp_routes(4, dist, orig_of, req, copies,
                                seed=sd * 997 + 3, restarts=10, outer_iters=60,
                                time_cap=LIMIT)
    ok = "合法" if S.check_cover(br, orig_of, req) else "非法"
    print(f"seed#{sd}: Tmax={bt/3600:.4f}h ({ok}) 累计{time.time()-t0:.0f}s",
          flush=True)
    if bt < best and ok == "合法":
        best, best_routes = bt, [r[:] for r in br]
    if bt <= LIMIT + 1e-6:
        print(">>> 找到 9h 内可行解！", flush=True)
        break

print(f"=== 完成：最好 Tmax={best/3600:.4f}h, 耗时 {time.time()-t0:.0f}s ===")
if best_routes is not None:
    ts = [S.route_seconds(r, dist, orig_of) for r in best_routes]
    print("各机时长(h):", [round(t/3600, 4) for t in ts])
