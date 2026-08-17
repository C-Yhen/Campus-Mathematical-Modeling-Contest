# -*- coding: utf-8 -*-
"""Case3 N=4 阶梯式证否：从 cap=10h 逐步收紧到 9h。

每轮用精确连通模型（connected=True）验证"每架 <= cap"是否可行：
  - 求解器证不可行 (status=2)：该 cap 下不存在任何方案 -> min Tmax > cap；
    只要 cap >= 9h，即严格证否 N=4；
  - 求解器找到整数解：说明存在 cap 内方案（上界进一步压低）；
  - 限时未定论：继续下一档（更小 cap 的可行域更小，更易证不可行）。

结果写入 docs/case3_n4_stair_run.txt（UTF-8）。
"""
import os
import sys
import time

sys.path.insert(0, "d:/数模校赛/A 题/code")
import certify_q1 as C

CASE = C.load_case("Case3")
N = 4
CAPS = [10.0, 9.8, 9.6, 9.4, 9.2, 9.0]
ROUND_TIME = 600.0  # 每轮 10 分钟
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(C.__file__))),
                   "docs", "case3_n4_stair_run.txt")


def log(text: str) -> None:
    print(text, flush=True)
    with open(OUT, "a", encoding="utf-8") as f:
        f.write(text + "\n")


def main() -> int:
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(f"Case3 N=4 阶梯式证否（精确连通模型，每轮限时 {ROUND_TIME:.0f}s）\n")
    proven_at = None
    for cap in CAPS:
        t0 = time.time()
        cert = C.solve_minmax_cycle_cover(
            CASE, N,
            time_limit=ROUND_TIME,
            feasibility_only=True,
            connected=True,
            cap_s=cap * 3600.0,
        )
        elapsed = time.time() - t0
        if cert.status == 2:
            proven_at = cap
            log(f"[轮] cap={cap:g}h: 求解器已证明不可行（status=2，用时 {elapsed:.0f}s）")
            log(f">>> 严格证否成立：min Tmax > {cap:g}h"
                + (" > 9h，故 N=4 不可行。" if cap >= 9.0 else "（仍低于 9h，需继续下一档）"))
            if cap >= 9.0:
                break
        elif cert.feasible_under_9h is True:
            log(f"[轮] cap={cap:g}h: 找到 {cap:g}h 内整数解（上界压低，用时 {elapsed:.0f}s）")
        else:
            log(f"[轮] cap={cap:g}h: 限时未定论（用时 {elapsed:.0f}s，继续下一档）")
    if proven_at is None:
        log("阶梯搜索未形成证否；需要更长时限或更强下界。")
    else:
        log(f"证明完成于 cap={proven_at:g}h。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
