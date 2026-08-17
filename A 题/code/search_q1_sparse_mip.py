# -*- coding: utf-8 -*-
"""在稀疏候选边图上搜索问题一的严格可行路线证书。

完整无向欧拉模型包含 O(NP^2) 条边变量。对 Case3 的 100 个点，完整模型
在 HiGHS 中仅根 LP 就可能耗时数分钟。本程序保留全部“基地--巡检点”边，
并只保留每个巡检点的若干条最近邻内部边，从而构造一个受限但仍严格的模型：

* 若模型给出可行解，欧拉分解得到的路线对原问题也严格可行；
* 若受限模型不可行，不能据此断言原问题不可行，只能增加候选邻边后再试。

模型直接令每个访问点消耗 ``y[k,i]`` 单位连通流，因此无需额外的访问二元
变量。点度约束 ``degree(i)=2*y[k,i]`` 且不含自环，保证同一无人机对同一点
的多次巡检之间必定先离开该点，符合“连续停留只计一次”的补充规则。

示例：
    python search_q1_sparse_mip.py --case Case3 --n 4 --neighbors 12 --time-limit 600
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import numpy as np
from scipy.optimize import Bounds, milp

from certify_q1 import CaseData, SERVICE_S, SPEED_MS, _Rows, _distance_data, load_case
from certify_q1_undirected import _components, _euler_route


@dataclass(frozen=True)
class SparseCertificate:
    case: str
    n_uav: int
    neighbors: int
    candidate_edges: int
    conclusion: str
    status: int
    message: str
    elapsed_s: float
    objective_hours: float | None = None
    route_hours: tuple[float, ...] | None = None
    routes: tuple[tuple[int, ...], ...] | None = None


def _candidate_edges(case: CaseData, neighbors: int) -> list[tuple[int, int]]:
    """返回全部基地边与对称 k 近邻内部边；``neighbors<=0`` 表示完整图。"""
    p = case.point_count
    edges = {(0, i) for i in range(1, p + 1)}
    if neighbors <= 0 or neighbors >= p - 1:
        edges.update((i, j) for i in range(1, p + 1) for j in range(i + 1, p + 1))
        return sorted(edges)

    point_dist, _ = _distance_data(case)
    k = min(neighbors, p - 1)
    for a in range(p):
        order = np.argsort(point_dist[a])
        kept = [int(b) for b in order if int(b) != a][:k]
        for b in kept:
            i, j = sorted((a + 1, b + 1))
            edges.add((i, j))
    return sorted(edges)


def solve_sparse(
    case: CaseData,
    n_uav: int,
    *,
    neighbors: int = 12,
    time_limit: float = 600.0,
    mip_rel_gap: float = 0.0,
    minimize_makespan: bool = False,
) -> SparseCertificate:
    p = case.point_count
    edges = _candidate_edges(case, neighbors)
    e_count = len(edges)
    incident: list[list[int]] = [[] for _ in range(p + 1)]
    for e, (i, j) in enumerate(edges):
        incident[i].append(e)
        incident[j].append(e)

    x_count = n_uav * e_count
    y_start = x_count
    y_count = n_uav * p
    q_start = y_start + y_count
    q_count = n_uav * e_count
    makespan_idx = q_start + q_count if minimize_makespan else None
    nvars = q_start + q_count + int(minimize_makespan)

    def x_idx(k: int, e: int) -> int:
        return k * e_count + e

    def y_idx(k: int, i: int) -> int:
        return y_start + k * p + (i - 1)

    def q_idx(k: int, e: int) -> int:
        return q_start + k * e_count + e

    objective = np.zeros(nvars)
    lower = np.zeros(nvars)
    upper = np.zeros(nvars)
    integrality = np.ones(nvars, dtype=np.uint8)
    total_visits = int(case.demand.sum())
    lower[q_start:] = -total_visits
    upper[q_start:] = total_visits
    integrality[q_start:] = 0
    if makespan_idx is not None:
        upper[makespan_idx] = 100.0

    point_dist, depot_dist = _distance_data(case)
    edge_hours = np.empty(e_count)
    for e, (i, j) in enumerate(edges):
        if i == 0:
            distance = depot_dist[j - 1]
            edge_upper = 2
        else:
            distance = point_dist[i - 1, j - 1]
            edge_upper = 2 * min(int(case.demand[i - 1]), int(case.demand[j - 1]))
        edge_hours[e] = distance / SPEED_MS / 3600.0
        for k in range(n_uav):
            upper[x_idx(k, e)] = edge_upper
            # 最小最大工时模式中，以极小总工时权重打破相同 Tmax 的退化解。
            objective[x_idx(k, e)] = edge_hours[e] * (1e-4 if minimize_makespan else 1.0)

    service_hours = SERVICE_S / 3600.0
    for k in range(n_uav):
        for i in range(1, p + 1):
            upper[y_idx(k, i)] = int(case.demand[i - 1])
            objective[y_idx(k, i)] = service_hours * (1e-4 if minimize_makespan else 1.0)
    if makespan_idx is not None:
        objective[makespan_idx] = 1.0

    rows = _Rows()
    # 所有无人机合计恰好完成每个点的规定巡检次数。
    for i in range(1, p + 1):
        demand = int(case.demand[i - 1])
        rows.add(((y_idx(k, i), 1.0) for k in range(n_uav)), demand, demand)

    route_time_terms: list[list[tuple[int, float]]] = []
    for k in range(n_uav):
        # Case3,N=4 的 140 次、每次 5 min 服务决定四架均必须启用；固定基地
        # 度数为 2 可去掉启用变量，并消除无意义的多次返航。
        rows.add(((x_idx(k, e), 1.0) for e in incident[0]), 2.0, 2.0)

        for i in range(1, p + 1):
            rows.add(
                [(x_idx(k, e), 1.0) for e in incident[i]]
                + [(y_idx(k, i), -2.0)],
                0.0,
                0.0,
            )

        # 每次巡检消耗一个流量单位。若 y[k,i]>0，则 i 必须经正 x 边与基地
        # 连通；否则该分量没有流量来源，无法满足平衡式。
        for e in range(e_count):
            rows.add(
                [(q_idx(k, e), 1.0), (x_idx(k, e), -total_visits)],
                -np.inf,
                0.0,
            )
            rows.add(
                [(q_idx(k, e), -1.0), (x_idx(k, e), -total_visits)],
                -np.inf,
                0.0,
            )
        for i in range(1, p + 1):
            balance: list[tuple[int, float]] = [(y_idx(k, i), -1.0)]
            for e in incident[i]:
                left, right = edges[e]
                balance.append((q_idx(k, e), 1.0 if right == i else -1.0))
            rows.add(balance, 0.0, 0.0)

        terms = [(x_idx(k, e), edge_hours[e]) for e in range(e_count)]
        terms.extend((y_idx(k, i), service_hours) for i in range(1, p + 1))
        route_time_terms.append(terms)
        if makespan_idx is None:
            rows.add(terms, -np.inf, 9.0)
        else:
            rows.add(terms + [(makespan_idx, -1.0)], -np.inf, 0.0)

    # 同质无人机按工时非增序排列；任意可行解都能重标号满足该约束。
    for k in range(n_uav - 1):
        ordered = route_time_terms[k] + [(idx, -value) for idx, value in route_time_terms[k + 1]]
        rows.add(ordered, 0.0, np.inf)

    started = time.monotonic()
    print(
        f"Sparse-MIP: neighbors={neighbors}, edges={e_count}, rows={len(rows.lb)}, "
        f"vars={nvars}, integer={q_start}"
    )
    result = milp(
        c=objective,
        integrality=integrality,
        bounds=Bounds(lower, upper),
        constraints=rows.constraint(nvars),
        options={
            "disp": True,
            "time_limit": float(time_limit),
            "mip_rel_gap": float(mip_rel_gap),
            "presolve": True,
        },
    )
    elapsed = time.monotonic() - started
    common = dict(
        case=case.name,
        n_uav=n_uav,
        neighbors=neighbors,
        candidate_edges=e_count,
        status=int(result.status),
        message=str(result.message),
        elapsed_s=elapsed,
    )
    if result.x is None:
        # 候选图是原问题的限制；即使被证明不可行，也只能报告“受限图不可行”。
        conclusion = "restricted_infeasible" if result.status == 2 else "unknown"
        return SparseCertificate(conclusion=conclusion, **common)

    routes: list[tuple[int, ...]] = []
    hours: list[float] = []
    for k in range(n_uav):
        y_values = result.x[y_start + k * p : y_start + (k + 1) * p]
        x_values = result.x[k * e_count : (k + 1) * e_count]
        active = {i + 1 for i, value in enumerate(y_values) if value > 0.5}
        components = _components(active, edges, x_values)
        if any(0 not in component for component in components if component - {0}):
            return SparseCertificate(conclusion="numerical_audit_failed", **common)
        route, route_h = _euler_route(case, edges, x_values, y_values)
        if not minimize_makespan and route_h > 9.0 + 1e-6:
            return SparseCertificate(conclusion="numerical_audit_failed", **common)
        routes.append(route)
        hours.append(route_h)

    # 全局覆盖次数终审，避免仅依赖求解器容差。
    realized = {int(pid): 0 for pid in case.ids}
    for route in routes:
        for pid in route:
            if pid:
                realized[pid] += 1
    expected = {int(pid): int(d) for pid, d in zip(case.ids, case.demand)}
    if realized != expected:
        return SparseCertificate(conclusion="numerical_audit_failed", **common)

    conclusion = "feasible" if max(hours) <= 9.0 + 1e-6 else "candidate"
    return SparseCertificate(
        conclusion=conclusion,
        objective_hours=float(result.x[makespan_idx]) if makespan_idx is not None else float(result.fun),
        route_hours=tuple(hours),
        routes=tuple(routes),
        **common,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3", choices=[f"Case{i}" for i in range(1, 5)])
    parser.add_argument("--n", type=int, required=True)
    parser.add_argument("--neighbors", type=int, default=12)
    parser.add_argument("--time-limit", type=float, default=600.0)
    parser.add_argument("--mip-gap", type=float, default=0.0)
    parser.add_argument(
        "--minimax",
        action="store_true",
        help="取消 9 h 硬上限并直接最小化最大路线工时",
    )
    args = parser.parse_args()
    cert = solve_sparse(
        load_case(args.case),
        args.n,
        neighbors=args.neighbors,
        time_limit=args.time_limit,
        mip_rel_gap=args.mip_gap,
        minimize_makespan=args.minimax,
    )
    print("\n", cert)
    if cert.conclusion == "feasible":
        print(f"严格可行证书：{args.case} 在 N={args.n}、单机 9 h 下可行。")
        assert cert.route_hours is not None and cert.routes is not None
        for k, (hours, route) in enumerate(zip(cert.route_hours, cert.routes), 1):
            print(f"UAV{k} ({hours:.6f} h): " + " -> ".join(map(str, route)))
    elif cert.conclusion == "candidate":
        print("已得到经过审计的整数候选路线，但最大工时尚未降至 9 h。")
        assert cert.route_hours is not None and cert.routes is not None
        for k, (hours, route) in enumerate(zip(cert.route_hours, cert.routes), 1):
            print(f"UAV{k} ({hours:.6f} h): " + " -> ".join(map(str, route)))
    elif cert.conclusion == "restricted_infeasible":
        print("当前候选边图不可行；这不构成原问题不可行证明，请增加 --neighbors。")
    else:
        print("给定时间内未找到经过审计的严格可行证书。")


if __name__ == "__main__":
    main()