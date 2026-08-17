# -*- coding: utf-8 -*-
"""把聚合最优弧多重集严格划分为若干条负载均衡的基地闭合路线。

``certify_q1.py --aggregate`` 求得的是若干条路线合计工时的全局最优弧流，
但任意一次欧拉分解可能极不均衡。本脚本固定该最优弧多重集，用一个较小的
混合整数规划把每条弧分配给 ``N`` 架无人机：

* 每条聚合弧按原重数恰好分配一次；
* 每架无人机在每个物理点满足入度等于出度；
* 每架无人机恰好从基地出发一次、返回一次；
* 单商品流保证每架无人机的正弧分量均与基地连通；
* 目标是最小化最大逐机工时，并可施加逐机硬上限。

因此，每份弧集都是与基地连通的有向欧拉多重图，可由 Hierholzer 算法严格
还原为一条不含中途返航的闭合路线。输出 JSON 与 ``search_q1_ortools.py`` 的
热启动格式兼容。
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import os
from collections import Counter
from dataclasses import asdict, dataclass

import numpy as np
from scipy.optimize import Bounds, milp

from certify_q1 import SERVICE_S, SPEED_MS, _Rows, _distance_data, load_case


@dataclass(frozen=True)
class PartitionCertificate:
    case: str
    n_uav: int
    conclusion: str
    capacity_hours: float
    status: int
    message: str
    objective_max_hours: float | None
    dual_bound_hours: float | None
    mip_gap: float | None
    node_count: int | None
    total_hours: float | None
    max_hours: float | None
    route_hours: tuple[float, ...] | None
    routes: tuple[tuple[int, ...], ...] | None


def _read_text(path: str) -> str:
    raw = open(path, "rb").read()
    if raw.startswith((b"\xff\xfe", b"\xfe\xff")):
        return raw.decode("utf-16")
    text = raw.decode("utf-8", errors="replace")
    return raw.decode("utf-16-le", errors="replace") if "\x00" in text else text


def _load_aggregate_routes(path: str) -> tuple[tuple[int, ...], ...]:
    """从 ``TotalTimeCertificate`` 日志提取带基地 0 的欧拉分解路线。"""
    line = next(
        (row for row in _read_text(path).splitlines() if "best_routes=" in row),
        None,
    )
    if line is None:
        raise ValueError(f"聚合证书没有 best_routes: {path}")
    text = line.split("best_routes=", 1)[1].rstrip()
    # 文本结尾的最后一个 ')' 属于 TotalTimeCertificate，而非 best_routes 元组。
    routes = ast.literal_eval(text[:-1])
    parsed = tuple(tuple(int(node) for node in route) for route in routes)
    if not parsed or any(len(route) < 2 or route[0] != 0 or route[-1] != 0 for route in parsed):
        raise ValueError("聚合证书路线必须均从基地0出发并返回")
    return parsed


def _aggregate_arcs(
    physical_routes: tuple[tuple[int, ...], ...], id_to_node: dict[int, int]
) -> tuple[list[tuple[int, int]], np.ndarray]:
    counts: Counter[tuple[int, int]] = Counter()
    for route in physical_routes:
        nodes = [0 if pid == 0 else id_to_node[pid] for pid in route]
        counts.update(zip(nodes, nodes[1:]))
    if any(i == j for i, j in counts):
        raise ValueError("聚合证书含自环/相邻同点，违反巡检计数规则")
    arcs = sorted(counts)
    return arcs, np.asarray([counts[arc] for arc in arcs], dtype=int)


def _euler_route(
    point_count: int, arcs: list[tuple[int, int]], multiplicity: np.ndarray
) -> list[int] | None:
    adjacency: list[list[int]] = [[] for _ in range(point_count + 1)]
    edge_count = 0
    for (i, j), count in zip(arcs, multiplicity):
        count = int(count)
        if count > 0:
            adjacency[i].extend([j] * count)
            edge_count += count
    # 固定排序使证书可复现；连通平衡图的任一邻接顺序都产生完整欧拉回路。
    for neighbors in adjacency:
        neighbors.sort(reverse=True)
    stack = [0]
    reverse_circuit: list[int] = []
    while stack:
        node = stack[-1]
        if adjacency[node]:
            stack.append(adjacency[node].pop())
        else:
            reverse_circuit.append(stack.pop())
    route = reverse_circuit[::-1]
    if len(route) != edge_count + 1 or route[0] != 0 or route[-1] != 0:
        return None
    if 0 in route[1:-1]:
        return None
    return route


def solve(
    case_name: str,
    n_uav: int,
    aggregate_path: str,
    *,
    capacity_hours: float,
    time_limit: float,
    mip_gap: float,
) -> PartitionCertificate:
    case = load_case(case_name)
    aggregate_routes = _load_aggregate_routes(aggregate_path)
    id_to_node = {int(pid): i + 1 for i, pid in enumerate(case.ids)}
    arcs, aggregate_multiplicity = _aggregate_arcs(aggregate_routes, id_to_node)
    arc_pos = {arc: a for a, arc in enumerate(arcs)}
    a_count = len(arcs)
    p = case.point_count

    # y[k,a] 为车辆k取得弧a的整数重数；f[k,a] 为连通性单商品流；最后为Tmax。
    y_count = n_uav * a_count
    f_start = y_count
    t_index = 2 * y_count
    nvars = t_index + 1

    def y(k: int, a: int) -> int:
        return k * a_count + a

    def f(k: int, a: int) -> int:
        return f_start + k * a_count + a

    point_dist, depot_dist = _distance_data(case)
    arc_hours = np.empty(a_count, dtype=float)
    for a, (i, j) in enumerate(arcs):
        if i == 0:
            distance = depot_dist[j - 1]
        elif j == 0:
            distance = depot_dist[i - 1]
        else:
            distance = point_dist[i - 1, j - 1]
        arc_hours[a] = distance / SPEED_MS / 3600.0 + (
            SERVICE_S / 3600.0 if j != 0 else 0.0
        )

    objective = np.zeros(nvars)
    objective[t_index] = 1.0
    lower = np.zeros(nvars)
    upper = np.zeros(nvars)
    integrality = np.zeros(nvars, dtype=np.uint8)
    integrality[:y_count] = 1
    for k in range(n_uav):
        for a, count in enumerate(aggregate_multiplicity):
            upper[y(k, a)] = int(count)
            upper[f(k, a)] = case.visit_count
    upper[t_index] = float(capacity_hours)

    rows = _Rows()

    # 每条聚合最优弧按原重数恰好分配一次。
    for a, count in enumerate(aggregate_multiplicity):
        rows.add(((y(k, a), 1.0) for k in range(n_uav)), int(count), int(count))

    outgoing: list[list[int]] = [[] for _ in range(p + 1)]
    incoming: list[list[int]] = [[] for _ in range(p + 1)]
    for a, (i, j) in enumerate(arcs):
        outgoing[i].append(a)
        incoming[j].append(a)

    big_m = case.visit_count
    for k in range(n_uav):
        # 恰好一次离开/回到基地，从而还原路线不含中途返航。
        rows.add(((y(k, a), 1.0) for a in outgoing[0]), 1.0, 1.0)
        rows.add(((y(k, a), 1.0) for a in incoming[0]), 1.0, 1.0)

        # 每个任务点的有向弧平衡。
        for node in range(1, p + 1):
            terms = [(y(k, a), 1.0) for a in incoming[node]]
            terms.extend((y(k, a), -1.0) for a in outgoing[node])
            rows.add(terms, 0.0, 0.0)

        # f<=M*y；每个被分配的入弧在其终点消费一个流单位，排除分离子环。
        for a in range(a_count):
            rows.add([(f(k, a), 1.0), (y(k, a), -big_m)], -np.inf, 0.0)
        for node in range(1, p + 1):
            terms = [(f(k, a), 1.0) for a in incoming[node]]
            terms.extend((f(k, a), -1.0) for a in outgoing[node])
            terms.extend((y(k, a), -1.0) for a in incoming[node])
            rows.add(terms, 0.0, 0.0)

        # 逐机工时不超过Tmax，且Tmax的变量上界即指定硬容量。
        terms = [(y(k, a), float(arc_hours[a])) for a in range(a_count)]
        terms.append((t_index, -1.0))
        rows.add(terms, -np.inf, 0.0)

    # 三条 0->43 弧对应的车辆仍对称；按相邻车辆的弧编码字典序弱排序会引入
    # 大量约束，收益有限。仅固定唯一的基地出弧类型（若存在）给车辆0即可消除
    # 其与其余车辆的对称性，且不损失一般性。
    depot_types = [a for a in outgoing[0] if aggregate_multiplicity[a] == 1]
    if depot_types:
        rows.add([(y(0, depot_types[0]), 1.0)], 1.0, 1.0)

    result = milp(
        c=objective,
        integrality=integrality,
        bounds=Bounds(lower, upper),
        constraints=rows.constraint(nvars),
        options={
            "disp": True,
            "time_limit": float(time_limit),
            "mip_rel_gap": float(mip_gap),
            "presolve": True,
        },
    )

    objective_h = None if result.fun is None or not np.isfinite(result.fun) else float(result.fun)
    raw_bound = getattr(result, "mip_dual_bound", None)
    dual_bound_h = None if raw_bound is None or not np.isfinite(raw_bound) else float(raw_bound)
    mip_gap_value = getattr(result, "mip_gap", None)
    node_count = getattr(result, "mip_node_count", None)

    physical_routes: tuple[tuple[int, ...], ...] | None = None
    route_hours: tuple[float, ...] | None = None
    conclusion = "unknown"
    if result.x is not None:
        assigned = np.rint(np.asarray(result.x[:y_count])).astype(int).reshape(n_uav, a_count)
        node_routes = [_euler_route(p, arcs, assigned[k]) for k in range(n_uav)]
        if all(route is not None for route in node_routes):
            complete_routes = [route for route in node_routes if route is not None]
            physical_routes = tuple(
                tuple(int(case.ids[node - 1]) for node in route[1:-1])
                for route in complete_routes
            )
            route_hours = tuple(
                float(sum(arc_hours[a] * assigned[k, a] for a in range(a_count)))
                for k in range(n_uav)
            )

            # 与优化状态独立的证书审计：弧重数、覆盖次数、相邻同点和逐机工时。
            recovered = Counter()
            realized = Counter()
            no_repeat = True
            for route in complete_routes:
                recovered.update(zip(route, route[1:]))
                ids = [int(case.ids[node - 1]) for node in route[1:-1]]
                realized.update(ids)
                no_repeat &= all(a != b for a, b in zip(ids, ids[1:]))
            expected_arcs = Counter({arc: int(count) for arc, count in zip(arcs, aggregate_multiplicity)})
            expected_visits = Counter(
                {int(pid): int(demand) for pid, demand in zip(case.ids, case.demand)}
            )
            audit_ok = (
                recovered == expected_arcs
                and realized == expected_visits
                and no_repeat
                and max(route_hours) <= capacity_hours + 1e-8
            )
            conclusion = "feasible" if audit_ok else "audit_failed"

    return PartitionCertificate(
        case=case.name,
        n_uav=n_uav,
        conclusion=conclusion,
        capacity_hours=float(capacity_hours),
        status=int(result.status),
        message=str(result.message),
        objective_max_hours=objective_h,
        dual_bound_hours=dual_bound_h,
        mip_gap=None if mip_gap_value is None else float(mip_gap_value),
        node_count=None if node_count is None else int(node_count),
        total_hours=None if route_hours is None else float(sum(route_hours)),
        max_hours=None if route_hours is None else float(max(route_hours)),
        route_hours=route_hours,
        routes=physical_routes,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3")
    parser.add_argument("--n", type=int, default=4)
    parser.add_argument(
        "--aggregate",
        default=os.path.join(os.path.dirname(__file__), "..", "docs", "aggregate_case3_n4_300s.txt"),
    )
    parser.add_argument("--capacity-hours", type=float, default=9.0)
    parser.add_argument("--time-limit", type=float, default=300.0)
    parser.add_argument("--mip-gap", type=float, default=0.0)
    parser.add_argument("--output")
    args = parser.parse_args()

    cert = solve(
        args.case,
        args.n,
        args.aggregate,
        capacity_hours=args.capacity_hours,
        time_limit=args.time_limit,
        mip_gap=args.mip_gap,
    )
    print("\n", cert)
    if cert.routes is not None and cert.route_hours is not None:
        for k, (hours, route) in enumerate(zip(cert.route_hours, cert.routes), 1):
            print(f"UAV{k} ({hours:.6f} h): 0 -> " + " -> ".join(map(str, route)) + " -> 0")
    if args.output:
        output = os.path.abspath(args.output)
        with open(output, "w", encoding="utf-8") as file:
            json.dump(asdict(cert), file, ensure_ascii=False, indent=2)
        print(f"证书已写入 {output}")


if __name__ == "__main__":
    main()