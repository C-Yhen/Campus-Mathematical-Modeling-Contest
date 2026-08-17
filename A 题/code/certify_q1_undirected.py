# -*- coding: utf-8 -*-
"""问题一的紧凑无向欧拉图精确证书模型。

对每架无人机 k、物理点 i：

* ``y[k,i]`` 是无人机 k 对点 i 的有效到达（巡检）次数；
* ``z[k,i]`` 表示该无人机是否访问过 i；
* ``x[k,e]`` 是无向边 e 的实际飞行次数。

点度约束 ``degree(i)=2*y[k,i]``、基地度数等于 2。模型不含自环，因此同架
无人机对同一点执行多次巡检时，必须先离开该点再返回；连续停留不会被重复
计数。这与官方补充规则一致。若所有正度数点均与基地连通，该无向偶图存在
一条基地闭合欧拉回路，故模型与题目规则严格等价。连通性采用单商品有符号
流：每条无向边增加一个连续流变量，
每个被访问点消耗一个单位流量。任一基地外正度分量都会违反流量守恒，
因而所有巡检点必定与基地连通。

若某一轮的松弛模型已被证明不可行，则原问题也不可行；若所得整数图全部
与基地连通，则可用 Hierholzer 算法直接输出真实路线证书。
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import numpy as np
from scipy.optimize import Bounds, milp

from certify_q1 import (
    SERVICE_S,
    SPEED_MS,
    TIME_LIMIT_S,
    CaseData,
    _Rows,
    _distance_data,
    load_case,
)


@dataclass(frozen=True)
class UndirectedCertificate:
    case: str
    n_uav: int
    conclusion: str
    rounds: int
    cuts: int
    status: int
    message: str
    elapsed_s: float
    route_hours: tuple[float, ...] | None = None
    routes: tuple[tuple[int, ...], ...] | None = None


def _components(active: set[int], edges: list[tuple[int, int]], values: np.ndarray):
    adjacency = {node: set() for node in active | {0}}
    for (i, j), value in zip(edges, values):
        if value > 0.5:
            adjacency.setdefault(i, set()).add(j)
            adjacency.setdefault(j, set()).add(i)
    unseen = set(adjacency)
    result = []
    while unseen:
        root = next(iter(unseen))
        stack = [root]
        component = set()
        while stack:
            node = stack.pop()
            if node in component:
                continue
            component.add(node)
            stack.extend(adjacency.get(node, ()))
        unseen -= component
        result.append(component)
    return result


def _euler_route(
    case: CaseData,
    edges: list[tuple[int, int]],
    values: np.ndarray,
    assigned_visits: np.ndarray,
) -> tuple[tuple[int, ...], float]:
    """把一个连通无向偶图还原成欧拉回路，并计算工时。"""
    adjacency: list[list[int]] = [[] for _ in range(case.point_count + 1)]
    edge_id = 0
    total_distance = 0.0
    point_dist, depot_dist = _distance_data(case)
    for (i, j), raw in zip(edges, values):
        count = int(round(float(raw)))
        if count <= 0:
            continue
        distance = (
            depot_dist[j - 1]
            if i == 0
            else depot_dist[i - 1]
            if j == 0
            else point_dist[i - 1, j - 1]
        )
        total_distance += count * distance
        for _ in range(count):
            adjacency[i].append((j, edge_id))
            adjacency[j].append((i, edge_id))
            edge_id += 1

    used = np.zeros(edge_id, dtype=bool)
    stack = [0]
    circuit: list[int] = []
    while stack:
        node = stack[-1]
        while adjacency[node] and used[adjacency[node][-1][1]]:
            adjacency[node].pop()
        if adjacency[node]:
            nxt, eid = adjacency[node].pop()
            if not used[eid]:
                used[eid] = True
                stack.append(nxt)
        else:
            circuit.append(stack.pop())
    circuit.reverse()
    if edge_id == 0 or not used.all() or circuit[0] != 0 or circuit[-1] != 0:
        raise AssertionError("整数图未能还原为单条基地欧拉回路")
    visits = int(round(float(assigned_visits.sum())))
    seconds = visits * SERVICE_S + total_distance / SPEED_MS
    # 欧拉回路每到达物理点一次即完成一次巡检；模型无自环，所以同一点的
    # 多次巡检一定被至少一个其他点（或基地）隔开。
    expanded = [0]
    for node in circuit[1:]:
        if node == 0:
            expanded.append(0)
        else:
            expanded.append(int(case.ids[node - 1]))
    realized = np.zeros(case.point_count, dtype=int)
    id_to_pos = {int(pid): pos for pos, pid in enumerate(case.ids)}
    for pid in expanded:
        if pid != 0:
            realized[id_to_pos[pid]] += 1
    if not np.array_equal(realized, np.rint(assigned_visits).astype(int)):
        raise AssertionError("欧拉回路到达次数与分配巡检次数不一致")
    route_ids = tuple(expanded)
    return route_ids, seconds / 3600.0


def solve_undirected_exact(
    case: CaseData,
    n_uav: int,
    *,
    time_limit: float = 1800.0,
    mip_rel_gap: float = 0.0,
) -> UndirectedCertificate:
    p = case.point_count
    nodes = range(p + 1)
    edges = [(i, j) for i in nodes for j in range(i + 1, p + 1)]
    incident = [[] for _ in nodes]
    for e, (i, j) in enumerate(edges):
        incident[i].append(e)
        incident[j].append(e)
    e_count = len(edges)

    x_count = n_uav * e_count
    y_start = x_count
    y_count = n_uav * p
    z_start = y_start + y_count
    z_count = n_uav * p
    q_start = z_start + z_count
    q_count = n_uav * e_count
    u_start = q_start + q_count
    nvars = u_start + n_uav

    def x_idx(k: int, e: int) -> int:
        return k * e_count + e

    def y_idx(k: int, i: int) -> int:
        return y_start + k * p + (i - 1)

    def z_idx(k: int, i: int) -> int:
        return z_start + k * p + (i - 1)

    def q_idx(k: int, e: int) -> int:
        return q_start + k * e_count + e

    def u_idx(k: int) -> int:
        return u_start + k

    objective = np.zeros(nvars)
    lower = np.zeros(nvars)
    upper = np.zeros(nvars)
    integrality = np.ones(nvars, dtype=np.uint8)
    lower[q_start:u_start] = -p
    upper[q_start:u_start] = p
    integrality[q_start:u_start] = 0
    upper[u_start:] = 1
    point_dist, depot_dist = _distance_data(case)
    edge_distance = np.empty(e_count)
    for e, (i, j) in enumerate(edges):
        if i == 0:
            distance = depot_dist[j - 1]
            edge_upper = 2
        else:
            distance = point_dist[i - 1, j - 1]
            # 节点 i 的总度数为 2*y_i，故任一内部边的安全紧上界为两端
            # 最大可用度数中的较小者。需求至多为 3，本题上界不超过 6。
            edge_upper = 2 * min(
                int(case.demand[i - 1]), int(case.demand[j - 1])
            )
        edge_distance[e] = distance
        for k in range(n_uav):
            upper[x_idx(k, e)] = edge_upper
            # 在 9 h 可行域内最小化总飞行时间。该目标不改变可行性结论，
            # 但相比全零目标能为 HiGHS 的舍入、局部搜索和分支提供明确方向。
            objective[x_idx(k, e)] = distance / SPEED_MS / 3600.0

    for k in range(n_uav):
        for i in range(1, p + 1):
            upper[y_idx(k, i)] = int(case.demand[i - 1])
            upper[z_idx(k, i)] = 1

    base_rows = _Rows()
    # 每个物理点的总巡检次数。
    for i in range(1, p + 1):
        demand = int(case.demand[i - 1])
        base_rows.add(((y_idx(k, i), 1.0) for k in range(n_uav)), demand, demand)

    time_terms: list[list[tuple[int, float]]] = []
    for k in range(n_uav):
        # 启用时基地度数为 2；允许至多 N 架，未启用无人机的图为空。
        base_rows.add(
            [(x_idx(k, e), 1.0) for e in incident[0]] + [(u_idx(k), -2.0)],
            0.0,
            0.0,
        )
        # 每次有效到达贡献两个边端点。无自环保证同一点的多次到达不能连续，
        # 从而严格落实“连续停留只算一次巡检”的补充规则。
        for i in range(1, p + 1):
            base_rows.add(
                [(x_idx(k, e), 1.0) for e in incident[i]]
                + [(y_idx(k, i), -2.0)],
                0.0,
                0.0,
            )
            demand = int(case.demand[i - 1])
            base_rows.add([(y_idx(k, i), 1.0), (z_idx(k, i), -1.0)], 0.0, np.inf)
            base_rows.add(
                [(y_idx(k, i), 1.0), (z_idx(k, i), -demand)], -np.inf, 0.0
            )
            base_rows.add([(z_idx(k, i), 1.0), (u_idx(k), -1.0)], -np.inf, 0.0)

        # 启用的无人机至少执行一次巡检。
        base_rows.add(
            [(y_idx(k, i), 1.0) for i in range(1, p + 1)] + [(u_idx(k), -1.0)],
            0.0,
            np.inf,
        )

        # 单商品有符号流：q>0 表示沿边 (i,j), i<j，从 i 流向 j。
        # |q_e| <= p*x_e；每个访问点消耗 z_i 单位流。
        for e in range(e_count):
            base_rows.add(
                [(q_idx(k, e), 1.0), (x_idx(k, e), -p)], -np.inf, 0.0
            )
            base_rows.add(
                [(q_idx(k, e), -1.0), (x_idx(k, e), -p)], -np.inf, 0.0
            )
        for i in range(1, p + 1):
            balance: list[tuple[int, float]] = [(z_idx(k, i), -1.0)]
            for e in incident[i]:
                left, right = edges[e]
                # 流入为正、流出为负。
                balance.append((q_idx(k, e), 1.0 if right == i else -1.0))
            base_rows.add(balance, 0.0, 0.0)

        terms = [(x_idx(k, e), edge_distance[e] / SPEED_MS) for e in range(e_count)]
        terms.extend((y_idx(k, i), SERVICE_S) for i in range(1, p + 1))
        time_terms.append(terms)
        base_rows.add(terms, -np.inf, TIME_LIMIT_S)

        # 若访问 i，闭合路线至少承担一次基地-i往返；服务项使用真实总次数。
        service = [(y_idx(k, h), SERVICE_S) for h in range(1, p + 1)]
        for i in range(1, p + 1):
            radial_s = 2.0 * depot_dist[i - 1] / SPEED_MS
            base_rows.add(service + [(z_idx(k, i), radial_s)], -np.inf, TIME_LIMIT_S)

        # 同时访问 i,j 时，飞行距离不少于最短三角闭合路程 0-i-j-0。
        # 仅保留会实际收紧 108 次纯服务上限的远点对，控制模型规模。
        for i in range(1, p + 1):
            for j in range(i + 1, p + 1):
                cycle_s = (
                    depot_dist[i - 1]
                    + point_dist[i - 1, j - 1]
                    + depot_dist[j - 1]
                ) / SPEED_MS
                if cycle_s > 0.35 * 3600.0:
                    base_rows.add(
                        service
                        + [(z_idx(k, i), cycle_s), (z_idx(k, j), cycle_s)],
                        -np.inf,
                        TIME_LIMIT_S + cycle_s,
                    )

    # 按总巡检次数非增序排列同质无人机，消除 4! 标签对称性。
    for k in range(n_uav - 1):
        base_rows.add([(u_idx(k), 1.0), (u_idx(k + 1), -1.0)], 0.0, np.inf)
        terms = [(y_idx(k, i), 1.0) for i in range(1, p + 1)]
        terms.extend((y_idx(k + 1, i), -1.0) for i in range(1, p + 1))
        base_rows.add(terms, 0.0, np.inf)

    started = time.monotonic()
    print(
        f"\nFlow-MIP: rows={len(base_rows.lb)}, vars={nvars}, "
        f"integer={q_start + n_uav}"
    )
    result = milp(
        c=objective,
        integrality=integrality,
        bounds=Bounds(lower, upper),
        constraints=base_rows.constraint(nvars),
        options={
            "disp": True,
            "time_limit": float(time_limit),
            "mip_rel_gap": float(mip_rel_gap),
            "presolve": True,
        },
    )
    elapsed = time.monotonic() - started
    if result.status == 2:
        return UndirectedCertificate(
            case.name, n_uav, "infeasible", 1, 0, int(result.status), str(result.message), elapsed
        )
    if result.x is None:
        return UndirectedCertificate(
            case.name, n_uav, "unknown", 1, 0, int(result.status), str(result.message), elapsed
        )

    routes = []
    hours = []
    for k in range(n_uav):
        if result.x[u_idx(k)] < 0.5:
            continue
        y_values = result.x[y_start + k * p : y_start + (k + 1) * p]
        active = {i + 1 for i, value in enumerate(y_values) if value > 0.5}
        x_values = result.x[k * e_count : (k + 1) * e_count]
        components = _components(active, edges, x_values)
        if any(0 not in component for component in components if component - {0}):
            return UndirectedCertificate(
                case.name,
                n_uav,
                "unknown",
                1,
                0,
                int(result.status),
                str(result.message) + "; 数值解未通过连通性审计",
                elapsed,
            )
        route, route_h = _euler_route(case, edges, x_values, y_values)
        routes.append(route)
        hours.append(route_h)
    return UndirectedCertificate(
        case.name,
        n_uav,
        "feasible",
        1,
        0,
        int(result.status),
        str(result.message),
        elapsed,
        tuple(hours),
        tuple(routes),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3", choices=[f"Case{i}" for i in range(1, 5)])
    parser.add_argument("--n", type=int, required=True)
    parser.add_argument("--time-limit", type=float, default=1800.0)
    parser.add_argument("--mip-gap", type=float, default=0.0)
    args = parser.parse_args()
    cert = solve_undirected_exact(
        load_case(args.case),
        args.n,
        time_limit=args.time_limit,
        mip_rel_gap=args.mip_gap,
    )
    print("\n", cert)
    if cert.conclusion == "infeasible":
        print(f"严格证书：{args.case} 在 N={args.n}、单机 9 h 下不可行。")
    elif cert.conclusion == "feasible":
        print(f"严格证书：{args.case} 在 N={args.n}、单机 9 h 下可行。")
        for k, (hours, route) in enumerate(zip(cert.route_hours, cert.routes), 1):
            print(f"UAV{k} ({hours:.6f} h): " + " -> ".join(map(str, route)))
    else:
        print("给定时间内未形成严格结论。")


if __name__ == "__main__":
    main()