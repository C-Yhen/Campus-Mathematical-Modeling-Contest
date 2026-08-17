# -*- coding: utf-8 -*-
"""在有依据的候选弧子图上搜索问题一严格多机路线证书。

候选弧由以下部分组成：全部基地弧、每个物理点的 k 个欧氏近邻弧、聚合低总
工时解中的弧，以及一个或多个已有多机可行解中的弧。模型在该子图上保留完整
的覆盖、逐机流平衡、单商品流连通和逐机工时约束；故找到的解对原问题严格
可行，而“不可行”只表示当前候选弧池不可行。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter
from dataclasses import asdict, dataclass

import numpy as np
from scipy.optimize import Bounds, milp

from certify_q1 import SERVICE_S, SPEED_MS, _Rows, _distance_data, load_case
from partition_q1_aggregate import _euler_route, _load_aggregate_routes, _read_text


@dataclass(frozen=True)
class CandidateCertificate:
    case: str
    n_uav: int
    conclusion: str
    capacity_hours: float
    neighbor_k: int
    arc_count: int
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


def _load_routes(path: str) -> tuple[tuple[int, ...], ...]:
    if path.lower().endswith(".json"):
        with open(path, encoding="utf-8") as file:
            raw_routes = json.load(file)["routes"]
        return tuple(tuple(map(int, route)) for route in raw_routes)
    routes = []
    for row in _read_text(path).splitlines():
        if row.strip().startswith("UAV") and "->" in row:
            chain = row.split(":", 1)[1]
            nodes = [int(token.strip()) for token in chain.split("->")]
            if nodes[0] != 0 or nodes[-1] != 0:
                raise ValueError(f"路线未从基地0出发并返回: {row}")
            routes.append(tuple(nodes[1:-1]))
    if not routes:
        raise ValueError(f"文件中没有UAV路线行: {path}")
    return tuple(routes)


def _add_route_arcs(
    pool: set[tuple[int, int]], routes: tuple[tuple[int, ...], ...], id_to_node: dict[int, int]
) -> None:
    for physical in routes:
        route = [0] + [id_to_node[int(pid)] for pid in physical] + [0]
        pool.update(zip(route, route[1:]))


def solve(
    case_name: str,
    n_uav: int,
    *,
    capacity_hours: float,
    neighbor_k: int,
    aggregate_path: str | None,
    route_paths: list[str],
    warm_start_path: str | None,
    time_limit: float,
    mip_gap: float,
    native_highs: bool,
) -> CandidateCertificate:
    case = load_case(case_name)
    p = case.point_count
    nodes = range(p + 1)
    id_to_node = {int(pid): i + 1 for i, pid in enumerate(case.ids)}
    point_dist, depot_dist = _distance_data(case)

    pool: set[tuple[int, int]] = set()
    # 所有基地弧允许MIP自由选择四条路线的起终点。
    pool.update((0, j) for j in range(1, p + 1))
    pool.update((j, 0) for j in range(1, p + 1))
    if neighbor_k > 0:
        order = np.argsort(point_dist, axis=1)
        for i in range(1, p + 1):
            for raw_j in order[i - 1, : min(neighbor_k, p - 1)]:
                j = int(raw_j) + 1
                if i != j:
                    pool.add((i, j))
                    pool.add((j, i))
    if aggregate_path:
        aggregate = _load_aggregate_routes(aggregate_path)
        for physical in aggregate:
            route = [0 if pid == 0 else id_to_node[int(pid)] for pid in physical]
            pool.update(zip(route, route[1:]))
    for path in route_paths:
        _add_route_arcs(pool, _load_routes(path), id_to_node)
    if warm_start_path:
        # 热启动路线的全部弧必须位于模型中；无需用户在 --routes 中重复指定。
        _add_route_arcs(pool, _load_routes(warm_start_path), id_to_node)

    arcs = sorted(pool)
    arc_pos = {arc: a for a, arc in enumerate(arcs)}
    a_count = len(arcs)
    outgoing: list[list[int]] = [[] for _ in nodes]
    incoming: list[list[int]] = [[] for _ in nodes]
    for a, (i, j) in enumerate(arcs):
        outgoing[i].append(a)
        incoming[j].append(a)

    x_count = n_uav * a_count
    f_start = x_count
    t_index = 2 * x_count
    nvars = t_index + 1

    def x(k: int, a: int) -> int:
        return k * a_count + a

    def f(k: int, a: int) -> int:
        return f_start + k * a_count + a

    arc_seconds = np.empty(a_count)
    for a, (i, j) in enumerate(arcs):
        if i == 0:
            distance = depot_dist[j - 1]
        elif j == 0:
            distance = depot_dist[i - 1]
        else:
            distance = point_dist[i - 1, j - 1]
        arc_seconds[a] = distance / SPEED_MS + (0.0 if j == 0 else SERVICE_S)

    objective = np.zeros(nvars)
    objective[t_index] = 1.0 / 3600.0
    lower = np.zeros(nvars)
    upper = np.zeros(nvars)
    integrality = np.zeros(nvars, dtype=np.uint8)
    integrality[:x_count] = 1
    max_visits = int(capacity_hours * 3600.0 // SERVICE_S)
    for k in range(n_uav):
        for a, (i, j) in enumerate(arcs):
            upper[x(k, a)] = 1 if i == 0 or j == 0 else min(
                int(case.demand[i - 1]), int(case.demand[j - 1])
            )
            upper[f(k, a)] = max_visits
    upper[t_index] = capacity_hours * 3600.0

    rows = _Rows()
    # 每个物理点的总到达次数精确等于巡检需求。
    for j in range(1, p + 1):
        rows.add(
            ((x(k, a), 1.0) for k in range(n_uav) for a in incoming[j]),
            int(case.demand[j - 1]),
            int(case.demand[j - 1]),
        )

    time_terms: list[list[tuple[int, float]]] = []
    for k in range(n_uav):
        for node in range(1, p + 1):
            terms = [(x(k, a), 1.0) for a in incoming[node]]
            terms.extend((x(k, a), -1.0) for a in outgoing[node])
            rows.add(terms, 0.0, 0.0)
        rows.add(((x(k, a), 1.0) for a in outgoing[0]), 1.0, 1.0)
        rows.add(((x(k, a), 1.0) for a in incoming[0]), 1.0, 1.0)

        # 单商品流：每个任务到达消费一个单位，且流仅能沿已使用弧传递。
        for a in range(a_count):
            rows.add([(f(k, a), 1.0), (x(k, a), -max_visits)], -np.inf, 0.0)
        for node in range(1, p + 1):
            terms = [(f(k, a), 1.0) for a in incoming[node]]
            terms.extend((f(k, a), -1.0) for a in outgoing[node])
            terms.extend((x(k, a), -1.0) for a in incoming[node])
            rows.add(terms, 0.0, 0.0)

        terms = [(x(k, a), float(arc_seconds[a])) for a in range(a_count)]
        time_terms.append(terms)
        rows.add(terms + [(t_index, -1.0)], -np.inf, 0.0)

    # 无人机同质：按路线工时非增序编号，消除4!个对称解。
    for k in range(n_uav - 1):
        rows.add(
            time_terms[k] + [(idx, -value) for idx, value in time_terms[k + 1]],
            0.0,
            np.inf,
        )

    constraint = rows.constraint(nvars)
    result_x = None
    if native_highs:
        vendor = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".vendor")
        if vendor not in sys.path:
            sys.path.insert(0, vendor)
        try:
            import highspy
        except ImportError as exc:
            raise RuntimeError(
                "--native-highs 需要项目本地 highspy；请安装到 A 题/.vendor"
            ) from exc

        highs = highspy.Highs()
        highs.setOptionValue("time_limit", float(time_limit))
        highs.setOptionValue("mip_rel_gap", float(mip_gap))
        highs.setOptionValue("presolve", "on")
        highs.setOptionValue("output_flag", True)
        all_cols = np.arange(nvars, dtype=np.int32)
        highs.addVars(nvars, lower, upper)
        highs.changeColsCost(nvars, all_cols, objective)
        integer_cols = np.flatnonzero(integrality).astype(np.int32)
        highs.changeColsIntegrality(
            len(integer_cols), integer_cols, np.ones(len(integer_cols), dtype=np.uint8)
        )
        matrix = constraint.A.tocsr()
        highs.addRows(
            matrix.shape[0],
            np.asarray(constraint.lb, dtype=np.float64),
            np.asarray(constraint.ub, dtype=np.float64),
            matrix.nnz,
            matrix.indptr.astype(np.int32),
            matrix.indices.astype(np.int32),
            matrix.data.astype(np.float64),
        )

        if warm_start_path:
            warm_routes = _load_routes(warm_start_path)
            if len(warm_routes) != n_uav:
                raise ValueError(
                    f"热启动含 {len(warm_routes)} 条路线，但模型要求 {n_uav} 条"
                )
            prepared = []
            for physical in warm_routes:
                node_route = [0] + [id_to_node[int(pid)] for pid in physical] + [0]
                seconds = sum(arc_seconds[arc_pos[edge]] for edge in zip(node_route, node_route[1:]))
                prepared.append((float(seconds), node_route))
            # 与模型中的 T_0 >= T_1 >= ... 对称性约束保持一致。
            prepared.sort(key=lambda item: item[0], reverse=True)
            warm = np.zeros(nvars, dtype=np.float64)
            for k, (_, node_route) in enumerate(prepared):
                remaining = len(node_route) - 2
                for edge in zip(node_route, node_route[1:]):
                    a = arc_pos[edge]
                    warm[x(k, a)] += 1.0
                    warm[f(k, a)] += float(remaining)
                    if edge[1] != 0:
                        remaining -= 1
                if remaining != 0:
                    raise AssertionError("热启动连通流构造失败")
            warm[t_index] = max(seconds for seconds, _ in prepared)
            activity = matrix @ warm
            row_violation = max(
                float(np.max(np.maximum(np.asarray(constraint.lb) - activity, 0.0))),
                float(np.max(np.maximum(activity - np.asarray(constraint.ub), 0.0))),
            )
            bound_violation = max(
                float(np.max(np.maximum(lower - warm, 0.0))),
                float(np.max(np.maximum(warm - upper, 0.0))),
            )
            print(
                f"MIP start audit: Tmax={warm[t_index] / 3600.0:.9f} h, "
                f"row_violation={row_violation:.3e}, bound_violation={bound_violation:.3e}"
            )
            if row_violation > 1e-6 or bound_violation > 1e-6:
                raise ValueError("MIP start 未通过模型矩阵审计，拒绝提交")
            highs.setSolution(nvars, all_cols, warm)

        highs.run()
        model_status = highs.getModelStatus()
        status_text = highs.modelStatusToString(model_status)
        info = highs.getInfo()
        solution = highs.getSolution()
        if solution.value_valid:
            candidate_x = np.asarray(solution.col_value, dtype=float)
            if len(candidate_x) == nvars and np.all(np.isfinite(candidate_x)):
                result_x = candidate_x
        objective_h = (
            float(info.objective_function_value)
            if result_x is not None and np.isfinite(info.objective_function_value)
            else None
        )
        dual_h = float(info.mip_dual_bound) if np.isfinite(info.mip_dual_bound) else None
        raw_gap = float(info.mip_gap) if np.isfinite(info.mip_gap) else None
        raw_nodes = int(info.mip_node_count)
        status = int(model_status)
        message = f"HiGHS model status: {status_text}"
        infeasible = model_status == highspy.HighsModelStatus.kInfeasible
    else:
        result = milp(
            c=objective,
            integrality=integrality,
            bounds=Bounds(lower, upper),
            constraints=constraint,
            options={
                "disp": True,
                "time_limit": float(time_limit),
                "mip_rel_gap": float(mip_gap),
                "presolve": True,
            },
        )
        if result.x is not None:
            result_x = np.asarray(result.x, dtype=float)
        objective_h = (
            None if result.fun is None or not np.isfinite(result.fun) else float(result.fun)
        )
        raw_bound = getattr(result, "mip_dual_bound", None)
        dual_h = None if raw_bound is None or not np.isfinite(raw_bound) else float(raw_bound)
        raw_gap = getattr(result, "mip_gap", None)
        raw_nodes = getattr(result, "mip_node_count", None)
        status = int(result.status)
        message = str(result.message)
        infeasible = result.status == 2

    physical_routes = None
    route_hours = None
    conclusion = "candidate_pool_infeasible" if infeasible else "unknown"

    if result_x is not None:
        assigned = np.rint(result_x[:x_count]).astype(int).reshape(n_uav, a_count)
        node_routes = [_euler_route(p, arcs, assigned[k]) for k in range(n_uav)]
        if all(route is not None for route in node_routes):
            complete = [route for route in node_routes if route is not None]
            physical_routes = tuple(
                tuple(int(case.ids[node - 1]) for node in route[1:-1]) for route in complete
            )
            route_hours = tuple(
                float(sum(arc_seconds[a] * assigned[k, a] for a in range(a_count)) / 3600.0)
                for k in range(n_uav)
            )
            actual = Counter(pid for route in physical_routes for pid in route)
            expected = Counter(
                {int(pid): int(demand) for pid, demand in zip(case.ids, case.demand)}
            )
            no_repeat = all(a != b for route in physical_routes for a, b in zip(route, route[1:]))
            audit_ok = (
                actual == expected
                and no_repeat
                and len(physical_routes) == n_uav
                and max(route_hours) <= capacity_hours + 1e-8
            )
            conclusion = "feasible" if audit_ok else "audit_failed"

    return CandidateCertificate(
        case=case.name,
        n_uav=n_uav,
        conclusion=conclusion,
        capacity_hours=float(capacity_hours),
        neighbor_k=int(neighbor_k),
        arc_count=a_count,
        status=status,
        message=message,
        objective_max_hours=objective_h,
        dual_bound_hours=dual_h,
        mip_gap=None if raw_gap is None else float(raw_gap),
        node_count=None if raw_nodes is None else int(raw_nodes),
        total_hours=None if route_hours is None else float(sum(route_hours)),
        max_hours=None if route_hours is None else float(max(route_hours)),
        route_hours=route_hours,
        routes=physical_routes,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3")
    parser.add_argument("--n", type=int, default=4)
    parser.add_argument("--capacity-hours", type=float, default=9.0)
    parser.add_argument("--neighbor-k", type=int, default=8)
    parser.add_argument("--aggregate")
    parser.add_argument("--routes", action="append", default=[])
    parser.add_argument("--warm-start", help="已审计多机路线JSON/TXT；原生HiGHS下作为MIP start")
    parser.add_argument("--native-highs", action="store_true", help="使用A 题/.vendor/highspy原生接口")
    parser.add_argument("--time-limit", type=float, default=300.0)
    parser.add_argument("--mip-gap", type=float, default=0.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    cert = solve(
        args.case,
        args.n,
        capacity_hours=args.capacity_hours,
        neighbor_k=args.neighbor_k,
        aggregate_path=args.aggregate,
        route_paths=args.routes,
        warm_start_path=args.warm_start,
        time_limit=args.time_limit,
        mip_gap=args.mip_gap,
        native_highs=args.native_highs,
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