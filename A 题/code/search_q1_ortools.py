# -*- coding: utf-8 -*-
"""用 OR-Tools 大邻域搜索问题一的严格多无人机路线证书。

模型把每次规定巡检展开为一个任务副本；同一物理点的不同副本坐标相同，但
显式禁止二者相邻，避免“连续停留却重复计数”。每条弧时间按毫秒向上取整，
所以求解器判定不超过 ``--capacity-hours`` 的路线对原始浮点计时同样可行。
最终结果还会按题目原始公式独立审计覆盖次数、相邻重复与逐机工时。

OR-Tools 安装在 ``A 题/.vendor``，不会修改全局 Python 环境。
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import os
import sys
import time
from dataclasses import asdict, dataclass

import numpy as np

VENDOR = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".vendor")
# OR-Tools 9.14 必须配套 Protobuf 6；先加载当前环境的兼容 NumPy，再临时把
# 私有目录置顶加载 OR-Tools 及其 Protobuf，之后恢复搜索路径，避免私有目录中的
# NumPy/Pandas 版本影响项目原有 SciPy 与数据读取代码。
sys.path.insert(0, VENDOR)
try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
finally:
    sys.path.remove(VENDOR)

from solve_q1 import LEVEL_TIMES, SCALE_M, SERVICE_S, SPEED_MS, load_case


@dataclass(frozen=True)
class RouteCertificate:
    case: str
    n_uav: int
    conclusion: str
    capacity_hours: float
    elapsed_s: float
    solver_status: int
    total_budget_hours: float | None = None
    objective: int | None = None
    total_hours: float | None = None
    max_hours: float | None = None
    route_hours: tuple[float, ...] | None = None
    routes: tuple[tuple[int, ...], ...] | None = None


def _instance(case: str):
    points, copies = load_case(case)
    coords = np.asarray([(0.0, 0.0)] + [(x, y) for _, x, y in copies], dtype=float)
    dist_m = np.sqrt(((coords[:, None, :] - coords[None, :, :]) ** 2).sum(axis=2)) * SCALE_M
    orig = [0] + [int(pid) for pid, _, _ in copies]
    demand = {int(pid): LEVEL_TIMES[level] for pid, _, _, level in points}
    return dist_m, orig, demand


def _read_initial(path: str | None, orig: list[int]) -> list[list[int]] | None:
    """读取文本 ``ROUTES`` 行或本程序 JSON，并映射为唯一任务副本编号。"""
    if not path:
        return None
    if path.lower().endswith(".json"):
        with open(path, encoding="utf-8") as file:
            payload = json.load(file)
        physical_routes = payload.get("routes")
        if physical_routes is None:
            raise ValueError(f"JSON 初始解没有 routes: {path}")
    else:
        with open(path, "rb") as file:
            raw = file.read()
        encoding = "utf-16" if raw.startswith((b"\xff\xfe", b"\xfe\xff")) else "utf-8"
        lines = raw.decode(encoding).splitlines()
        line = next((row for row in lines if row.strip().startswith("ROUTES ")), None)
        if line is not None:
            physical_routes = ast.literal_eval(line.split("ROUTES", 1)[1].strip())
        else:
            # 支持 ``UAV1 (... h): 0 -> 12 -> ... -> 0`` 审计日志。
            route_lines = [row for row in lines if row.strip().startswith("UAV") and "->" in row]
            if not route_lines:
                raise ValueError(f"初始解文件没有 ROUTES 或 UAV 路线行: {path}")
            physical_routes = []
            for row in route_lines:
                chain = row.split(":", 1)[1]
                nodes = [int(token.strip()) for token in chain.split("->")]
                if len(nodes) < 2 or nodes[0] != 0 or nodes[-1] != 0:
                    raise ValueError(f"UAV 路线必须从基地0出发并返回: {row}")
                physical_routes.append(nodes[1:-1])

    available: dict[int, list[int]] = {}
    for node, pid in enumerate(orig[1:], 1):
        available.setdefault(pid, []).append(node)
    used = {pid: 0 for pid in available}
    node_routes: list[list[int]] = []
    for route in physical_routes:
        converted = []
        for raw_pid in route:
            pid = int(raw_pid)
            if pid not in available or used[pid] >= len(available[pid]):
                raise ValueError(f"初始解中点 {pid} 的巡检次数超过需求")
            converted.append(available[pid][used[pid]])
            used[pid] += 1
        node_routes.append(converted)
    missing = {pid: len(nodes) - used[pid] for pid, nodes in available.items() if len(nodes) != used[pid]}
    if missing:
        raise ValueError(f"初始解未覆盖全部任务副本: {missing}")
    return node_routes


def _exact_seconds(route: list[int], dist_m: np.ndarray) -> float:
    nodes = [0] + route + [0]
    travel_s = sum(float(dist_m[a, b]) / SPEED_MS for a, b in zip(nodes, nodes[1:]))
    return len(route) * SERVICE_S + travel_s


def solve(
    case: str,
    n_uav: int,
    *,
    capacity_hours: float,
    total_hours: float | None,
    soft_capacity_hours: float | None,
    soft_penalty: int,
    quadratic_soft_capacity_hours: float | None,
    quadratic_soft_penalty: int,
    time_limit: float,
    span_coefficient: int,
    initial_path: str | None,
    seed: int,
    log_search: bool,
) -> RouteCertificate:
    del seed  # RoutingSearchParameters 当前 Python API 不暴露随机种子字段。
    started = time.monotonic()
    dist_m, orig, demand = _instance(case)
    n_nodes = len(orig)
    manager = pywrapcp.RoutingIndexManager(n_nodes, n_uav, 0)
    routing = pywrapcp.RoutingModel(manager)

    # 每次访问的 5 min 作业时间计在“进入任务点”的弧上；返回基地不加作业。
    # 向上取整到毫秒，确保内部时间绝不低估真实值。
    service_ms = int(round(SERVICE_S * 1000.0))
    transit_ms = np.zeros((n_nodes, n_nodes), dtype=np.int64)
    for i in range(n_nodes):
        for j in range(n_nodes):
            travel_ms = int(math.ceil(float(dist_m[i, j]) / SPEED_MS * 1000.0 - 1e-12))
            transit_ms[i, j] = travel_ms + (service_ms if j != 0 else 0)

    def transit(from_index: int, to_index: int) -> int:
        return int(transit_ms[manager.IndexToNode(from_index), manager.IndexToNode(to_index)])

    transit_index = routing.RegisterTransitCallback(transit)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)
    capacity_ms = int(math.floor(capacity_hours * 3600.0 * 1000.0 + 1e-9))
    routing.AddDimension(transit_index, 0, capacity_ms, True, "Time")
    time_dimension = routing.GetDimensionOrDie("Time")
    time_dimension.SetGlobalSpanCostCoefficient(span_coefficient)
    if total_hours is not None:
        if total_hours <= 0:
            raise ValueError("总工时预算必须为正数")
        total_ms = int(math.floor(total_hours * 3600.0 * 1000.0 + 1e-9))
        routing.solver().Add(
            routing.solver().Sum(
                [time_dimension.CumulVar(routing.End(vehicle)) for vehicle in range(n_uav)]
            )
            <= total_ms
        )
    if soft_capacity_hours is not None:
        if soft_capacity_hours > capacity_hours:
            raise ValueError("软容量不能大于硬容量")
        soft_capacity_ms = int(math.floor(soft_capacity_hours * 3600.0 * 1000.0 + 1e-9))
        for vehicle in range(n_uav):
            time_dimension.SetCumulVarSoftUpperBound(
                routing.End(vehicle), soft_capacity_ms, soft_penalty
            )
    if quadratic_soft_capacity_hours is not None:
        if quadratic_soft_capacity_hours > capacity_hours:
            raise ValueError("二次软容量不能大于硬容量")
        quadratic_bound_ms = int(
            math.floor(quadratic_soft_capacity_hours * 3600.0 * 1000.0 + 1e-9)
        )
        bound_cost = pywrapcp.BoundCost(quadratic_bound_ms, quadratic_soft_penalty)
        for vehicle in range(n_uav):
            time_dimension.SetQuadraticCostSoftSpanUpperBoundForVehicle(
                bound_cost, vehicle
            )

    # 同一物理点的副本不能直接相接，否则连续停留只能算一次巡检。
    groups: dict[int, list[int]] = {}
    for node, pid in enumerate(orig[1:], 1):
        groups.setdefault(pid, []).append(node)
    for nodes in groups.values():
        if len(nodes) < 2:
            continue
        for source in nodes:
            source_index = manager.NodeToIndex(source)
            for target in nodes:
                if source != target:
                    routing.NextVar(source_index).RemoveValue(manager.NodeToIndex(target))

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromMilliseconds(max(1, int(round(time_limit * 1000.0))))
    params.log_search = log_search
    params.use_full_propagation = True

    initial_routes = _read_initial(initial_path, orig)
    assignment = None
    if initial_routes is not None:
        if len(initial_routes) != n_uav:
            raise ValueError(f"初始解有 {len(initial_routes)} 条路线，但 n_uav={n_uav}")
        initial = routing.ReadAssignmentFromRoutes(initial_routes, True)
        if initial is not None:
            print("已载入任务副本热启动解")
            assignment = routing.SolveFromAssignmentWithParameters(initial, params)
        else:
            print("热启动解不满足当前硬约束，改用自动构造初始解")
    if assignment is None:
        assignment = routing.SolveWithParameters(params)

    elapsed = time.monotonic() - started
    status = int(routing.status())
    if assignment is None:
        return RouteCertificate(
            case,
            n_uav,
            "unknown",
            capacity_hours,
            elapsed,
            status,
            total_budget_hours=total_hours,
        )

    node_routes: list[list[int]] = []
    for vehicle in range(n_uav):
        route = []
        index = routing.Start(vehicle)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node != 0:
                route.append(node)
            index = assignment.Value(routing.NextVar(index))
        node_routes.append(route)

    # 与求解器解码独立的严格审计。
    flattened = [node for route in node_routes for node in route]
    exact_once = sorted(flattened) == list(range(1, n_nodes))
    no_adjacent_repeat = all(
        orig[a] != orig[b] for route in node_routes for a, b in zip(route, route[1:])
    )
    realized = {pid: 0 for pid in demand}
    for node in flattened:
        realized[orig[node]] += 1
    exact_cover = realized == demand
    route_hours = [_exact_seconds(route, dist_m) / 3600.0 for route in node_routes]
    within_capacity = bool(route_hours) and max(route_hours) <= capacity_hours + 1e-9
    within_total_budget = total_hours is None or sum(route_hours) <= total_hours + 1e-9
    conclusion = (
        "feasible"
        if exact_once
        and no_adjacent_repeat
        and exact_cover
        and within_capacity
        and within_total_budget
        else "audit_failed"
    )
    physical_routes = tuple(tuple(orig[node] for node in route) for route in node_routes)
    return RouteCertificate(
        case,
        n_uav,
        conclusion,
        capacity_hours,
        elapsed,
        status,
        total_hours,
        int(assignment.ObjectiveValue()),
        float(sum(route_hours)),
        float(max(route_hours)),
        tuple(float(v) for v in route_hours),
        physical_routes,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3", choices=[f"Case{i}" for i in range(1, 5)])
    parser.add_argument("--n", type=int, default=4)
    parser.add_argument("--capacity-hours", type=float, default=10.3)
    parser.add_argument(
        "--total-hours",
        type=float,
        help="可选全体无人机总工时硬预算（小时），用于在低总工时邻域内压缩最大工时",
    )
    parser.add_argument(
        "--soft-capacity-hours",
        type=float,
        help="可选逐机软上限（小时）；超限量按 --soft-penalty 计入目标",
    )
    parser.add_argument("--soft-penalty", type=int, default=100)
    parser.add_argument(
        "--quadratic-soft-capacity-hours",
        type=float,
        help="可选逐机二次软上限（小时），用于同时压缩总超载并抑制超载集中",
    )
    parser.add_argument("--quadratic-soft-penalty", type=int, default=1)
    parser.add_argument("--time-limit", type=float, default=300.0)
    parser.add_argument("--span-coefficient", type=int, default=100)
    parser.add_argument("--initial", help="含 ROUTES 行的可选热启动日志")
    parser.add_argument("--seed", type=int, default=20260817)
    parser.add_argument("--log-search", action="store_true")
    parser.add_argument("--output", help="可选 JSON 证书路径")
    args = parser.parse_args()
    cert = solve(
        args.case,
        args.n,
        capacity_hours=args.capacity_hours,
        total_hours=args.total_hours,
        soft_capacity_hours=args.soft_capacity_hours,
        soft_penalty=args.soft_penalty,
        quadratic_soft_capacity_hours=args.quadratic_soft_capacity_hours,
        quadratic_soft_penalty=args.quadratic_soft_penalty,
        time_limit=args.time_limit,
        span_coefficient=args.span_coefficient,
        initial_path=args.initial,
        seed=args.seed,
        log_search=args.log_search,
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