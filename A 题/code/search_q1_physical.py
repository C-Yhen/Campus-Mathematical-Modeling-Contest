# -*- coding: utf-8 -*-
"""问题一物理点覆盖启发式。

与 ``solve_q1.py`` 的任务副本编码不同，本程序直接令每个物理点出现在
``demand`` 条不同的无人机路线中。由于候选架数不少于最大需求 3，同一点
的多次巡检没有必要由同一架无人机绕行后重复访问；该表示能显著缩小搜索
空间。算法采用多起点角度分簇、最便宜插入、路径内 2-opt，以及带退火接受
准则的跨路径搬迁/交换。

示例：
    python search_q1_physical.py --case Case3 --n 4 --restarts 20 --iters 80000
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
from dataclasses import dataclass

import numpy as np

from solve_q1 import LEVEL_TIMES, SCALE_M, SERVICE_S, SPEED_MS, load_case


@dataclass(frozen=True)
class PhysicalSolution:
    case: str
    n_uav: int
    max_hours: float
    min_hours: float
    total_hours: float
    route_hours: tuple[float, ...]
    routes: tuple[tuple[int, ...], ...]


def _data(sheet: str):
    points, _ = load_case(sheet)
    ids = np.asarray([p[0] for p in points], dtype=int)
    xy = np.asarray([(p[1], p[2]) for p in points], dtype=float) * SCALE_M
    demand = np.asarray([LEVEL_TIMES[p[3]] for p in points], dtype=int)
    all_xy = np.vstack((np.zeros((1, 2)), xy))
    dist = np.sqrt(((all_xy[:, None, :] - all_xy[None, :, :]) ** 2).sum(axis=2))
    return ids, xy, demand, dist


def _route_seconds(route: list[int], dist: np.ndarray) -> float:
    if not route:
        return 0.0
    nodes = [0] + [p + 1 for p in route] + [0]
    travel_m = sum(dist[a, b] for a, b in zip(nodes, nodes[1:]))
    return len(route) * SERVICE_S + travel_m / SPEED_MS


def _best_insert(route: list[int], point: int, dist: np.ndarray) -> list[int]:
    node = point + 1
    best_pos = 0
    best_delta = math.inf
    for pos in range(len(route) + 1):
        left = 0 if pos == 0 else route[pos - 1] + 1
        right = 0 if pos == len(route) else route[pos] + 1
        delta = dist[left, node] + dist[node, right] - dist[left, right]
        if delta < best_delta:
            best_delta = delta
            best_pos = pos
    return route[:best_pos] + [point] + route[best_pos:]


def _two_opt(route: list[int], dist: np.ndarray, rounds: int = 30) -> list[int]:
    r = route[:]
    n = len(r)
    if n < 4:
        return r
    for _ in range(rounds):
        improved = False
        for a in range(n - 1):
            left = 0 if a == 0 else r[a - 1] + 1
            first = r[a] + 1
            for b in range(a + 1, n):
                last = r[b] + 1
                right = 0 if b == n - 1 else r[b + 1] + 1
                delta = dist[left, last] + dist[first, right] - dist[left, first] - dist[last, right]
                if delta < -1e-8:
                    r[a : b + 1] = reversed(r[a : b + 1])
                    improved = True
        if not improved:
            break
    return r


def _initial_routes(
    xy: np.ndarray,
    demand: np.ndarray,
    dist: np.ndarray,
    n_uav: int,
    rng: random.Random,
) -> list[list[int]]:
    """按极角中心生成重叠扇区，再以最便宜插入确定各路线次序。"""
    angle = np.arctan2(xy[:, 1], xy[:, 0])
    amin, amax = float(angle.min()), float(angle.max())
    span = max(amax - amin, 1e-6)
    jitter = rng.uniform(-0.20, 0.20) * span / n_uav
    centers = amin + (np.arange(n_uav) + 0.5) * span / n_uav + jitter

    memberships: list[list[int]] = [[] for _ in range(n_uav)]
    load = np.zeros(n_uav)
    order = list(range(len(demand)))
    rng.shuffle(order)
    order.sort(key=lambda p: (-int(demand[p]), float(angle[p])))
    for p in order:
        # 高需求点分给若干条不同路线；角度相近与当前服务负载同时参与选择。
        angular = np.abs(centers - angle[p]) / span
        noise = np.asarray([rng.random() for _ in range(n_uav)]) * 0.035
        chosen: list[int] = []
        for _ in range(int(demand[p])):
            score = angular + 0.16 * load / max(1.0, float(demand.sum()) / n_uav) + noise
            for k in chosen:
                score[k] = math.inf
            k = int(np.argmin(score))
            chosen.append(k)
            load[k] += 1
        for k in chosen:
            memberships[k].append(p)

    routes: list[list[int]] = []
    for members in memberships:
        route: list[int] = []
        # 远点优先插入可避免最近邻在末端形成很差的返航边。
        for p in sorted(members, key=lambda q: dist[0, q + 1], reverse=True):
            route = _best_insert(route, p, dist)
        routes.append(_two_opt(route, dist, rounds=50))
    return routes


def _score(times: list[float]) -> float:
    # 最大工时是主目标，小权重总工时帮助跳出“均衡但四条路线都很长”的解。
    return max(times) + 0.055 * sum(times)


def _audit(routes: list[list[int]], demand: np.ndarray, n_uav: int) -> None:
    counts = np.zeros(len(demand), dtype=int)
    for route in routes:
        if len(route) != len(set(route)):
            raise AssertionError("同一物理点在一条路线中重复出现")
        for p in route:
            counts[p] += 1
    if len(routes) != n_uav or not np.array_equal(counts, demand):
        raise AssertionError("路线覆盖次数与需求不一致")


def search(
    sheet: str,
    n_uav: int,
    *,
    restarts: int = 16,
    iterations: int = 60000,
    seed: int = 20260817,
    stop_hours: float | None = 9.0,
) -> PhysicalSolution:
    ids, xy, demand, dist = _data(sheet)
    if n_uav < int(demand.max()):
        raise ValueError(f"N={n_uav} 小于单点最大巡检次数 {int(demand.max())}")

    global_best_routes = None
    global_best_times = None
    global_best_key = None

    for restart in range(restarts):
        rng = random.Random(seed + restart * 1009)
        routes = _initial_routes(xy, demand, dist, n_uav, rng)
        route_sets = [set(r) for r in routes]
        times = [_route_seconds(r, dist) for r in routes]
        current_score = _score(times)
        local_best_routes = [r[:] for r in routes]
        local_best_times = times[:]
        local_best_key = (max(times), sum(times))

        temperature = 900.0
        for it in range(iterations):
            # 更多地从最忙路线取点，但保留随机选择以允许绕开局部障碍。
            if rng.random() < 0.72:
                a = int(np.argmax(times))
            else:
                a = rng.randrange(n_uav)
            b = rng.randrange(n_uav - 1)
            if b >= a:
                b += 1
            if not routes[a]:
                continue

            cand_a = None
            cand_b = None
            if rng.random() < 0.64:
                candidates = [p for p in routes[a] if p not in route_sets[b]]
                if not candidates:
                    continue
                p = rng.choice(candidates)
                cand_a = routes[a][:]
                cand_a.remove(p)
                cand_b = _best_insert(routes[b], p, dist)
            else:
                pa = [p for p in routes[a] if p not in route_sets[b]]
                pb = [p for p in routes[b] if p not in route_sets[a]]
                if not pa or not pb:
                    continue
                p, q = rng.choice(pa), rng.choice(pb)
                cand_a = routes[a][:]
                cand_b = routes[b][:]
                cand_a.remove(p)
                cand_b.remove(q)
                cand_a = _best_insert(cand_a, q, dist)
                cand_b = _best_insert(cand_b, p, dist)

            ta = _route_seconds(cand_a, dist)
            tb = _route_seconds(cand_b, dist)
            cand_times = times[:]
            cand_times[a], cand_times[b] = ta, tb
            cand_score = _score(cand_times)
            delta = cand_score - current_score
            if delta <= 0.0 or rng.random() < math.exp(-delta / max(temperature, 1e-9)):
                routes[a], routes[b] = cand_a, cand_b
                route_sets[a], route_sets[b] = set(cand_a), set(cand_b)
                times = cand_times
                current_score = cand_score

            temperature *= 0.99992
            if (it + 1) % 2500 == 0:
                routes = [_two_opt(r, dist, rounds=15) for r in routes]
                route_sets = [set(r) for r in routes]
                times = [_route_seconds(r, dist) for r in routes]
                current_score = _score(times)
                temperature = max(temperature, 80.0)

            key = (max(times), sum(times))
            if key < local_best_key:
                local_best_key = key
                local_best_routes = [r[:] for r in routes]
                local_best_times = times[:]

        routes = [_two_opt(r, dist, rounds=80) for r in local_best_routes]
        times = [_route_seconds(r, dist) for r in routes]
        _audit(routes, demand, n_uav)
        key = (max(times), sum(times))
        print(
            f"restart {restart + 1:>2}/{restarts}: "
            f"Tmax={max(times) / 3600:.6f} h, total={sum(times) / 3600:.6f} h"
        )
        if global_best_key is None or key < global_best_key:
            global_best_key = key
            global_best_routes = [r[:] for r in routes]
            global_best_times = times[:]
        if stop_hours is not None and max(global_best_times) <= stop_hours * 3600 + 1e-7:
            break

    assert global_best_routes is not None and global_best_times is not None
    _audit(global_best_routes, demand, n_uav)
    routes_id = tuple(tuple(int(ids[p]) for p in r) for r in global_best_routes)
    hours = tuple(float(t / 3600.0) for t in global_best_times)
    return PhysicalSolution(
        case=sheet,
        n_uav=n_uav,
        max_hours=max(hours),
        min_hours=min(hours),
        total_hours=sum(hours),
        route_hours=hours,
        routes=routes_id,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3", choices=[f"Case{i}" for i in range(1, 5)])
    parser.add_argument("--n", type=int, default=4)
    parser.add_argument("--restarts", type=int, default=16)
    parser.add_argument("--iters", type=int, default=60000)
    parser.add_argument("--seed", type=int, default=20260817)
    parser.add_argument("--output", help="可选 JSON 证书路径")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    solution = search(
        args.case,
        args.n,
        restarts=args.restarts,
        iterations=args.iters,
        seed=args.seed,
    )
    print(solution)
    for k, (hours, route) in enumerate(zip(solution.route_hours, solution.routes), 1):
        print(f"UAV{k} ({hours:.6f} h): " + " -> ".join(map(str, route)))
    if args.output:
        output = os.path.abspath(args.output)
        with open(output, "w", encoding="utf-8") as f:
            json.dump(solution.__dict__, f, ensure_ascii=False, indent=2)
        print(f"证书已写入 {output}")


if __name__ == "__main__":
    main()