# -*- coding: utf-8 -*-
"""从既有问题一证书热启动任务副本变邻域搜索。

复用 ``solve_q2_q3.py`` 已验证的 2-opt、relocate、swap、块搬移、块交换与
循环切入点邻域；任务只移动而不增删，因此覆盖多重集保持不变。每条候选路线
仍需通过“不允许相邻访问同一物理点”的检查。
"""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from dataclasses import asdict, dataclass
from itertools import combinations
from typing import Callable, Sequence

from solve_q1 import LEVEL_TIMES, load_case
from solve_q2_q3 import RouteMetric, optimise, static_metric, valid_route


@dataclass(frozen=True)
class VnsCertificate:
    case: str
    n_uav: int
    conclusion: str
    max_hours: float
    min_hours: float
    total_hours: float
    route_hours: tuple[float, ...]
    exact_cover: bool
    no_adjacent_repeat: bool
    routes: tuple[tuple[int, ...], ...]


def expected_cover(case: str) -> Counter[int]:
    points, _ = load_case(case)
    return Counter({int(pid): int(LEVEL_TIMES[level]) for pid, _, _, level in points})


def route_cover(routes: list[list[int]]) -> Counter[int]:
    return Counter(pid for route in routes for pid in route)


def load_json_routes(path: str) -> list[list[int]]:
    with open(path, encoding="utf-8") as file:
        payload = json.load(file)
    routes = payload.get("routes")
    if not isinstance(routes, list) or not routes:
        raise ValueError(f"{path} 不含非空 routes 数组")
    return [[int(pid) for pid in route] for route in routes]


def _key(metrics: Sequence[RouteMetric]) -> tuple[float, float, float]:
    times = [metric.total for metric in metrics]
    return max(times), max(times) - min(times), sum(times)


def capacity_key(
    metrics: Sequence[RouteMetric], capacity_seconds: float
) -> tuple[float, float, float, float]:
    """严格容量可行性词典序目标：总超载、最大超载、峰值、总工时。"""
    times = [metric.total for metric in metrics]
    overloads = [max(0.0, value - capacity_seconds) for value in times]
    return sum(overloads), max(overloads), max(times), sum(times)


def _two_opt_descent(
    route: list[int], metric: Callable[[Sequence[int]], RouteMetric]
) -> tuple[list[int], RouteMetric]:
    """对一条路线执行确定性 best-improvement 2-opt。"""
    current = route[:]
    current_metric = metric(current)
    while len(current) >= 3:
        best_route = None
        best_metric = current_metric
        for left in range(len(current) - 1):
            for right in range(left + 1, len(current)):
                candidate = current[:left] + list(reversed(current[left : right + 1])) + current[right + 1 :]
                if not valid_route(candidate):
                    continue
                candidate_metric = metric(candidate)
                if candidate_metric.total < best_metric.total - 1e-8:
                    best_route, best_metric = candidate, candidate_metric
        if best_route is None:
            break
        current, current_metric = best_route, best_metric
    return current, current_metric


def _bottleneck_pair_descent(
    routes: list[list[int]],
    metrics: list[RouteMetric],
    metric: Callable[[Sequence[int]], RouteMetric],
    *,
    max_block: int = 3,
    key_fn: Callable[[Sequence[RouteMetric]], tuple[float, ...]] = _key,
    all_pairs: bool = False,
) -> tuple[list[list[int]], list[RouteMetric], bool]:
    """对最忙路线相关的路线对执行一次 best-improvement 大邻域下降。

    邻域包括跨路线 2-opt*、长度不超过 ``max_block`` 的 Or-opt 搬移，
    以及两条路线间的短块交换。候选仅移动已有任务副本，因此天然保持精确
    覆盖；仍逐条检查相邻重复约束。目标使用 ``_key`` 的严格词典序。
    """
    n_routes = len(routes)
    source = max(range(n_routes), key=lambda k: metrics[k].total)
    current_key = key_fn(metrics)
    best_key = current_key
    best_pair: tuple[int, int, list[int], list[int], RouteMetric, RouteMetric] | None = None

    def consider(a: int, b: int, ra: list[int], rb: list[int]) -> None:
        nonlocal best_key, best_pair
        if not ra or not rb or not valid_route(ra) or not valid_route(rb):
            return
        ma, mb = metric(ra), metric(rb)
        candidate_metrics = metrics[:]
        candidate_metrics[a], candidate_metrics[b] = ma, mb
        candidate_key = key_fn(candidate_metrics)
        if candidate_key < best_key:
            best_key = candidate_key
            best_pair = (a, b, ra, rb, ma, mb)

    # 默认只枚举含当前瓶颈路线的路线对；容量模式可枚举全部无序路线对，
    # 以便在两条非峰值路线之间重聚类并降低全队总工时。
    pairs = (
        list(combinations(range(n_routes), 2))
        if all_pairs
        else [(source, k) for k in sorted(
            (j for j in range(n_routes) if j != source),
            key=lambda j: metrics[j].total,
        )]
    )
    for a, b in pairs:
        ra, rb = routes[a], routes[b]

        # 跨路线 2-opt*：交换两个切点之后的尾链。切点包含端点，但两条新路线
        # 必须非空；这种断边重连可一次移动多个任务并同时改变两条路径边界。
        for cut_a in range(1, len(ra) + 1):
            for cut_b in range(1, len(rb) + 1):
                new_a = ra[:cut_a] + rb[cut_b:]
                new_b = rb[:cut_b] + ra[cut_a:]
                consider(a, b, new_a, new_b)

        # Or-opt：从任一方向搬移长度 2..max_block 的连续任务块；同时考察
        # 原方向与反向插入。单点邻域留给 directed_descent，避免重复计算。
        for donor, receiver in ((a, b), (b, a)):
            rd, rr = routes[donor], routes[receiver]
            for length in range(2, min(max_block, len(rd) - 1) + 1):
                for start in range(len(rd) - length + 1):
                    block = rd[start : start + length]
                    reduced = rd[:start] + rd[start + length :]
                    if not valid_route(reduced):
                        continue
                    for position in range(len(rr) + 1):
                        for inserted_block in (block, list(reversed(block))):
                            expanded = rr[:position] + inserted_block + rr[position:]
                            if donor == a:
                                consider(a, b, reduced, expanded)
                            else:
                                consider(a, b, expanded, reduced)

        # 短块交换：允许不同块长，以跨越单点交换的局部最优台阶。
        for len_a in range(1, min(max_block, len(ra)) + 1):
            for start_a in range(len(ra) - len_a + 1):
                block_a = ra[start_a : start_a + len_a]
                for len_b in range(1, min(max_block, len(rb)) + 1):
                    for start_b in range(len(rb) - len_b + 1):
                        block_b = rb[start_b : start_b + len_b]
                        for insert_a, insert_b in (
                            (block_b, block_a),
                            (list(reversed(block_b)), block_a),
                            (block_b, list(reversed(block_a))),
                        ):
                            new_a = ra[:start_a] + insert_a + ra[start_a + len_a :]
                            new_b = rb[:start_b] + insert_b + rb[start_b + len_b :]
                            consider(a, b, new_a, new_b)

    if best_pair is None:
        return routes, metrics, False
    a, b, ra, rb, ma, mb = best_pair
    improved_routes = [route[:] for route in routes]
    improved_metrics = metrics[:]
    improved_routes[a], improved_routes[b] = ra, rb
    improved_metrics[a], improved_metrics[b] = ma, mb
    return improved_routes, improved_metrics, True


def directed_descent(
    initial: Sequence[Sequence[int]],
    metric: Callable[[Sequence[int]], RouteMetric],
    *,
    rounds: int,
    pair_rounds: int = 0,
    max_block: int = 3,
    key_fn: Callable[[Sequence[RouteMetric]], tuple[float, ...]] = _key,
    all_pairs: bool = False,
) -> tuple[list[list[int]], list[RouteMetric]]:
    """瓶颈定向下降，并穿插路径内 2-opt 与可选的大邻域路线对搜索。"""
    routes = [list(route) for route in initial]
    metrics: list[RouteMetric] = []
    for route in routes:
        improved, value = _two_opt_descent(route, metric)
        routes[len(metrics)] = improved
        metrics.append(value)

    for iteration in range(rounds):
        sources = (
            list(range(len(routes)))
            if all_pairs
            else [max(range(len(routes)), key=lambda k: metrics[k].total)]
        )
        current_key = key_fn(metrics)
        best_key = current_key
        best_routes = None
        best_metrics = None
        for source in sources:
            source_route = routes[source]
            if len(source_route) <= 1:
                continue
            # 优先考察闲路线，但所有接收路线都保留。
            targets = sorted(
                (k for k in range(len(routes)) if k != source),
                key=lambda k: metrics[k].total,
            )
            for target in targets:
                target_route = routes[target]
                for i, pid in enumerate(source_route):
                    reduced = source_route[:i] + source_route[i + 1 :]
                    if not valid_route(reduced):
                        continue
                    reduced_metric = metric(reduced)
                    for position in range(len(target_route) + 1):
                        inserted = target_route[:position] + [pid] + target_route[position:]
                        if not valid_route(inserted):
                            continue
                        inserted_metric = metric(inserted)
                        candidate_metrics = metrics[:]
                        candidate_metrics[source] = reduced_metric
                        candidate_metrics[target] = inserted_metric
                        candidate_key = key_fn(candidate_metrics)
                        if candidate_key < best_key:
                            candidate_routes = [route[:] for route in routes]
                            candidate_routes[source] = reduced
                            candidate_routes[target] = inserted
                            best_key = candidate_key
                            best_routes = candidate_routes
                            best_metrics = candidate_metrics

                    for j, other in enumerate(target_route):
                        swapped_source = source_route[:]
                        swapped_target = target_route[:]
                        swapped_source[i], swapped_target[j] = other, pid
                        if not valid_route(swapped_source) or not valid_route(swapped_target):
                            continue
                        candidate_metrics = metrics[:]
                        candidate_metrics[source] = metric(swapped_source)
                        candidate_metrics[target] = metric(swapped_target)
                        candidate_key = key_fn(candidate_metrics)
                        if candidate_key < best_key:
                            candidate_routes = [route[:] for route in routes]
                            candidate_routes[source] = swapped_source
                            candidate_routes[target] = swapped_target
                            best_key = candidate_key
                            best_routes = candidate_routes
                            best_metrics = candidate_metrics

        if best_routes is None or best_metrics is None:
            break
        routes, metrics = best_routes, best_metrics
        # 搬移改变边界后立即精修两条受影响路线；只接受全局词典序改进。
        polished_routes = [route[:] for route in routes]
        polished_metrics = metrics[:]
        for k, route in enumerate(polished_routes):
            polished_routes[k], polished_metrics[k] = _two_opt_descent(route, metric)
        if key_fn(polished_metrics) <= key_fn(metrics):
            routes, metrics = polished_routes, polished_metrics
        if (iteration + 1) % 5 == 0:
            key = _key(metrics)
            print(
                f"    descent {iteration + 1}: Tmax={key[0] / 3600:.6f} h, "
                f"spread={key[1] / 3600:.6f} h, total={key[2] / 3600:.6f} h"
            )

    for iteration in range(pair_rounds):
        routes, metrics, changed = _bottleneck_pair_descent(
            routes,
            metrics,
            metric,
            max_block=max_block,
            key_fn=key_fn,
            all_pairs=all_pairs,
        )
        if not changed:
            break
        # 大邻域接受后精修所有单路线，并仅在全局词典序不退化时保留。
        polished_routes = [route[:] for route in routes]
        polished_metrics = metrics[:]
        for k, route in enumerate(polished_routes):
            polished_routes[k], polished_metrics[k] = _two_opt_descent(route, metric)
        if key_fn(polished_metrics) <= key_fn(metrics):
            routes, metrics = polished_routes, polished_metrics
        key = _key(metrics)
        print(
            f"    pair {iteration + 1}: Tmax={key[0] / 3600:.6f} h, "
            f"spread={key[1] / 3600:.6f} h, total={key[2] / 3600:.6f} h"
        )
    return routes, metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3", choices=[f"Case{i}" for i in range(1, 5)])
    parser.add_argument("--n", type=int, default=4)
    parser.add_argument("--initial", required=True, help="含 routes 的 JSON 热启动证书")
    parser.add_argument("--iterations", type=int, default=500_000)
    parser.add_argument("--restarts", type=int, default=6)
    parser.add_argument("--balance-weight", type=float, default=0.35)
    parser.add_argument("--descent-rounds", type=int, default=80)
    parser.add_argument("--pair-rounds", type=int, default=12)
    parser.add_argument("--max-block", type=int, default=3)
    parser.add_argument("--seed", type=int, default=20260817)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    points, _ = load_case(args.case)
    coords = {int(pid): (float(x), float(y)) for pid, x, y, _ in points}
    expected = expected_cover(args.case)
    initial = load_json_routes(args.initial)
    if len(initial) != args.n:
        raise ValueError(f"热启动含 {len(initial)} 条路线，而 --n={args.n}")
    if route_cover(initial) != expected:
        raise ValueError("热启动覆盖多重集与题目需求不一致")
    if not all(valid_route(route) for route in initial):
        raise ValueError("热启动含空路线或相邻重复物理点")

    metric = lambda route: static_metric(route, coords)
    before = [metric(route).total / 3600.0 for route in initial]
    print(
        f"initial: Tmax={max(before):.6f} h, Tmin={min(before):.6f} h, "
        f"total={sum(before):.6f} h"
    )
    initial, initial_metrics = directed_descent(
        initial,
        metric,
        rounds=args.descent_rounds,
        pair_rounds=args.pair_rounds,
        max_block=args.max_block,
    )
    key = _key(initial_metrics)
    print(
        f"directed: Tmax={key[0] / 3600:.6f} h, spread={key[1] / 3600:.6f} h, "
        f"total={key[2] / 3600:.6f} h"
    )
    routes, metrics = optimise(
        initial,
        metric,
        seed=args.seed,
        iterations=args.iterations,
        restarts=args.restarts,
        balance_weight=args.balance_weight,
    )
    routes, metrics = directed_descent(
        routes,
        metric,
        rounds=args.descent_rounds,
        pair_rounds=args.pair_rounds,
        max_block=args.max_block,
    )

    exact_cover = route_cover(routes) == expected
    no_adjacent_repeat = len(routes) == args.n and all(valid_route(route) for route in routes)
    hours = tuple(float(value.total / 3600.0) for value in metrics)
    certificate = VnsCertificate(
        case=args.case,
        n_uav=args.n,
        conclusion="feasible" if exact_cover and no_adjacent_repeat else "audit_failed",
        max_hours=max(hours),
        min_hours=min(hours),
        total_hours=sum(hours),
        route_hours=hours,
        exact_cover=exact_cover,
        no_adjacent_repeat=no_adjacent_repeat,
        routes=tuple(tuple(route) for route in routes),
    )
    print("\n", certificate)
    for k, (value, route) in enumerate(zip(hours, routes), 1):
        print(f"UAV{k} ({value:.6f} h): 0 -> " + " -> ".join(map(str, route)) + " -> 0")

    output = os.path.abspath(args.output)
    with open(output, "w", encoding="utf-8") as file:
        json.dump(asdict(certificate), file, ensure_ascii=False, indent=2)
    print(f"证书已写入 {output}")


if __name__ == "__main__":
    main()