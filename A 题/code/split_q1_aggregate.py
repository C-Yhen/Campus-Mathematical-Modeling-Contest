# -*- coding: utf-8 -*-
"""把聚合最优欧拉序列切分为若干条基地闭合路线，用于构造问题一可行证书。"""

from __future__ import annotations

import ast
import argparse
import math
import os
import random

import numpy as np

from solve_q1 import LEVEL_TIMES, SERVICE_S, SPEED_MS, SCALE_M, load_case


def _load_aggregate(path: str) -> list[list[int]]:
    # Windows PowerShell 5 的 ``>``/``Tee-Object`` 默认会生成 UTF-16LE 日志；
    # Python/现代终端通常生成 UTF-8，因此按 BOM 自动兼容两者。
    with open(path, "rb") as file:
        raw = file.read()
    encoding = "utf-16" if raw.startswith((b"\xff\xfe", b"\xfe\xff")) else "utf-8"
    line = next(row for row in raw.decode(encoding).splitlines() if "best_routes=" in row)
    text = line.split("best_routes=", 1)[1].rstrip()
    # 最后一个右括号属于 TotalTimeCertificate，而不是 best_routes 元组。
    routes = ast.literal_eval(text[:-1])
    return [[int(pid) for pid in route if int(pid) != 0] for route in routes]


def _data(case: str):
    points, _ = load_case(case)
    ids = np.asarray([p[0] for p in points], dtype=int)
    xy = np.asarray([(p[1], p[2]) for p in points], dtype=float) * SCALE_M
    all_xy = np.vstack((np.zeros((1, 2)), xy))
    dist = np.sqrt(((all_xy[:, None, :] - all_xy[None, :, :]) ** 2).sum(axis=2))
    demand = {int(pid): LEVEL_TIMES[level] for pid, _, _, level in points}
    id_to_node = {int(pid): i + 1 for i, pid in enumerate(ids)}
    return dist, demand, id_to_node


def _seconds(route: list[int], dist: np.ndarray, node: dict[int, int]) -> float:
    if not route:
        return 0.0
    nodes = [0] + [node[pid] for pid in route] + [0]
    travel = sum(dist[a, b] for a, b in zip(nodes, nodes[1:]))
    return len(route) * SERVICE_S + travel / SPEED_MS


def _insert_extra(route: list[int], pid: int, dist: np.ndarray, node: dict[int, int], rng: random.Random) -> None:
    candidates = []
    v = node[pid]
    for pos in range(len(route) + 1):
        if (pos and route[pos - 1] == pid) or (pos < len(route) and route[pos] == pid):
            continue
        left = 0 if pos == 0 else node[route[pos - 1]]
        right = 0 if pos == len(route) else node[route[pos]]
        delta = dist[left, v] + dist[v, right] - dist[left, right]
        candidates.append((float(delta), rng.random(), pos))
    # 在若干便宜位置中随机选取，产生不同的可切分结构。
    candidates.sort()
    _, _, pos = candidates[rng.randrange(min(12, len(candidates)))]
    route.insert(pos, pid)


def _best_partition(sequence: list[int], n_uav: int, dist: np.ndarray, node: dict[int, int]):
    m = len(sequence)
    cost = np.full((m, m), np.inf)
    for i in range(m):
        travel = dist[0, node[sequence[i]]]
        for j in range(i, m):
            if j > i:
                travel += dist[node[sequence[j - 1]], node[sequence[j]]]
            cost[i, j] = (j - i + 1) * SERVICE_S + (travel + dist[node[sequence[j]], 0]) / SPEED_MS

    dp = np.full((n_uav + 1, m + 1), np.inf)
    parent = np.full((n_uav + 1, m + 1), -1, dtype=int)
    dp[0, 0] = 0.0
    for k in range(1, n_uav + 1):
        for end in range(k, m + 1):
            for start in range(k - 1, end):
                value = max(dp[k - 1, start], cost[start, end - 1])
                if value < dp[k, end]:
                    dp[k, end] = value
                    parent[k, end] = start
    routes = []
    end = m
    for k in range(n_uav, 0, -1):
        start = int(parent[k, end])
        routes.append(sequence[start:end])
        end = start
    routes.reverse()
    return float(dp[n_uav, m]), routes


def search(case: str, n_uav: int, aggregate_path: str, trials: int, seed: int):
    aggregate = _load_aggregate(aggregate_path)
    dist, demand, node = _data(case)
    main_index = int(np.argmax([len(r) for r in aggregate]))
    base = aggregate[main_index]
    extras = [pid for k, route in enumerate(aggregate) if k != main_index for pid in route]
    best = (math.inf, None)
    rng = random.Random(seed)
    for trial in range(trials):
        sequence = base[:]
        rng.shuffle(extras)
        for pid in extras:
            _insert_extra(sequence, pid, dist, node, rng)
        # 基地已把聚合主序列的首尾固定；同时尝试反向，额外插入的随机性负责多样化。
        for candidate in (sequence, list(reversed(sequence))):
            value, routes = _best_partition(candidate, n_uav, dist, node)
            if value < best[0]:
                best = value, [r[:] for r in routes]
                times = [_seconds(r, dist, node) / 3600.0 for r in routes]
                print(f"trial={trial + 1} Tmax={max(times):.6f}h total={sum(times):.6f}h visits={[len(r) for r in routes]}")

    value, routes = best
    assert routes is not None
    realized = {pid: 0 for pid in demand}
    for route in routes:
        for a, b in zip(route, route[1:]):
            if a == b:
                raise AssertionError("路线含相邻重复点")
        for pid in route:
            realized[pid] += 1
    if realized != demand:
        raise AssertionError("巡检次数不匹配")
    times = [_seconds(r, dist, node) / 3600.0 for r in routes]
    print("\nFINAL", times, "Tmax=", max(times), "total=", sum(times))
    for k, (hours, route) in enumerate(zip(times, routes), 1):
        print(f"UAV{k} ({hours:.6f} h): 0 -> " + " -> ".join(map(str, route)) + " -> 0")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3")
    parser.add_argument("--n", type=int, default=4)
    parser.add_argument("--aggregate", default=os.path.join(os.path.dirname(__file__), "..", "docs", "aggregate_case3_n4_routes.txt"))
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260817)
    args = parser.parse_args()
    search(args.case, args.n, args.aggregate, args.trials, args.seed)


if __name__ == "__main__":
    main()