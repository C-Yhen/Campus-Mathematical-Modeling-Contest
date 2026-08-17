# -*- coding: utf-8 -*-
"""A 题问题 2、3：负载均衡与分时圆形禁飞区下的多无人机调度。

问题 2 固定问题 1 已采用的机队规模，以 (Tmax, Tmax-Tmin) 为词典序目标；
问题 3 仍固定该机队规模，从 08:00 同时开始执行。对每条航段进行连续时间
事件仿真，并在“入界前等待”和“沿圆外切线—安全圆弧绕飞”之间选较早到达者。

算法是多起点模拟退火/变邻域搜索，任务副本始终只移动、不增删，因此覆盖次数
保持不变；相邻同一物理点的方案被判为非法。输出 result2.xlsx、result3.xlsx，
并生成 docs/q2_q3_summary.txt 供独立核验。

运行：python solve_q2_q3.py
依赖：openpyxl（以及 solve_q1.py 的数据读取常量）
"""

from __future__ import annotations

import math
import os
import random
from dataclasses import dataclass
from datetime import time as dt_time
from functools import lru_cache
from typing import Callable, Iterable, Sequence

import openpyxl

from solve_q1 import (
    BASE_DIR,
    LEVEL_TIMES,
    SCALE_M,
    SERVICE_S,
    SPEED_MS,
    load_case,
)


RESULT1 = os.path.join(BASE_DIR, "result1.xlsx")
RESULT2 = os.path.join(BASE_DIR, "result2.xlsx")
RESULT3 = os.path.join(BASE_DIR, "result3.xlsx")
DATA2 = os.path.join(BASE_DIR, "附件2.xlsx")
SUMMARY = os.path.join(BASE_DIR, "docs", "q2_q3_summary.txt")
EPS = 1e-7
INF = 1e100


@dataclass(frozen=True)
class Zone:
    zid: str
    x: float
    y: float
    radius: float
    start: float  # 相对 08:00 的秒数
    end: float


@dataclass(frozen=True)
class RouteMetric:
    total: float
    flight: float
    service: float
    wait: float


def clock_seconds(value) -> float:
    """把 Excel 中的 HH:MM / datetime.time 转成相对 08:00 的秒数。"""
    if isinstance(value, dt_time):
        hour, minute, second = value.hour, value.minute, value.second
    else:
        parts = str(value).strip().split(":")
        hour, minute = int(parts[0]), int(parts[1])
        second = int(float(parts[2])) if len(parts) > 2 else 0
    return (hour - 8) * 3600.0 + minute * 60.0 + second


def load_zones(sheet: str) -> list[Zone]:
    wb = openpyxl.load_workbook(DATA2, data_only=True, read_only=True)
    ws = wb[sheet]
    zones = []
    for zid, x, y, radius, start, end in ws.iter_rows(min_row=2, values_only=True):
        if zid is None:
            continue
        st, en = clock_seconds(start), clock_seconds(end)
        # 附件 Case4 的 Z8 为 17:00--17:00，零长度窗口不构成管制时段。
        if en > st + EPS:
            zones.append(Zone(str(zid), float(x), float(y), float(radius), st, en))
    wb.close()
    return zones


def load_routes(path: str, sheet: str) -> list[list[int]]:
    wb = openpyxl.load_workbook(path, data_only=True, read_only=True)
    ws = wb[sheet]
    routes = []
    for row in ws.iter_rows(min_row=2, values_only=True):
        if row[0] is not None:
            routes.append([int(v) for v in row[1:] if v is not None])
    wb.close()
    return routes


def valid_route(route: Sequence[int]) -> bool:
    return bool(route) and all(a != b for a, b in zip(route, route[1:]))


@lru_cache(maxsize=500_000)
def segment_disc_interval(a: tuple[float, float], b: tuple[float, float],
                          z: Zone) -> tuple[float, float] | None:
    """线段 a->b 位于闭圆内的参数区间 [u,v]，参数范围为 [0,1]。"""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    fx, fy = ax - z.x, ay - z.y
    aa = dx * dx + dy * dy
    cc = fx * fx + fy * fy - z.radius * z.radius
    if aa <= EPS:
        return (0.0, 1.0) if cc <= EPS else None
    bb = 2.0 * (fx * dx + fy * dy)
    disc = bb * bb - 4.0 * aa * cc
    if disc < -EPS:
        return None
    root = math.sqrt(max(0.0, disc))
    lo = max(0.0, (-bb - root) / (2.0 * aa))
    hi = min(1.0, (-bb + root) / (2.0 * aa))
    return (lo, hi) if lo <= hi + EPS else None


def straight_timed_leg(start_time: float, a: tuple[float, float], b: tuple[float, float],
                       destination_service: float, zones: Sequence[Zone],
                       destination_clearance: dict[str, float] | None = None) -> RouteMetric:
    """沿直线航段飞行；必要时在圆区入界前等待，返回到达并完成服务后的指标。

    对每个圆，线段在圆内的飞行时间是连续区间。若该区间（终点在圆内时连同
    终点服务）与管制窗口有交集，就把等待安排在其入界点外。按入界位置排序后
    逐个处理，已增加的等待会自动推迟后续圆区的到达时刻。
    """
    distance_m = math.dist(a, b) * SCALE_M
    duration = distance_m / SPEED_MS
    events = []
    clearance = destination_clearance or {}
    for z in zones:
        interval = segment_disc_interval(a, b, z)
        if interval is None:
            continue
        u, v = interval
        inside_tail = (
            destination_service + clearance.get(z.zid, 0.0)
            if v >= 1.0 - EPS else 0.0
        )
        events.append((u, v, inside_tail, z))
    events.sort(key=lambda item: (item[0], item[1], item[3].start))

    waiting = 0.0
    for u, v, inside_tail, z in events:
        enter = start_time + waiting + u * duration
        leave = start_time + waiting + v * duration + inside_tail
        # 接触边界按禁飞处理；等待至 end+EPS 后进入。
        if enter < z.end - EPS and leave > z.start + EPS:
            # 航段起点已在圆内时，不能把等待安排在圆内；这种时序应由前一
            # 入界航段的“服务+下一段离界前视”提前推迟到解禁后。
            if u <= EPS:
                return RouteMetric(INF, INF, INF, INF)
            waiting += z.end - enter + EPS

    return RouteMetric(
        total=duration + destination_service + waiting,
        flight=duration,
        service=destination_service,
        wait=waiting,
    )


@lru_cache(maxsize=200_000)
def tangent_detours(a: tuple[float, float], b: tuple[float, float],
                    z: Zone) -> tuple[tuple[tuple[float, float], ...], ...]:
    """给出绕单个圆的两条较短切线—离散圆弧候选折线（不含起终点）。"""
    # 1 个坐标单位即 100m；外扩 1 单位可抵消圆弧离散弦的内缩误差。
    radius = z.radius + 1.0
    ca = (a[0] - z.x, a[1] - z.y)
    cb = (b[0] - z.x, b[1] - z.y)
    da, db = math.hypot(*ca), math.hypot(*cb)
    if da <= radius + EPS or db <= radius + EPS:
        return ()
    alpha, beta = math.atan2(ca[1], ca[0]), math.atan2(cb[1], cb[0])
    ta = [alpha - math.acos(radius / da), alpha + math.acos(radius / da)]
    tb = [beta - math.acos(radius / db), beta + math.acos(radius / db)]
    candidates: list[tuple[float, list[tuple[float, float]]]] = []
    max_step = math.pi / 18.0  # 10°，在外扩圆上取弦仍位于原禁飞圆外
    for aa in ta:
        for bb in tb:
            for direction in (1, -1):
                delta = (bb - aa) % (2.0 * math.pi)
                if direction < 0:
                    delta = -((aa - bb) % (2.0 * math.pi))
                steps = max(1, int(math.ceil(abs(delta) / max_step)))
                angles = [aa + delta * k / steps for k in range(steps + 1)]
                path = [(z.x + radius * math.cos(t), z.y + radius * math.sin(t)) for t in angles]
                length = math.dist(a, path[0]) + sum(
                    math.dist(p, q) for p, q in zip(path, path[1:])
                ) + math.dist(path[-1], b)
                candidates.append((length, path))
    candidates.sort(key=lambda item: item[0])
    # 两侧绕行通常对应最短的两个候选；去除数值上重复的路径。
    answer: list[tuple[tuple[float, float], ...]] = []
    signatures = set()
    for _, path in candidates:
        sig = tuple((round(x, 5), round(y, 5)) for x, y in path)
        if sig not in signatures:
            signatures.add(sig)
            answer.append(tuple(path))
        if len(answer) == 2:
            break
    return tuple(answer)


def polyline_metric(start_time: float, points: Sequence[tuple[float, float]],
                    destination_service: float, zones: Sequence[Zone],
                    destination_clearance: dict[str, float] | None = None) -> RouteMetric:
    """用直线事件评价器逐段评价给定折线，服务只在最后一个点发生。"""
    now = start_time
    flight = service = waiting = 0.0
    for i, (a, b) in enumerate(zip(points, points[1:])):
        serv = destination_service if i == len(points) - 2 else 0.0
        clearance = destination_clearance if serv > 0.0 else None
        m = straight_timed_leg(now, a, b, serv, zones, clearance)
        if m.total >= INF / 2:
            return RouteMetric(INF, INF, INF, INF)
        now += m.total
        flight += m.flight
        service += m.service
        waiting += m.wait
    return RouteMetric(now - start_time, flight, service, waiting)


def timed_leg(start_time: float, a: tuple[float, float], b: tuple[float, float],
              destination_service: float, zones: Sequence[Zone],
              destination_clearance: dict[str, float] | None = None) -> RouteMetric:
    """返回等待直飞与切线绕飞候选中的最早到达方案。"""
    best = straight_timed_leg(
        start_time, a, b, destination_service, zones, destination_clearance
    )
    if best.wait <= EPS:
        return best
    for z in zones:
        if segment_disc_interval(a, b, z) is None:
            continue
        for detour in tangent_detours(a, b, z):
            candidate = polyline_metric(
                start_time, [a, *detour, b], destination_service, zones,
                destination_clearance,
            )
            if candidate.total < best.total - EPS:
                best = candidate
    return best


def static_metric(route: Sequence[int], coords: dict[int, tuple[float, float]]) -> RouteMetric:
    if not valid_route(route):
        return RouteMetric(INF, INF, INF, INF)
    flight = 0.0
    previous = (0.0, 0.0)
    for pid in route:
        point = coords[pid]
        flight += math.dist(previous, point) * SCALE_M / SPEED_MS
        previous = point
    flight += math.dist(previous, (0.0, 0.0)) * SCALE_M / SPEED_MS
    service = len(route) * SERVICE_S
    return RouteMetric(flight + service, flight, service, 0.0)


def clearance_after_visit(route: Sequence[int], position: int,
                          coords: dict[int, tuple[float, float]], z: Zone) -> float:
    """抵达 route[position] 并完成本次服务后，沿后续路线首次离开 z 的时间。

    当前点的服务时间由入界航段单独计入；返回值包括后续圆内飞行，以及若后续
    巡检点仍在圆内时的服务时间。该前视用于把整段连续圆内占用统一安排在管制
    窗口之前或之后，避免在圆内任务链中途非法等待。
    """
    clearance = 0.0
    a = coords[route[position]]
    for next_position in range(position + 1, len(route) + 1):
        is_base = next_position == len(route)
        b = (0.0, 0.0) if is_base else coords[route[next_position]]
        duration = math.dist(a, b) * SCALE_M / SPEED_MS
        interval = segment_disc_interval(a, b, z)
        if interval is None or interval[0] > EPS:
            return clearance
        clearance += interval[1] * duration
        if interval[1] < 1.0 - EPS or is_base:
            return clearance
        clearance += SERVICE_S
        a = b
    return clearance


def dynamic_metric(route: Sequence[int], coords: dict[int, tuple[float, float]],
                   zones: Sequence[Zone]) -> RouteMetric:
    if not valid_route(route):
        return RouteMetric(INF, INF, INF, INF)
    now = flight = service = waiting = 0.0
    previous = (0.0, 0.0)
    for position, pid in enumerate(route):
        destination = coords[pid]
        clearance: dict[str, float] = {}
        for z in zones:
            value = clearance_after_visit(route, position, coords, z)
            if value > EPS:
                clearance[z.zid] = value
        m = timed_leg(now, previous, destination, SERVICE_S, zones, clearance)
        if m.total >= INF / 2:
            return RouteMetric(INF, INF, INF, INF)
        now += m.total
        flight += m.flight
        service += m.service
        waiting += m.wait
        previous = destination
    m = timed_leg(now, previous, (0.0, 0.0), 0.0, zones)
    if m.total >= INF / 2:
        return RouteMetric(INF, INF, INF, INF)
    return RouteMetric(now + m.total, flight + m.flight, service, waiting + m.wait)


def objective(times: Sequence[float], balance_weight: float) -> float:
    """主目标 Tmax；次目标极差以较小权重参与搜索。"""
    if not times or max(times) >= INF / 2:
        return INF
    return max(times) + balance_weight * (max(times) - min(times))


def lex_key(times: Sequence[float]) -> tuple[float, float, float]:
    """保存最优解时采用严格词典序，再以方差作第三判据。"""
    mx, mn = max(times), min(times)
    mean = sum(times) / len(times)
    return mx, mx - mn, sum((x - mean) ** 2 for x in times)


def best_cyclic_route(route: Sequence[int],
                      metric: Callable[[Sequence[int]], RouteMetric]) -> tuple[list[int], RouteMetric]:
    """穷举正反向循环切入位置，返回该任务环的最佳起止点。"""
    best_route = list(route)
    best_metric = metric(best_route)
    for oriented in (list(route), list(reversed(route))):
        for cut in range(len(oriented)):
            candidate = oriented[cut:] + oriented[:cut]
            if not valid_route(candidate):
                continue
            candidate_metric = metric(candidate)
            if candidate_metric.total < best_metric.total - EPS:
                best_route, best_metric = candidate, candidate_metric
    return best_route, best_metric


def cyclic_improve(routes: Sequence[Sequence[int]],
                   metric: Callable[[Sequence[int]], RouteMetric]) -> tuple[list[list[int]], list[RouteMetric]]:
    improved = [best_cyclic_route(route, metric) for route in routes]
    return [x[0] for x in improved], [x[1] for x in improved]


def mutate(routes: list[list[int]], rng: random.Random) -> tuple[list[list[int]], set[int]]:
    """生成一个保持任务副本总集合不变的邻域解。"""
    candidate = [r[:] for r in routes]
    n = len(candidate)
    move = rng.random()
    changed: set[int]

    if move < 0.18:  # 路径内 2-opt
        i = rng.randrange(n)
        if len(candidate[i]) < 3:
            return candidate, {i}
        a, b = sorted(rng.sample(range(len(candidate[i])), 2))
        candidate[i][a:b + 1] = reversed(candidate[i][a:b + 1])
        changed = {i}
    elif move < 0.31:  # 路径内 relocate
        i = rng.randrange(n)
        if len(candidate[i]) < 2:
            return candidate, {i}
        a = rng.randrange(len(candidate[i]))
        value = candidate[i].pop(a)
        b = rng.randrange(len(candidate[i]) + 1)
        candidate[i].insert(b, value)
        changed = {i}
    elif move < 0.49:  # 跨路径 relocate
        i, j = rng.sample(range(n), 2)
        if len(candidate[i]) <= 1:
            return candidate, {i, j}
        a = rng.randrange(len(candidate[i]))
        value = candidate[i].pop(a)
        b = rng.randrange(len(candidate[j]) + 1)
        candidate[j].insert(b, value)
        changed = {i, j}
    elif move < 0.64:  # 路径内或跨路径 swap
        i, j = rng.randrange(n), rng.randrange(n)
        if not candidate[i] or not candidate[j]:
            return candidate, {i, j}
        a, b = rng.randrange(len(candidate[i])), rng.randrange(len(candidate[j]))
        candidate[i][a], candidate[j][b] = candidate[j][b], candidate[i][a]
        changed = {i, j}
    elif move < 0.78:  # 成段跨路径 relocate，跨越时间窗目标的台阶
        i, j = rng.sample(range(n), 2)
        if len(candidate[i]) <= 2:
            return candidate, {i, j}
        length = rng.randint(2, min(10, len(candidate[i]) - 1))
        a = rng.randrange(len(candidate[i]) - length + 1)
        block = candidate[i][a:a + length]
        del candidate[i][a:a + length]
        b = rng.randrange(len(candidate[j]) + 1)
        if rng.random() < 0.5:
            block.reverse()
        candidate[j][b:b] = block
        changed = {i, j}
    elif move < 0.90:  # 两条路径交换短任务块
        i, j = rng.sample(range(n), 2)
        li = rng.randint(1, min(6, len(candidate[i])))
        lj = rng.randint(1, min(6, len(candidate[j])))
        a = rng.randrange(len(candidate[i]) - li + 1)
        b = rng.randrange(len(candidate[j]) - lj + 1)
        bi, bj = candidate[i][a:a + li], candidate[j][b:b + lj]
        candidate[i][a:a + li], candidate[j][b:b + lj] = bj, bi
        changed = {i, j}
    else:  # 改变任务环相对基地的切入点，可整体平移到达时序
        i = rng.randrange(n)
        cut = rng.randrange(len(candidate[i]))
        candidate[i] = candidate[i][cut:] + candidate[i][:cut]
        if rng.random() < 0.35:
            candidate[i].reverse()
        changed = {i}
    return candidate, changed


def optimise(initial: Sequence[Sequence[int]], metric: Callable[[Sequence[int]], RouteMetric],
             *, seed: int, iterations: int, restarts: int,
             balance_weight: float) -> tuple[list[list[int]], list[RouteMetric]]:
    """多起点模拟退火；只重算发生变化的 1--2 条路径。"""
    global_routes = [list(r) for r in initial]
    global_metrics = [metric(r) for r in global_routes]
    global_key = lex_key([m.total for m in global_metrics])

    for restart in range(restarts):
        rng = random.Random(seed * 1009 + restart * 9176)
        routes = [r[:] for r in global_routes]
        # 除首轮外进行若干扰动，以建立不同搜索盆地。
        if restart:
            for _ in range(8 + 2 * restart):
                trial, changed = mutate(routes, rng)
                if all(valid_route(trial[k]) for k in changed):
                    routes = trial
        metrics = [metric(r) for r in routes]
        times = [m.total for m in metrics]
        score = objective(times, balance_weight)
        start_temp = max(30.0, 0.015 * max(x for x in times if x < INF / 2))

        for it in range(iterations):
            trial, changed = mutate(routes, rng)
            if any(not valid_route(trial[k]) for k in changed):
                continue
            trial_metrics = metrics[:]
            for k in changed:
                trial_metrics[k] = metric(trial[k])
            trial_times = [m.total for m in trial_metrics]
            trial_score = objective(trial_times, balance_weight)
            # 周期性回温比单调降温更适合跨路径离散重分配。
            phase = (it % max(1, iterations // 4)) / max(1, iterations // 4)
            temp = start_temp * (0.002 ** phase)
            delta = trial_score - score
            if delta <= 0.0 or (delta < 40.0 * temp and rng.random() < math.exp(-delta / temp)):
                routes, metrics, times, score = trial, trial_metrics, trial_times, trial_score
                key = lex_key(times)
                if key < global_key:
                    global_routes = [r[:] for r in routes]
                    global_metrics = metrics[:]
                    global_key = key
        print(
            f"    restart {restart + 1}/{restarts}: "
            f"Tmax={global_key[0] / 3600:.4f}h, spread={global_key[1] / 3600:.4f}h"
        )
    return global_routes, global_metrics


def write_workbook(path: str, results: dict[str, list[list[int]]]) -> None:
    wb = openpyxl.Workbook()
    wb.remove(wb.active)
    for sheet, routes in results.items():
        ws = wb.create_sheet(sheet)
        width = max(len(r) for r in routes)
        ws.append(["UAV ID"] + [f"{i}th Inspection Point" for i in range(1, width + 1)])
        for uid, route in enumerate(routes, 1):
            ws.append([uid] + route)
    wb.save(path)


def cover_signature(routes: Iterable[Sequence[int]]) -> dict[int, int]:
    counts: dict[int, int] = {}
    for route in routes:
        for pid in route:
            counts[pid] = counts.get(pid, 0) + 1
    return counts


def expected_signature(sheet: str) -> dict[int, int]:
    points, _ = load_case(sheet)
    return {int(pid): LEVEL_TIMES[level] for pid, _, _, level in points}


def summary_line(sheet: str, routes: Sequence[Sequence[int]], metrics: Sequence[RouteMetric]) -> str:
    times = [m.total for m in metrics]
    waits = [m.wait for m in metrics]
    assert cover_signature(routes) == expected_signature(sheet)
    assert all(valid_route(r) for r in routes)
    return (
        f"{sheet}: N={len(routes)}, Tmax={max(times)/3600:.6f} h, "
        f"Tmin={min(times)/3600:.6f} h, delta={(max(times)-min(times))/3600:.6f} h, "
        f"total={sum(times)/3600:.6f} h, wait={sum(waits)/3600:.6f} h\n"
        f"  route_hours={[round(x/3600, 6) for x in times]}\n"
        f"  wait_hours={[round(x/3600, 6) for x in waits]}\n"
    )


def main() -> None:
    q2_routes: dict[str, list[list[int]]] = {}
    q2_metrics: dict[str, list[RouteMetric]] = {}
    q3_routes: dict[str, list[list[int]]] = {}
    q3_metrics: dict[str, list[RouteMetric]] = {}

    for idx, sheet in enumerate(("Case1", "Case2", "Case3", "Case4"), 1):
        points, _ = load_case(sheet)
        coords = {int(pid): (float(x), float(y)) for pid, x, y, _ in points}
        initial = load_routes(RESULT1, sheet)
        print(f"[{sheet}] 问题2：静态工期—均衡性词典序搜索")
        metric2 = lambda route, c=coords: static_metric(route, c)
        r2, m2 = optimise(
            initial, metric2, seed=20260816 + idx, iterations=30000,
            restarts=3, balance_weight=0.015,
        )
        q2_routes[sheet], q2_metrics[sheet] = r2, m2

        print(f"[{sheet}] 问题3：分时禁飞区事件仿真与重调度")
        zones = load_zones(sheet)
        metric3 = lambda route, c=coords, z=zones: dynamic_metric(route, c, z)
        cyclic_routes, cyclic_metrics = cyclic_improve(r2, metric3)
        cyclic_times = [m.total for m in cyclic_metrics]
        print(
            f"    循环切入优化：Tmax={max(cyclic_times) / 3600:.4f}h, "
            f"spread={(max(cyclic_times) - min(cyclic_times)) / 3600:.4f}h"
        )
        r3, m3 = optimise(
            cyclic_routes, metric3, seed=20260916 + idx, iterations=50000,
            restarts=4, balance_weight=0.025,
        )
        q3_routes[sheet], q3_metrics[sheet] = r3, m3

    write_workbook(RESULT2, q2_routes)
    write_workbook(RESULT3, q3_routes)
    os.makedirs(os.path.dirname(SUMMARY), exist_ok=True)
    with open(SUMMARY, "w", encoding="utf-8") as f:
        f.write("问题2（固定问题1机队规模；主目标Tmax，次目标delta）\n")
        for sheet in q2_routes:
            f.write(summary_line(sheet, q2_routes[sheet], q2_metrics[sheet]))
        f.write("\n问题3（08:00出发；等待直飞与圆外切线绕飞取较早到达）\n")
        for sheet in q3_routes:
            f.write(summary_line(sheet, q3_routes[sheet], q3_metrics[sheet]))
    print(f"已写入 {RESULT2}\n已写入 {RESULT3}\n已写入 {SUMMARY}")


if __name__ == "__main__":
    main()