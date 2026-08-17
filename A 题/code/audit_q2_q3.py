# -*- coding: utf-8 -*-
"""独立审计 A 题问题 2、3 的结果工作簿。

审计内容：
1. UAV 数量、巡检次数与同一物理点非连续约束；
2. 问题 2 的欧氏航程、服务时间和总工期；
3. 问题 3 中逐航段重建“等待直飞/切线绕飞”的最早到达方案；
4. 对重建轨迹的每段飞行、等待和终点服务逐一检查时变圆形管制区；
5. 指标分解 total = flight + service + wait 及汇总数值一致性。

运行：python audit_q2_q3.py
"""

from __future__ import annotations

import math
import os
from collections import Counter
from dataclasses import dataclass
from typing import Sequence

from solve_q1 import BASE_DIR, SCALE_M, SERVICE_S, SPEED_MS, load_case
from solve_q2_q3 import (
    EPS,
    RESULT2,
    RESULT3,
    RouteMetric,
    Zone,
    clearance_after_visit,
    expected_signature,
    load_routes,
    load_zones,
    polyline_metric,
    segment_disc_interval,
    static_metric,
    straight_timed_leg,
    tangent_detours,
)


AUDIT_PATH = os.path.join(BASE_DIR, "docs", "audit_q2_q3.txt")
TOL_TIME = 2e-4
TOL_GEOM = 2e-6


@dataclass(frozen=True)
class Activity:
    kind: str
    start: float
    end: float
    a: tuple[float, float]
    b: tuple[float, float]
    note: str = ""


def _overlap(a0: float, a1: float, b0: float, b1: float) -> bool:
    """半开时间区间是否有正长度交集。"""
    return max(a0, b0) < min(a1, b1) - TOL_TIME


def _point_inside(point: tuple[float, float], zone: Zone) -> bool:
    # 等待在入界点外侧的极限位置，故仅把严格落入圆内判为违规。
    return math.dist(point, (zone.x, zone.y)) < zone.radius - TOL_GEOM


def _travel_violates(activity: Activity, zone: Zone) -> bool:
    interval = segment_disc_interval(activity.a, activity.b, zone)
    if interval is None or activity.end <= activity.start + TOL_TIME:
        return False
    u, v = interval
    enter = activity.start + u * (activity.end - activity.start)
    leave = activity.start + v * (activity.end - activity.start)
    return _overlap(enter, leave, zone.start, zone.end)


def _straight_trace(
    start_time: float,
    a: tuple[float, float],
    b: tuple[float, float],
    destination_service: float,
    zones: Sequence[Zone],
    destination_clearance: dict[str, float] | None = None,
) -> tuple[RouteMetric, list[Activity]]:
    """按求解器的事件规则独立重建一条直线段的完整活动时间线。"""
    duration = math.dist(a, b) * SCALE_M / SPEED_MS
    events = []
    clearance = destination_clearance or {}
    for z in zones:
        interval = segment_disc_interval(a, b, z)
        if interval is None:
            continue
        u, v = interval
        tail = (
            destination_service + clearance.get(z.zid, 0.0)
            if v >= 1.0 - EPS else 0.0
        )
        events.append((u, v, tail, z))
    events.sort(key=lambda item: (item[0], item[1], item[3].start))

    activities: list[Activity] = []
    waiting = 0.0
    last_u = 0.0
    for u, v, tail, z in events:
        enter = start_time + waiting + u * duration
        leave = start_time + waiting + v * duration + tail
        if enter < z.end - EPS and leave > z.start + EPS:
            if u <= EPS:
                return RouteMetric(float("inf"), float("inf"), float("inf"), float("inf")), []
            # 与求解器一致：等待点从入界点后退到所有生效圆之外
            u_wait = u
            seg_len = max(math.dist(a, b), 1e-9)
            for _ in range(10000):
                pw = (a[0] + (b[0] - a[0]) * u_wait, a[1] + (b[1] - a[1]) * u_wait)
                wait_start = start_time + waiting + u_wait * duration
                safe = True
                for z2 in zones:
                    if (wait_start < z2.end - EPS and z.end > z2.start + EPS
                            and math.dist(pw, (z2.x, z2.y)) < z2.radius - EPS):
                        safe = False
                        break
                if safe:
                    break
                u_wait -= 1.0 / seg_len
                if u_wait <= EPS:
                    return RouteMetric(float("inf"), float("inf"), float("inf"), float("inf")), []
            enter_w = start_time + waiting + u_wait * duration
            if u_wait > last_u + EPS:
                p0 = (a[0] + (b[0] - a[0]) * last_u, a[1] + (b[1] - a[1]) * last_u)
                p1 = (a[0] + (b[0] - a[0]) * u_wait, a[1] + (b[1] - a[1]) * u_wait)
                t0 = start_time + waiting + last_u * duration
                activities.append(Activity("flight", t0, enter_w, p0, p1, z.zid))
            p1 = (a[0] + (b[0] - a[0]) * u_wait, a[1] + (b[1] - a[1]) * u_wait)
            delay = z.end - enter_w + EPS
            activities.append(Activity("wait", enter_w, enter_w + delay, p1, p1, z.zid))
            waiting += delay
            last_u = u_wait

    if last_u < 1.0 - EPS:
        p0 = (a[0] + (b[0] - a[0]) * last_u, a[1] + (b[1] - a[1]) * last_u)
        t0 = start_time + waiting + last_u * duration
        activities.append(Activity("flight", t0, start_time + waiting + duration, p0, b))
    elif duration <= EPS:
        activities.append(Activity("flight", start_time + waiting, start_time + waiting, a, b))

    arrival = start_time + waiting + duration
    if destination_service > EPS:
        activities.append(Activity("service", arrival, arrival + destination_service, b, b))
    metric = RouteMetric(duration + destination_service + waiting, duration, destination_service, waiting)

    reference = straight_timed_leg(
        start_time, a, b, destination_service, zones, destination_clearance
    )
    assert abs(metric.total - reference.total) <= TOL_TIME
    assert abs(metric.wait - reference.wait) <= TOL_TIME
    return metric, activities


def _polyline_trace(
    start_time: float,
    points: Sequence[tuple[float, float]],
    destination_service: float,
    zones: Sequence[Zone],
    destination_clearance: dict[str, float] | None = None,
) -> tuple[RouteMetric, list[Activity]]:
    now = start_time
    flight = service = waiting = 0.0
    activities: list[Activity] = []
    for i, (a, b) in enumerate(zip(points, points[1:])):
        serv = destination_service if i == len(points) - 2 else 0.0
        clearance = destination_clearance if serv > 0.0 else None
        metric, trace = _straight_trace(now, a, b, serv, zones, clearance)
        if not math.isfinite(metric.total):
            return metric, []
        now += metric.total
        flight += metric.flight
        service += metric.service
        waiting += metric.wait
        activities.extend(trace)
    return RouteMetric(now - start_time, flight, service, waiting), activities


def _timed_leg_trace(
    start_time: float,
    a: tuple[float, float],
    b: tuple[float, float],
    destination_service: float,
    zones: Sequence[Zone],
    destination_clearance: dict[str, float] | None = None,
) -> tuple[RouteMetric, list[Activity], str]:
    best, trace = _straight_trace(
        start_time, a, b, destination_service, zones, destination_clearance
    )
    label = "straight"
    if best.wait <= EPS:
        return best, trace, label

    for zone in zones:
        if segment_disc_interval(a, b, zone) is None:
            continue
        for side, detour in enumerate(tangent_detours(a, b, zone), 1):
            points = [a, *detour, b]
            candidate = polyline_metric(
                start_time, points, destination_service, zones, destination_clearance
            )
            if candidate.total < best.total - EPS:
                traced, candidate_trace = _polyline_trace(
                    start_time, points, destination_service, zones, destination_clearance
                )
                assert abs(candidate.total - traced.total) <= TOL_TIME
                best, trace = traced, candidate_trace
                label = f"detour:{zone.zid}:side{side}"
    return best, trace, label


def _audit_activity(activity: Activity, zones: Sequence[Zone]) -> list[str]:
    failures = []
    for zone in zones:
        if not _overlap(activity.start, activity.end, zone.start, zone.end):
            continue
        if activity.kind == "flight":
            bad = _travel_violates(activity, zone)
        else:
            bad = _point_inside(activity.a, zone)
        if bad:
            failures.append(
                f"{activity.kind} {activity.start:.3f}-{activity.end:.3f}s "
                f"intersects active {zone.zid} ({zone.start:.1f}-{zone.end:.1f}s)"
            )
    return failures


def _audit_dynamic_route(
    route: Sequence[int], coords: dict[int, tuple[float, float]], zones: Sequence[Zone]
) -> tuple[RouteMetric, int, int, list[str]]:
    now = flight = service = waiting = 0.0
    previous = (0.0, 0.0)
    detours = waits = 0
    failures: list[str] = []
    destinations = [coords[pid] for pid in route] + [(0.0, 0.0)]
    services = [SERVICE_S] * len(route) + [0.0]
    for leg_no, (destination, serv) in enumerate(zip(destinations, services), 1):
        following = destinations[leg_no] if leg_no < len(destinations) else None
        clearance: dict[str, float] | None = None
        if following is not None and serv > 0.0:
            clearance = {}
            for zone in zones:
                value = clearance_after_visit(route, leg_no - 1, coords, zone)
                if value > EPS:
                    clearance[zone.zid] = value
        metric, activities, label = _timed_leg_trace(
            now, previous, destination, serv, zones, clearance
        )
        if not math.isfinite(metric.total):
            failures.append(f"leg {leg_no}: infeasible departure from active-zone interior")
            return metric, detours, waits, failures
        if label.startswith("detour"):
            detours += 1
        waits += sum(a.kind == "wait" and a.end > a.start + TOL_TIME for a in activities)
        for activity in activities:
            for issue in _audit_activity(activity, zones):
                failures.append(f"leg {leg_no} ({label}): {issue}")
        now += metric.total
        flight += metric.flight
        service += metric.service
        waiting += metric.wait
        previous = destination
    metric = RouteMetric(now, flight, service, waiting)
    if abs(metric.total - metric.flight - metric.service - metric.wait) > TOL_TIME:
        failures.append("route metric decomposition mismatch")
    return metric, detours, waits, failures


def _route_constraints(sheet: str, routes: Sequence[Sequence[int]]) -> list[str]:
    failures = []
    expected = Counter(expected_signature(sheet))
    actual = Counter(pid for route in routes for pid in route)
    if actual != expected:
        failures.append(f"coverage mismatch: expected={dict(expected)}, actual={dict(actual)}")
    for uid, route in enumerate(routes, 1):
        for pos, (a, b) in enumerate(zip(route, route[1:]), 1):
            if a == b:
                failures.append(f"UAV {uid}, positions {pos}/{pos+1}: adjacent repeated point {a}")
    return failures


def run_audit() -> str:
    lines = [
        "A题问题2、3独立审计",
        "判据：覆盖次数精确；同点不连续；逐段飞行/等待/服务均不与生效圆区内部相交。",
        "等待位置按入界点外侧极限解释；切线绕飞使用外扩1坐标单位的安全圆。",
        "",
    ]
    all_failures: list[str] = []
    for sheet in ("Case1", "Case2", "Case3", "Case4"):
        points, _ = load_case(sheet)
        coords = {int(pid): (float(x), float(y)) for pid, x, y, _ in points}
        zones = load_zones(sheet)
        routes2 = load_routes(RESULT2, sheet)
        routes3 = load_routes(RESULT3, sheet)

        failures2 = _route_constraints(sheet, routes2)
        failures3 = _route_constraints(sheet, routes3)
        metrics2 = [static_metric(route, coords) for route in routes2]
        metrics3 = []
        total_detours = total_wait_events = 0
        for uid, route in enumerate(routes3, 1):
            metric, detours, waits, route_failures = _audit_dynamic_route(route, coords, zones)
            metrics3.append(metric)
            total_detours += detours
            total_wait_events += waits
            failures3.extend(f"UAV {uid}: {x}" for x in route_failures)

        for problem, routes, metrics, failures in (
            (2, routes2, metrics2, failures2),
            (3, routes3, metrics3, failures3),
        ):
            times = [m.total for m in metrics]
            lines.append(
                f"{sheet} Q{problem}: N={len(routes)}, visits={sum(map(len, routes))}, "
                f"Tmax={max(times)/3600:.6f}h, Tmin={min(times)/3600:.6f}h, "
                f"delta={(max(times)-min(times))/3600:.6f}h, "
                f"coverage={'PASS' if not _route_constraints(sheet, routes) else 'FAIL'}, "
                f"status={'PASS' if not failures else 'FAIL'}"
            )
            if problem == 3:
                lines.append(
                    f"  flight={sum(m.flight for m in metrics)/3600:.6f}h, "
                    f"service={sum(m.service for m in metrics)/3600:.6f}h, "
                    f"wait={sum(m.wait for m in metrics)/3600:.6f}h, "
                    f"detour_legs={total_detours}, wait_events={total_wait_events}"
                )
            for failure in failures:
                lines.append(f"  FAILURE: {failure}")
                all_failures.append(f"{sheet} Q{problem}: {failure}")

    lines.extend(["", f"OVERALL: {'PASS' if not all_failures else 'FAIL'}"])
    if all_failures:
        lines.append(f"failure_count={len(all_failures)}")
    return "\n".join(lines) + "\n"


def main() -> None:
    report = run_audit()
    os.makedirs(os.path.dirname(AUDIT_PATH), exist_ok=True)
    with open(AUDIT_PATH, "w", encoding="utf-8") as f:
        f.write(report)
    print(report, end="")
    if "OVERALL: FAIL" in report:
        raise SystemExit(1)


if __name__ == "__main__":
    main()