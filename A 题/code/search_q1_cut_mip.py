# -*- coding: utf-8 -*-
"""用迭代子环割搜索问题一的严格 9 h 路线证书。

完整的单商品流模型会为每架无人机、每条候选边增加连续流变量。对于
Case3、N=4，这些变量会明显加重根节点松弛。本程序改用 branch-and-cut 的
外循环实现：

1. 先求满足覆盖次数、偶数度、基地度数和 9 h 上限的整数循环覆盖；
2. 检查每架无人机的正边是否全部与基地连通；
3. 对每个不含基地的分量 S 加入

       d_h sum(x[k,e] : e in delta(S)) >= 2 y[k,h],  h in S,

   再重新求解。

其中 ``y[k,h]`` 是无人机 k 在 h 的巡检次数，``d_h`` 是该点总需求。任何
真实闭合路线只要访问 h，就至少以两条边跨越割 delta(S)，且
``y[k,h] <= d_h``，所以上式严格有效。迭代结束后，
每架无人机的整数边集都是含基地的连通无向偶图，可以直接还原为一条欧拉
闭合路线。候选图是完整图的子图，因此：找到可行解可严格证明原问题可行；
候选图不可行则不能证明原问题不可行，需要增加 ``--neighbors``。

示例：
    python search_q1_cut_mip.py --case Case3 --n 4 --neighbors 16 \
        --time-limit 900 --round-time 90
"""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import asdict, dataclass

import numpy as np
from scipy.optimize import Bounds, milp

from certify_q1 import SERVICE_S, SPEED_MS, CaseData, _Rows, _distance_data, load_case
from certify_q1_undirected import _components, _euler_route
from search_q1_sparse_mip import _candidate_edges


@dataclass(frozen=True)
class CutCertificate:
    case: str
    n_uav: int
    neighbors: int
    candidate_edges: int
    conclusion: str
    rounds: int
    dynamic_cuts: int
    initial_cuts: int
    status: int
    message: str
    elapsed_s: float
    objective_hours: float | None = None
    route_hours: tuple[float, ...] | None = None
    routes: tuple[tuple[int, ...], ...] | None = None


def _load_cut_pool(
    path: str | None,
    case: CaseData,
    n_uav: int,
) -> set[tuple[int, frozenset[int], int]]:
    """读取持久化连通割；点集使用 ``CaseData`` 的 1-based 内部编号。"""
    if not path or not os.path.exists(path):
        return set()
    with open(path, encoding="utf-8") as file:
        payload = json.load(file)
    if payload.get("schema_version") != 1:
        raise ValueError(f"不支持的割池格式: {path}")
    expected_ids = [int(pid) for pid in case.ids]
    expected_demand = [int(value) for value in case.demand]
    if (
        payload.get("case") != case.name
        or int(payload.get("n_uav", -1)) != n_uav
        or payload.get("point_ids") != expected_ids
        or payload.get("demand") != expected_demand
    ):
        raise ValueError(f"割池与当前算例或无人机数量不匹配: {path}")

    p = case.point_count
    cuts: set[tuple[int, frozenset[int], int]] = set()
    for item in payload.get("cuts", []):
        k = int(item["uav"])
        subset = frozenset(int(i) for i in item["subset"])
        h = int(item["anchor"])
        if not (0 <= k < n_uav) or not subset or h not in subset:
            raise ValueError(f"割池含非法连通割: {item}")
        if min(subset) < 1 or max(subset) > p:
            raise ValueError(f"割池含越界点编号: {item}")
        cuts.add((k, subset, h))
    return cuts


def _save_cut_pool(
    path: str | None,
    case: CaseData,
    n_uav: int,
    neighbors: int,
    cuts: set[tuple[int, frozenset[int], int]],
) -> None:
    """原子写入割池，避免长时间搜索中断时留下半个 JSON 文件。"""
    if not path:
        return
    output = os.path.abspath(path)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    ordered = sorted(cuts, key=lambda item: (item[0], tuple(sorted(item[1])), item[2]))
    payload = {
        "schema_version": 1,
        "case": case.name,
        "n_uav": n_uav,
        "source_neighbors": neighbors,
        "point_ids": [int(pid) for pid in case.ids],
        "demand": [int(value) for value in case.demand],
        "cuts": [
            {
                "uav": k,
                "subset": sorted(subset),
                "anchor": h,
            }
            for k, subset, h in ordered
        ],
    }
    temporary = output + ".tmp"
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, separators=(",", ":"))
    os.replace(temporary, output)


def solve_with_cuts(
    case: CaseData,
    n_uav: int,
    *,
    neighbors: int = 16,
    time_limit: float = 900.0,
    round_time: float = 90.0,
    max_rounds: int = 40,
    mip_rel_gap: float = 0.0,
    minimize_distance: bool = False,
    cut_pool_path: str | None = None,
) -> CutCertificate:
    """在候选边图上迭代消除断开子环，返回经过审计的严格路线证书。"""
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
    nvars = y_start + y_count

    def x_idx(k: int, e: int) -> int:
        return k * e_count + e

    def y_idx(k: int, i: int) -> int:
        return y_start + k * p + (i - 1)

    objective = np.zeros(nvars)
    lower = np.zeros(nvars)
    upper = np.zeros(nvars)
    integrality = np.ones(nvars, dtype=np.uint8)
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
            if minimize_distance:
                objective[x_idx(k, e)] = edge_hours[e]

    service_h = SERVICE_S / 3600.0
    for k in range(n_uav):
        for i in range(1, p + 1):
            upper[y_idx(k, i)] = int(case.demand[i - 1])

    rows = _Rows()

    # 全部无人机合计恰好完成规定次数。
    for i in range(1, p + 1):
        demand = int(case.demand[i - 1])
        rows.add(((y_idx(k, i), 1.0) for k in range(n_uav)), demand, demand)

    route_terms: list[list[tuple[int, float]]] = []
    visit_terms: list[list[tuple[int, float]]] = []
    for k in range(n_uav):
        # Case3,N=4 中总服务时间已超过 3*9 h，四架必须全部启用。对一般输入，
        # 本搜索器按命令行给定的 N 固定使用 N 架，符合“验证恰好 N 架”的目的。
        rows.add(((x_idx(k, e), 1.0) for e in incident[0]), 2.0, 2.0)

        for i in range(1, p + 1):
            # 无自环且 degree=2*y：同机多次巡检同一点时必须先离开再返回。
            rows.add(
                [(x_idx(k, e), 1.0) for e in incident[i]]
                + [(y_idx(k, i), -2.0)],
                0.0,
                0.0,
            )
        terms = [(x_idx(k, e), edge_hours[e]) for e in range(e_count)]
        terms.extend((y_idx(k, i), service_h) for i in range(1, p + 1))
        route_terms.append(terms)
        rows.add(terms, -np.inf, 9.0)

        visits = [(y_idx(k, i), 1.0) for i in range(1, p + 1)]
        visit_terms.append(visits)

        # 径向有效不等式。用 y_i/d_i 作为访问指示量的线性下界；若同机
        # 承担该点全部 d_i 次，便恢复完整的基地-i往返下界。
        service_terms = [(y_idx(k, h), service_h) for h in range(1, p + 1)]
        for i in range(1, p + 1):
            radial_h = 2.0 * depot_dist[i - 1] / SPEED_MS / 3600.0
            rows.add(
                service_terms
                + [(y_idx(k, i), radial_h / float(case.demand[i - 1]))],
                -np.inf,
                9.0,
            )

    # 同质无人机按巡检次数、再按工时非增排序，削减标签对称性。
    for k in range(n_uav - 1):
        rows.add(
            visit_terms[k] + [(idx, -value) for idx, value in visit_terms[k + 1]],
            0.0,
            np.inf,
        )
        rows.add(
            route_terms[k] + [(idx, -value) for idx, value in route_terms[k + 1]],
            0.0,
            np.inf,
        )

    # 不预置 O(NP) 个邻域割：先快速取得循环覆盖，再只对实际断开的分量
    # 分离连通割。若提供割池，则恢复以往轮次发现的严格有效割。
    seen_cuts = _load_cut_pool(cut_pool_path, case, n_uav)

    def add_connectivity_cut(key: tuple[int, frozenset[int], int]) -> None:
        k, subset, h = key
        crossing = [
            e for e, (i, j) in enumerate(edges) if (i in subset) != (j in subset)
        ]
        demand_h = float(case.demand[h - 1])
        rows.add(
            [(x_idx(k, e), demand_h) for e in crossing]
            + [(y_idx(k, h), -2.0)],
            0.0,
            np.inf,
        )

    for saved_cut in seen_cuts:
        add_connectivity_cut(saved_cut)
    initial_cuts = len(seen_cuts)
    if initial_cuts:
        print(f"从割池恢复 {initial_cuts} 条连通割: {os.path.abspath(cut_pool_path)}")

    started = time.monotonic()
    dynamic_cuts = 0
    last_status = 4
    last_message = "尚未调用求解器"
    last_objective = None

    for round_no in range(1, max_rounds + 1):
        elapsed = time.monotonic() - started
        remaining = time_limit - elapsed
        if remaining <= 0.05:
            break
        this_limit = min(round_time, remaining)
        print(
            f"\nCut-MIP round {round_no}/{max_rounds}: edges={e_count}, "
            f"rows={len(rows.lb)}, cuts={dynamic_cuts}, limit={this_limit:.1f}s"
        )
        result = milp(
            c=objective,
            integrality=integrality,
            bounds=Bounds(lower, upper),
            constraints=rows.constraint(nvars),
            options={
                "disp": True,
                "time_limit": float(this_limit),
                "mip_rel_gap": float(mip_rel_gap),
                "presolve": True,
            },
        )
        last_status = int(result.status)
        last_message = str(result.message)
        if result.fun is not None and np.isfinite(result.fun):
            last_objective = float(result.fun) if minimize_distance else None

        if result.x is None:
            conclusion = "restricted_infeasible" if result.status == 2 else "unknown"
            return CutCertificate(
                case.name,
                n_uav,
                neighbors,
                e_count,
                conclusion,
                round_no,
                dynamic_cuts,
                initial_cuts,
                last_status,
                last_message,
                time.monotonic() - started,
                last_objective,
            )

        # HiGHS 超时也可能返回整数 incumbent；先审计整数性，再检查连通性。
        integer_values = result.x[:nvars]
        if np.max(np.abs(integer_values - np.rint(integer_values))) > 1e-5:
            last_message += "; incumbent 未通过整数性审计"
            break

        bad: list[tuple[int, set[int], np.ndarray]] = []
        all_x: list[np.ndarray] = []
        all_y: list[np.ndarray] = []
        for k in range(n_uav):
            x_values = np.rint(result.x[k * e_count : (k + 1) * e_count])
            y_values = np.rint(result.x[y_start + k * p : y_start + (k + 1) * p])
            all_x.append(x_values)
            all_y.append(y_values)
            active = {i + 1 for i, value in enumerate(y_values) if value > 0.5}
            for component in _components(active, edges, x_values):
                physical = set(component) - {0}
                if physical and 0 not in component:
                    bad.append((k, physical, y_values))

        if not bad:
            routes: list[tuple[int, ...]] = []
            hours: list[float] = []
            for k in range(n_uav):
                route, route_h = _euler_route(case, edges, all_x[k], all_y[k])
                routes.append(route)
                hours.append(route_h)

            realized = {int(pid): 0 for pid in case.ids}
            no_adjacent_repeat = True
            for route in routes:
                previous = None
                for pid in route:
                    if pid != 0:
                        realized[int(pid)] += 1
                        if pid == previous:
                            no_adjacent_repeat = False
                    previous = pid
            expected = {int(pid): int(d) for pid, d in zip(case.ids, case.demand)}
            if realized != expected or not no_adjacent_repeat or max(hours) > 9.0 + 1e-6:
                return CutCertificate(
                    case.name,
                    n_uav,
                    neighbors,
                    e_count,
                    "numerical_audit_failed",
                    round_no,
                    dynamic_cuts,
                    initial_cuts,
                    last_status,
                    last_message,
                    time.monotonic() - started,
                    last_objective,
                )
            return CutCertificate(
                case.name,
                n_uav,
                neighbors,
                e_count,
                "feasible",
                round_no,
                dynamic_cuts,
                initial_cuts,
                last_status,
                last_message,
                time.monotonic() - started,
                float(sum(hours)),
                tuple(float(v) for v in hours),
                tuple(routes),
            )

        added = 0
        print(
            "disconnected components:",
            ", ".join(f"UAV{k + 1}:|S|={len(subset)}" for k, subset, _ in bad),
        )
        for k, subset, y_values in bad:
            frozen = frozenset(subset)
            # 为当前分量内每个实际访问点加入激活割。未来即使任务重新分配，
            # 只要该点仍由同一无人机访问，这个割仍会保持有效并发挥作用。
            for h in subset:
                if y_values[h - 1] < 0.5:
                    continue
                key = (k, frozen, h)
                if key in seen_cuts:
                    continue
                add_connectivity_cut(key)
                seen_cuts.add(key)
                dynamic_cuts += 1
                added += 1
        print(f"added {added} new connectivity cuts")
        _save_cut_pool(cut_pool_path, case, n_uav, neighbors, seen_cuts)
        if added == 0:
            last_message += "; 断开 incumbent 未产生新割"
            break

    return CutCertificate(
        case.name,
        n_uav,
        neighbors,
        e_count,
        "unknown",
        min(max_rounds, round_no if "round_no" in locals() else 0),
        dynamic_cuts,
        initial_cuts,
        last_status,
        last_message,
        time.monotonic() - started,
        last_objective,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="Case3", choices=[f"Case{i}" for i in range(1, 5)])
    parser.add_argument("--n", type=int, required=True)
    parser.add_argument("--neighbors", type=int, default=16)
    parser.add_argument("--time-limit", type=float, default=900.0, help="全部轮次总时间上限（秒）")
    parser.add_argument("--round-time", type=float, default=90.0, help="单轮 MIP 时间上限（秒）")
    parser.add_argument("--max-rounds", type=int, default=40)
    parser.add_argument("--mip-gap", type=float, default=0.0)
    parser.add_argument(
        "--min-distance",
        action="store_true",
        help="每轮最小化飞行时间；默认仅搜索任意 9 h 可行整数解",
    )
    parser.add_argument("--output", help="可选 JSON 证书路径")
    parser.add_argument(
        "--cut-pool",
        help="可选 JSON 连通割池；每轮原子保存，下次运行自动恢复",
    )
    args = parser.parse_args()

    cert = solve_with_cuts(
        load_case(args.case),
        args.n,
        neighbors=args.neighbors,
        time_limit=args.time_limit,
        round_time=args.round_time,
        max_rounds=args.max_rounds,
        mip_rel_gap=args.mip_gap,
        minimize_distance=args.min_distance,
        cut_pool_path=args.cut_pool,
    )
    print("\n", cert)
    if cert.conclusion == "feasible":
        print(f"严格可行证书：{args.case} 在 N={args.n}、单机 9 h 下可行。")
        assert cert.route_hours is not None and cert.routes is not None
        for k, (hours, route) in enumerate(zip(cert.route_hours, cert.routes), 1):
            print(f"UAV{k} ({hours:.6f} h): " + " -> ".join(map(str, route)))
    elif cert.conclusion == "restricted_infeasible":
        print("候选边图不可行；这不构成原问题不可行证明，请增加 --neighbors。")
    else:
        print("给定时间和候选边图下尚未找到严格可行证书。")

    if args.output:
        output = os.path.abspath(args.output)
        with open(output, "w", encoding="utf-8") as file:
            json.dump(asdict(cert), file, ensure_ascii=False, indent=2)
        print(f"证书已写入 {output}")


if __name__ == "__main__":
    main()